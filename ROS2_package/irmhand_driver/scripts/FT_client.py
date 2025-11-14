#!/usr/bin/env python3

# This is based off of the Feetech SDK
import atexit
import logging
import os
import random
import time
from typing import Optional, Sequence, Tuple

import numpy as np
from scservo_sdk import *

# --- Control Table Addresses ---
SMS_STS_TORQUE_ENABLE = 40
SMS_STS_ACC = 41
SMS_STS_GOAL_POSITION_L = 42
SMS_STS_GOAL_SPEED_L = 46
SMS_STS_KP1_L = 50
SMS_STS_KD1_L = 51
SMS_STS_KI1_L = 52
SMS_STS_PRESENT_POSITION_L = 56
SMS_STS_PRESENT_SPEED_L = 58
SMS_STS_PRESENT_CURRENT_L = 69
SMS_STS_PRESENT_VOLTAGE = 62

# --- Data Byte Lengths ---
LEN_PRESENT_POSITION = 2
LEN_PRESENT_SPEED = 2
LEN_PRESENT_CURRENT = 2
LEN_PRESENT_VOLTAGE = 1

# --- Unit Conversions ---
# 4096 ticks per revolution
DEFAULT_POS_SCALE_RAD = 2.0 * np.pi / 4096
DEFAULT_VEL_SCALE_RAD = 0.732 * 2.0 * np.pi / 60.0  # 0.732 rpm per unit -> rad/s
DEFAULT_POS_SCALE_DEG = 360.0 / 4096  # -> degrees
DEFAULT_VEL_SCALE_DEG = 0.732 * 360.0 / 60.0  # 0.732 rpm per unit -> deg/s
DEFAULT_CUR_SCALE = 6.5 / 1000.0  # 6.5mA per unit -> Amperes
DEFAULT_VLT_SCALE = 0.1  # 0.1V per unit -> Volts


def signed_to_unsigned(value: int, size: int) -> int:
    """
    Converts a signed integer to its sign-magnitude representation for
    writing to the servo.
    """
    bit_size = 8 * size
    sign_bit_mask = 1 << (bit_size - 1)

    if value < 0:
        # Get magnitude and set the sign bit
        magnitude = abs(value)
        return sign_bit_mask | magnitude
    else:
        # Positive value is its own representation
        return value


def unsigned_to_signed(value: int, size: int) -> int:
    """
    Converts a value from its sign-magnitude representation as read from
    the servo.
    """
    bit_size = 8 * size
    sign_bit_mask = 1 << (bit_size - 1)
    magnitude_mask = sign_bit_mask - 1

    # Check if the sign bit is set
    if (value & sign_bit_mask) != 0:
        # It's a negative number. Get the magnitude from the remaining bits
        # and return it with a negative sign.
        magnitude = value & magnitude_mask
        return -magnitude
    else:
        # It's a positive number. The value is its own magnitude.
        return value


class FTClient:
    """
    A client for controlling Feetech (FT) smart servos over a serial port.
    Manages connection, communication, and unit conversions.
    """

    # Class-level tracking of open clients for automatic cleanup on exit
    OPEN_CLIENTS = set()

    def __init__(
        self,
        motor_ids: Sequence[int],
        port: str = "/dev/ttyUSB0",
        baudrate: int = 1000000,
        use_degrees: bool = True,
    ):
        self.motor_ids = list(motor_ids)
        self.port_name = port
        self.baudrate = baudrate
        self._is_connected = False
        self.use_degrees = use_degrees

        self.port_handler = PortHandler(self.port_name)
        self.packet_handler = sms_sts(self.port_handler)

        # Set scaling factors based on desired units (deg vs rad)
        if self.use_degrees:
            self.pos_scale = DEFAULT_POS_SCALE_DEG
            self.vel_scale = DEFAULT_VEL_SCALE_DEG
        else:
            self.pos_scale = DEFAULT_POS_SCALE_RAD
            self.vel_scale = DEFAULT_VEL_SCALE_RAD

        self.cur_scale = DEFAULT_CUR_SCALE

        # Initialize efficient GroupSyncRead handlers
        self._pos_vel_reader = FTPosVelReader(self, self.motor_ids)
        self._pos_reader = FTPosReader(self, self.motor_ids)
        self._vel_reader = FTVelReader(self, self.motor_ids)

    @property
    def is_connected(self) -> bool:
        """Returns the connection status of the client."""
        return self._is_connected

    def connect(self):
        """Connects to the FT motors. Does NOT enable torque by default."""
        if self._is_connected:
            logging.warning("Client is already connected.")
            return

        if not self.port_handler.openPort():
            raise OSError(f"Failed to open port: {self.port_name}")
        logging.info("Succeeded to open port: %s", self.port_name)

        if not self.port_handler.setBaudRate(self.baudrate):
            raise OSError(f"Failed to set baudrate to {self.baudrate}")
        logging.info("Succeeded to set baudrate to %d", self.baudrate)

        self._is_connected = True
        FTClient.OPEN_CLIENTS.add(self)

    def disconnect(self):
        """Disconnects from the ft device. Assumes torque has been handled."""
        if not self._is_connected:
            return
        self.port_handler.closePort()
        self._is_connected = False
        if self in FTClient.OPEN_CLIENTS:
            FTClient.OPEN_CLIENTS.remove(self)

    def set_torque_enabled(self, motor_ids: Sequence[int], enabled: bool):
        """Enables or disables torque for the specified motors."""
        self.check_connected()
        for motor_id in motor_ids:
            comm_result, ft_error = self.packet_handler.write1ByteTxRx(
                motor_id, SMS_STS_TORQUE_ENABLE, int(enabled)
            )
            self.handle_packet_result(
                comm_result, ft_error, motor_id, context=f"set_torque_enabled={enabled}"
            )

    def read_pid_gains(self) -> dict:
        """Reads the PID gains (P, I, D) from all motors individually."""
        self.check_connected()
        pid_gains = {}
        logging.info("Reading PID gains from all motors...")
        for motor_id in self.motor_ids:
            # Read 3 bytes starting from the P-gain address (P, D, I)
            data, comm_result, error = self.packet_handler.readTxRx(
                motor_id, SMS_STS_KP1_L, 3
            )
            if self.handle_packet_result(
                comm_result, error, motor_id, "read_pid_gains"
            ):
                if data and len(data) == 3:
                    p, d, i = data
                    pid_gains[motor_id] = {"P": p, "D": d, "I": i}
                else:
                    logging.error(
                        f"[ID:{motor_id:03d}] Failed to get valid PID data packet."
                    )
                    pid_gains[motor_id] = {"P": -1, "D": -1, "I": -1}
            else:
                pid_gains[motor_id] = {"P": -1, "D": -1, "I": -1}
        return pid_gains

    def set_pid_gains(self, p: int, i: int, d: int):
        """
        Sets the P, I, and D gains for all motors using GroupSyncWrite.
        """
        self.check_connected()
        logging.info(f"Setting PID gains for all motors to: P={p}, I={i}, D={d}")

        # Note: Address is P (50), length is 3 (P, D, I)
        pid_sync_writer = GroupSyncWrite(self.packet_handler, SMS_STS_KP1_L, 3)
        # Note the order for the write packet is P, D, I
        param_pid = [p, d, i]

        for motor_id in self.motor_ids:
            addparam_result = pid_sync_writer.addParam(motor_id, param_pid)
            if not addparam_result:
                logging.error(
                    f"[ID:{motor_id:03d}] Failed to add PID gains to sync write packet."
                )

        comm_result = pid_sync_writer.txPacket()
        self.handle_packet_result(comm_result, context="set_pid_gains")
        pid_sync_writer.clearParam()

    def read_pos_vel(self) -> Tuple[np.ndarray, np.ndarray]:
        """Returns the current positions and velocities from all motors."""
        return self._pos_vel_reader.read()

    def read_pos(self) -> np.ndarray:
        """Returns the current positions from all motors."""
        return self._pos_reader.read()

    def read_vel(self) -> np.ndarray:
        """Returns the current velocities from all motors."""
        return self._vel_reader.read()

    def HLSSyncWritePosEx(self, motor_id, position, vel, acc):
        """Helper to format the GroupSyncWrite packet for position."""
        txpacket = [
            acc,
            self.packet_handler.scs_lobyte(position),
            self.packet_handler.scs_hibyte(position),
            self.packet_handler.scs_lobyte(1500),  # Placeholder time, not used by motor
            self.packet_handler.scs_hibyte(1500),  # Placeholder time, not used by motor
            self.packet_handler.scs_lobyte(vel),
            self.packet_handler.scs_hibyte(vel),
        ]
        return self.packet_handler.groupSyncWrite.addParam(motor_id, txpacket)

    def write_desired_pos_simple(self, motor_ids: Sequence[int], positions: np.ndarray):
        """
        Writes desired positions with fixed velocity (1000) and acceleration (0).
        This is a simplified method for basic position control.
        Units (deg or rad) are determined by the `use_degrees` flag.
        """
        self.check_connected()
        assert len(motor_ids) == len(positions)
        for i, motor_id in enumerate(motor_ids):
            pos_ticks = int(positions[i] / self.pos_scale)

            # --- CRITICAL SAFETY CHECK ---
            # Ensure command is within the motor's absolute 0-4096 tick range.
            if not (0 <= pos_ticks <= 4096):
                logging.critical(
                    f"FATAL: Position command for motor ID {motor_id} is {pos_ticks}, which is out of the valid range (0-4096)."
                )
                logging.critical(
                    "Disabling torque on all motors for safety and quitting."
                )
                self.set_torque_enabled(self.motor_ids, False)
                raise ValueError(
                    f"Position command for motor {motor_id} is out of valid range."
                )

            # Use fixed velocity (1000 ticks) and acceleration (0)
            addparam_result = self.HLSSyncWritePosEx(motor_id, pos_ticks, 1000, 0)
            if not addparam_result:
                logging.error(f"[ID:{motor_id:03d}] SyncWritePosEx add param failed")

        # Send the combined packet to all motors
        comm_result = self.packet_handler.groupSyncWrite.txPacket()
        self.handle_packet_result(comm_result, context="groupSyncWrite.txPacket")
        self.packet_handler.groupSyncWrite.clearParam()

    def write_desired_pos_vel_acc(
        self,
        motor_ids: Sequence[int],
        positions: np.ndarray,
        velocities: np.ndarray,
        accels: np.ndarray,
    ):
        """
        Writes desired positions, velocities, and accelerations to motors.
        Includes a critical safety check for position range.
        Units (deg or rad) are determined by the `use_degrees` flag.
        """
        self.check_connected()
        assert len(motor_ids) == len(positions) == len(velocities) == len(accels)

        for i, motor_id in enumerate(motor_ids):
            pos_ticks = int(positions[i] / self.pos_scale)
            vel_ticks = int(abs(velocities[i] / self.vel_scale))
            acc_val = int(accels[i])

            # --- CRITICAL SAFETY CHECK ---
            if not (0 <= pos_ticks <= 4096):
                logging.critical(
                    f"FATAL: Position command for motor ID {motor_id} is {pos_ticks}, which is out of the valid range (0-4096)."
                )
                logging.critical(
                    "Disabling torque on all motors for safety and quitting."
                )
                self.set_torque_enabled(self.motor_ids, False)
                raise ValueError(
                    f"Position command for motor {motor_id} is out of valid range."
                )

            addparam_result = self.HLSSyncWritePosEx(
                motor_id, pos_ticks, vel_ticks, acc_val
            )
            if not addparam_result:
                logging.error(f"[ID:{motor_id:03d}] SyncWritePosEx add param failed")

        comm_result = self.packet_handler.groupSyncWrite.txPacket()
        self.handle_packet_result(comm_result, context="groupSyncWrite.txPacket")
        self.packet_handler.groupSyncWrite.clearParam()

    def check_connected(self):
        """Raises an exception if the client is not connected."""
        if not self.is_connected:
            raise RuntimeError("Client is not connected. Call connect() first.")

    def handle_packet_result(
        self,
        comm_result: int,
        ft_error: Optional[int] = 0,
        ft_id: Optional[int] = None,
        context: Optional[str] = None,
    ):
        """Handles the result from a communication request, logging any errors."""
        if comm_result != COMM_SUCCESS:
            message = (
                f"{context or ''}: {self.packet_handler.getTxRxResult(comm_result)}"
            )
            if ft_id is not None:
                message = f"[ID:{ft_id:03d}] {message}"
            logging.error(message)
            return False
        elif ft_error != 0:
            message = (
                f"{context or ''}: {self.packet_handler.getRxPacketError(ft_error)}"
            )
            if ft_id is not None:
                message = f"[ID:{ft_id:03d}] {message}"
            logging.error(message)
            return False
        return True

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()


@atexit.register
def ft_cleanup_handler():
    """Ensures all motor clients are properly disconnected on exit."""
    open_clients = list(FTClient.OPEN_CLIENTS)
    for client in open_clients:
        logging.warning("Client was not disconnected. Forcing disconnect.")
        try:
            if client.is_connected:
                # Attempt to disable torque for safety on unclean exit
                client.set_torque_enabled(client.motor_ids, False)
        except Exception as e:
            logging.error(f"Error during final cleanup torque disable: {e}")
        client.disconnect()


class FTReader:
    """
    Generic base class for reading data from FT motors using GroupSyncRead.
    """

    def __init__(self, client, motor_ids: Sequence[int], address: int, size: int):
        self.client = client
        self.motor_ids = motor_ids
        self.address = address
        self.size = size
        # Initialize the SDK's GroupSyncRead helper
        self.operation = GroupSyncRead(client.packet_handler, address, size)
        for motor_id in self.motor_ids:
            success = self.operation.addParam(motor_id)
            if not success:
                raise OSError(
                    f"[ID:{motor_id:03d}] Could not add motor to GroupSyncRead operation."
                )
        self._initialize_data()

    def read(self):
        """Reads data from all registered motors."""
        self.client.check_connected()
        comm_result = self.operation.txRxPacket()
        if not self.client.handle_packet_result(comm_result, context="GroupSyncRead"):
            return self._get_data()  # Return last known data on comm failure

        errored_ids = []
        for i, motor_id in enumerate(self.motor_ids):
            # Check if data for this specific motor is available
            is_available, scs_error = self.operation.isAvailable(
                motor_id, self.address, self.size
            )
            if (
                self.client.handle_packet_result(
                    0, scs_error, motor_id, "GroupSyncRead.isAvailable"
                )
                and is_available
            ):
                # If available, update its data
                self._update_data(i, motor_id)
            else:
                errored_ids.append(motor_id)

        if errored_ids:
            logging.warning(
                "GroupSyncRead data unavailable for IDs: %s", str(errored_ids)
            )

        return self._get_data()

    def _initialize_data(self):
        raise NotImplementedError

    def _update_data(self, index: int, motor_id: int):
        raise NotImplementedError

    def _get_data(self):
        raise NotImplementedError


class FTPosVelReader(FTReader):
    """Reads both Position and Velocity in a single GroupSyncRead operation."""

    def __init__(self, client, motor_ids: Sequence[int]):
        # Read 4 bytes starting from Position (Pos=2 bytes, Vel=2 bytes)
        super().__init__(client, motor_ids, SMS_STS_PRESENT_POSITION_L, 4)

    def _initialize_data(self):
        self._pos_data = np.zeros(len(self.motor_ids), dtype=np.float32)
        self._vel_data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index: int, motor_id: int):
        # Get the raw 2-byte data for each value
        pos_raw = self.operation.getData(
            motor_id, SMS_STS_PRESENT_POSITION_L, LEN_PRESENT_POSITION
        )
        vel_raw = self.operation.getData(
            motor_id, SMS_STS_PRESENT_SPEED_L, LEN_PRESENT_SPEED
        )
        # Convert from ticks to scaled units (rad or deg)
        self._pos_data[index] = (
            unsigned_to_signed(pos_raw, size=2) * self.client.pos_scale
        )
        self._vel_data[index] = (
            unsigned_to_signed(vel_raw, size=2) * self.client.vel_scale
        )

    def _get_data(self):
        return self._pos_data.copy(), self._vel_data.copy()


class FTPosReader(FTReader):
    """Reads only Position using a GroupSyncRead operation."""

    def __init__(self, client, motor_ids: Sequence[int]):
        super().__init__(
            client, motor_ids, SMS_STS_PRESENT_POSITION_L, LEN_PRESENT_POSITION
        )

    def _initialize_data(self):
        self._pos_data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index: int, motor_id: int):
        pos_raw = self.operation.getData(
            motor_id, SMS_STS_PRESENT_POSITION_L, LEN_PRESENT_POSITION
        )
        self._pos_data[index] = (
            unsigned_to_signed(pos_raw, size=2) * self.client.pos_scale
        )

    def _get_data(self):
        return self._pos_data.copy()


class FTVelReader(FTReader):
    """Reads only Velocity using a GroupSyncRead operation."""

    def __init__(self, client, motor_ids: Sequence[int]):
        super().__init__(client, motor_ids, SMS_STS_PRESENT_SPEED_L, LEN_PRESENT_SPEED)

    def _initialize_data(self):
        self._vel_data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index: int, motor_id: int):
        vel_raw = self.operation.getData(
            motor_id, SMS_STS_PRESENT_SPEED_L, LEN_PRESENT_SPEED
        )
        self._vel_data[index] = (
            unsigned_to_signed(vel_raw, size=2) * self.client.vel_scale
        )

    def _get_data(self):
        return self._vel_data.copy()


if __name__ == "__main__":
    import argparse

    logging.basicConfig(level=logging.INFO)
    parser = argparse.ArgumentParser(
        description="FTClient for controlling Feetech motors."
    )
    parser.add_argument(
        "-m", "--motors", required=True, help="Comma-separated IDs (e.g., 1,2,3)."
    )
    parser.add_argument("-d", "--device", default="/dev/ttyUSB0", help="Serial device.")
    parser.add_argument("-b", "--baud", default=1000000, type=int, help="Baudrate.")
    parser.add_argument(
        "--mode",
        type=int,
        default=0,
        choices=[0, 1, 2],
        help="0: Read-Only Monitor, 1: Sine Wiggle, 2: Waypoint Control",
    )
    parser.add_argument(
        "--pid",
        nargs="*",
        type=int,
        metavar="P I D",
        help="Set PID gains. Use without args for default (32 0 64) or specify P I D.",
    )
    parser.add_argument(
        "--units",
        type=str,
        default="deg",
        choices=["deg", "rad"],
        help="Specify units for position and velocity (deg or rad). Default is deg.",
    )

    parsed_args = parser.parse_args()

    motor_ids = [int(motor) for motor in parsed_args.motors.split(",")]
    num_motors = len(motor_ids)
    use_degrees = parsed_args.units == "deg"

    try:
        with FTClient(
            motor_ids, parsed_args.device, parsed_args.baud, use_degrees=use_degrees
        ) as ft_client:
            # If --pid is passed, set the PID gains.
            if parsed_args.pid is not None:
                if len(parsed_args.pid) == 3:
                    p_gain, i_gain, d_gain = parsed_args.pid
                elif len(parsed_args.pid) == 0:
                    p_gain, i_gain, d_gain = 32, 0, 64  # Default values
                    logging.info(
                        "No PID values provided, using defaults: P=32, I=0, D=64"
                    )
                else:
                    raise ValueError(
                        "PID argument requires exactly 0 or 3 values (P I D)."
                    )
                ft_client.set_pid_gains(p=p_gain, i=i_gain, d=d_gain)

            # --- Mode Selection ---
            if parsed_args.mode == 0:
                # --- Mode 0: Read-Only Monitoring ---
                all_pid_gains = ft_client.read_pid_gains()
                np.set_printoptions(precision=2, suppress=True)
                unit_str = "deg" if use_degrees else "rad"

                while True:
                    os.system("cls" if os.name == "nt" else "clear")
                    print(f"--- Feetech Motor Monitor (Mode 0) | Units: {unit_str} ---")
                    print(f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}")

                    random_motor_id = random.choice(list(all_pid_gains.keys()))
                    gains = all_pid_gains[random_motor_id]
                    print("\n--- Randomly Sampled PID Gains (P, D, I) ---")
                    print(
                        f"  Showing Motor ID {random_motor_id:02d}: P={gains['P']:<3} D={gains['D']:<3} I={gains['I']:<3}"
                    )

                    pos_now, vel_now = ft_client.read_pos_vel()

                    print(f"\n--- Motor State ({unit_str}, {unit_str}/s) ---")
                    print("Position:")
                    print(pos_now.reshape(-1, 4))
                    print("\nVelocity:")
                    print(vel_now.reshape(-1, 4))

                    time.sleep(0.1)

            elif parsed_args.mode == 1 or parsed_args.mode == 2:
                # --- Setup for motion modes ---
                if not os.path.exists("logs"):
                    os.makedirs("logs")
                log_filename = (
                    f"logs/motor_log_{time.strftime('%Y-%m-%d_%H-%M-%S')}.txt"
                )
                log_file = open(log_filename, "w")
                logging.info(f"Logging motor data to {log_filename}")

                # --- Modes 1 & 2 require torque to be enabled ---
                print("Enabling torque for all motors...")
                ft_client.set_torque_enabled(motor_ids, True)

                try:
                    if parsed_args.mode == 1:
                        # --- Mode 1: Sine Wiggle (CONTAINS BUG) ---
                        unit_str = "deg" if use_degrees else "rad"
                        np.set_printoptions(precision=2, suppress=True)

                        if use_degrees:
                            center_pos = 180.0
                            amplitude = 10.0
                        else:
                            center_pos = np.pi
                            amplitude = np.deg2rad(10)

                        print(
                            f"--- Starting Sine Wiggle (Mode 1) | Units: {unit_str} ---"
                        )
                        print(
                            f"Moving to neutral position ({center_pos:.2f} {unit_str}) and holding for 5 seconds..."
                        )
                        neutral_positions = np.full(num_motors, center_pos)
                        ft_client.write_desired_pos_simple(motor_ids, neutral_positions)
                        time.sleep(5)

                        frequency_hz = 0.5
                        start_time = time.time()
                        phase_offsets = np.linspace(
                            0, 2 * np.pi, num_motors, endpoint=False
                        )

                        print("Starting wiggle...")
                        while True:
                            loop_start_time = time.time()
                            os.system("cls" if os.name == "nt" else "clear")
                            print(
                                f"--- Feetech Motor Wiggle (Mode 1) | Units: {unit_str} ---"
                            )
                            timestamp_str = time.strftime("%Y-%m-%d %H:%M:%S")
                            print(f"Timestamp: {timestamp_str}")

                            elapsed_time = time.time() - start_time

                            # *** BUGGY LOGIC HERE ***
                            # This maps sin() from [-1, 1] to [0, 1]
                            sin_wave = (
                                np.sin(
                                    2 * np.pi * frequency_hz * elapsed_time
                                    + phase_offsets
                                )
                                + 1
                            ) / 2.0
                            # This results in oscillation between [center_pos, center_pos + amplitude]
                            goal_pos = center_pos + amplitude * sin_wave

                            ft_client.write_desired_pos_simple(motor_ids, goal_pos)
                            pos_now, vel_now = ft_client.read_pos_vel()

                            print(f"\n--- Motor State ({unit_str}, {unit_str}/s) ---")
                            print(f" Goal Pos: {goal_pos.round(2)}")
                            print(f" Read Pos: {pos_now.round(2)}")
                            print(f" Read Vel: {vel_now.round(2)}")

                            log_file.write(f"--- Timestamp: {timestamp_str} ---\n")
                            log_file.write(f" Goal Pos: {goal_pos}\n")
                            log_file.write(f" Read Pos: {pos_now}\n")
                            log_file.write(f" Read Vel: {vel_now}\n\n")
                            log_file.flush()

                            loop_duration = time.time() - loop_start_time
                            if loop_duration > 0:
                                print(f"Loop freq: {(1.0 / loop_duration):.2f} Hz")
                            time.sleep(max(0, 0.01 - loop_duration))  # Aim for ~100Hz

                    elif parsed_args.mode == 2:
                        # --- Mode 2: Waypoint Control ---
                        unit_str = "deg" if use_degrees else "rad"
                        np.set_printoptions(precision=2, suppress=True)

                        if use_degrees:
                            way_points = [
                                np.full(num_motors, 0.0),
                                np.full(num_motors, 90.0),
                                np.full(num_motors, 180.0),
                                np.full(num_motors, 270.0),
                            ]
                        else:
                            way_points = [
                                np.zeros(num_motors),
                                np.full(num_motors, np.pi / 2),
                                np.full(num_motors, np.pi),
                                np.full(num_motors, 3 * np.pi / 2),
                            ]

                        waypoint_change_interval = 5.0  # seconds
                        start_time = time.time()

                        print(
                            f"--- Starting Waypoint Control (Mode 2) | Units: {unit_str} ---"
                        )
                        while True:
                            os.system("cls" if os.name == "nt" else "clear")
                            print(
                                f"--- Feetech Waypoint Control (Mode 2) | Units: {unit_str} ---"
                            )
                            timestamp_str = time.strftime("%Y-%m-%d %H:%M:%S")
                            print(f"Timestamp: {timestamp_str}")

                            elapsed_time = time.time() - start_time
                            current_waypoint_index = int(
                                elapsed_time / waypoint_change_interval
                            ) % len(way_points)
                            goal_pos = way_points[current_waypoint_index]

                            ft_client.write_desired_pos_simple(motor_ids, goal_pos)
                            pos_now, vel_now = ft_client.read_pos_vel()

                            print(f"\n--- Motor State ({unit_str}, {unit_str}/s) ---")
                            print(f" Goal Pos: {goal_pos.round(2)}")
                            print(f" Read Pos: {pos_now.round(2)}")
                            print(f" Read Vel: {vel_now.round(2)}")

                            log_file.write(f"--- Timestamp: {timestamp_str} ---\n")
                            log_file.write(f" Goal Pos: {goal_pos}\n")
                            log_file.write(f" Read Pos: {pos_now}\n")
                            log_file.write(f" Read Vel: {vel_now}\n\n")
                            log_file.flush()
                            time.sleep(0.1)
                finally:
                    # Ensure the log file is closed properly on exit
                    if log_file:
                        log_file.close()
                        logging.info(f"Log file {log_filename} closed.")

    except (KeyboardInterrupt, ValueError, RuntimeError) as e:
        print(f"\nExiting script. Reason: {e}")
