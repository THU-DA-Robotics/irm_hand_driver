#!/usr/bin/env python3
#
# ********* IRM Hand (Feetech Motor) Utility Script *********
#
# This script provides an interactive command-line interface to configure
# Feetech HTS/STS series smart servos.
#
# Features:
# 1. (Mode 0) Center Calibration: Interactively sets the center position (offset)
#    for motors one by one.
#
# 2. (Mode 1) Batch PID Editor: Sets the P, I, and D position gains for all
#    connected servos at once.
#
# 3. (Mode 2) Batch Generic Write: A powerful tool to write any value to a
#    specific register address on all or a selected subset of connected motors.
#
# 4. (Mode 3) Batch Generic Read: Reads and displays data from a specific
#    register address for all or a selected subset of connected motors.
#

import os
import sys
import threading
import time

from scservo_sdk import *

# --- Configuration ---
# Your serial port name.
# Linux: "/dev/ttyUSB0", "/dev/ttyAMA0", etc.
PORT_NAME = "/dev/ttyUSB0"

# Baudrate for the motors
BAUDRATE = 1000000

# Motor ID scan range
MIN_ID = 0
MAX_ID = 50

# --- Control Table Addresses ---
SMS_STS_TORQUE_ENABLE = 40
SMS_STS_PRESENT_POSITION_L = 56
INST_RECENTER = 11

# --- PID EEPROM Registers (Position Loop) ---
SMS_STS_POS_P = 21  # Kp: Position Loop P Gain
SMS_STS_POS_D = 22  # Kd: Position Loop D Gain
SMS_STS_POS_I = 23  # Ki: Position Loop I Gain

# The desired center position value. The calibration command sets it to 2048.
TARGET_CENTER_POSITION = 2048

# --- Global variable for the position reading thread ---
stop_thread = threading.Event()


def position_reader_thread(packetHandler, motor_id):
    """A thread that continuously reads and displays the motor's position."""
    print("\nStarting real-time position display...")
    while not stop_thread.is_set():
        scs_present_position, _, scs_comm_result, scs_error = (
            packetHandler.ReadPosSpeed(motor_id)
        )
        if scs_comm_result == COMM_SUCCESS and scs_error == 0:
            sys.stdout.write(
                f"\rCurrent Position: {scs_present_position:04d} | Move motor to desired center, then press Enter to confirm."
            )
            sys.stdout.flush()
        else:
            sys.stdout.write(
                f"\rError reading position. Check connection. Comm: {packetHandler.getTxRxResult(scs_comm_result)}, Err: {packetHandler.getRxPacketError(scs_error)}"
            )
            sys.stdout.flush()
            if scs_comm_result != COMM_SUCCESS:
                break
        time.sleep(0.1)


def ping_for_motors(packetHandler):
    """Scans for available motors and returns a list of their IDs."""
    print("Pinging for motors...")
    available_ids = []
    for motor_id in range(MIN_ID, MAX_ID + 1):
        _, scs_comm_result, scs_error = packetHandler.ping(motor_id)
        if scs_comm_result == COMM_SUCCESS and scs_error == 0:
            sys.stdout.write(f"\rFound motor with ID: {motor_id}   ")
            sys.stdout.flush()
            available_ids.append(motor_id)
            time.sleep(0.01)
    print("\n")
    return available_ids


def select_motor_from_list(available_ids):
    """Prompts the user to select a motor from a list of IDs."""
    print(f"Available motor IDs: {available_ids}")
    while True:
        try:
            choice = input(
                "Enter the ID of the motor to calibrate (or type 'q' to quit): "
            )
            if choice.lower() == "q":
                return None
            chosen_id = int(choice)
            if chosen_id in available_ids:
                return chosen_id
            else:
                print("Invalid ID. Please choose from the available list.")
        except ValueError:
            print("Invalid input. Please enter a number.")


def recenter_motor_command(packetHandler, scs_id):
    """Sends the raw, low-level recenter command (Instruction 11)."""
    txpacket = [0] * 6
    txpacket[2] = scs_id
    txpacket[3] = 2
    txpacket[4] = INST_RECENTER
    _, result, error = packetHandler.txRxPacket(txpacket)
    return result, error


def calibrate_motor(packetHandler, motor_id):
    """Guides the user through the calibration process for a single motor."""
    print(f"\n--- Calibrating Motor ID: {motor_id} ---")
    global stop_thread
    stop_thread.clear()
    reader = threading.Thread(
        target=position_reader_thread, args=(packetHandler, motor_id)
    )
    reader.start()
    input()
    stop_thread.set()
    reader.join()
    print("\n")

    print("Writing calibration command to motor EEPROM...")
    scs_comm_result, _ = packetHandler.write1ByteTxRx(
        motor_id, SMS_STS_TORQUE_ENABLE, 0
    )
    if scs_comm_result != COMM_SUCCESS:
        print("Error: Failed to disable torque.")
        return
    print("Torque disabled.")
    time.sleep(0.05)

    scs_comm_result, _ = packetHandler.unLockEprom(motor_id)
    if scs_comm_result != COMM_SUCCESS:
        print("Error: Failed to unlock EEPROM.")
        return
    print("EEPROM unlocked.")
    time.sleep(0.05)

    scs_comm_result, _ = recenter_motor_command(packetHandler, motor_id)
    if scs_comm_result != COMM_SUCCESS:
        print("Error: Failed to send recenter command.")
        packetHandler.LockEprom(motor_id)
        return
    print("Recenter command sent successfully.")
    time.sleep(0.05)

    scs_comm_result, _ = packetHandler.LockEprom(motor_id)
    if scs_comm_result != COMM_SUCCESS:
        print("CRITICAL ERROR: Failed to re-lock EEPROM. The change may not be saved.")
    else:
        print("EEPROM re-locked. Change is saved.")
    time.sleep(0.05)

    print(f"\n--- Calibration Complete for Motor ID: {motor_id} ---")


def get_pid_input():
    """Gets and validates PID values from the user."""
    while True:
        try:
            p_val = int(input("Enter P gain (0-254): "))
            d_val = int(input("Enter D gain (0-254): "))
            i_val = int(input("Enter I gain (0-254): "))
            if 0 <= p_val <= 254 and 0 <= d_val <= 254 and 0 <= i_val <= 254:
                return p_val, d_val, i_val
            else:
                print("Error: All values must be between 0 and 254.")
        except ValueError:
            print("Invalid input. Please enter numbers only.")


def batch_edit_pid(packetHandler):
    """Guides the user through batch editing PID gains for all connected motors."""
    print("\n--- Batch Editing PID Gains ---")
    available_ids = ping_for_motors(packetHandler)
    if not available_ids:
        print("No motors found to edit.")
        return
    print(f"The following motor IDs will be updated: {available_ids}")

    p_val, d_val, i_val = get_pid_input()
    confirm = input(
        f"\nThis will write P={p_val}, D={d_val}, I={i_val} to ALL {len(available_ids)} motors. \nAre you sure? (y/n): "
    )
    if confirm.lower() != "y":
        print("Operation cancelled.")
        return

    print("\nStarting PID write process...")
    successful_writes = 0
    failed_ids = []
    for motor_id in available_ids:
        print(f"--- Processing Motor ID: {motor_id} ---")
        packetHandler.write1ByteTxRx(motor_id, SMS_STS_TORQUE_ENABLE, 0)
        time.sleep(0.05)

        scs_comm_result, _ = packetHandler.unLockEprom(motor_id)
        if scs_comm_result != COMM_SUCCESS:
            print(f"Error: Failed to unlock EEPROM for motor {motor_id}. Skipping.")
            failed_ids.append(motor_id)
            continue
        print(f"Motor {motor_id}: EEPROM unlocked.")
        time.sleep(0.05)

        p_res, _ = packetHandler.write1ByteTxRx(motor_id, SMS_STS_POS_P, p_val)
        time.sleep(0.01)
        d_res, _ = packetHandler.write1ByteTxRx(motor_id, SMS_STS_POS_D, d_val)
        time.sleep(0.01)
        i_res, _ = packetHandler.write1ByteTxRx(motor_id, SMS_STS_POS_I, i_val)
        time.sleep(0.01)

        if p_res != COMM_SUCCESS or d_res != COMM_SUCCESS or i_res != COMM_SUCCESS:
            print(f"Error: Failed to write PID values to motor {motor_id}.")
            failed_ids.append(motor_id)
        else:
            print(f"Motor {motor_id}: PID values written successfully.")

        lock_res, _ = packetHandler.LockEprom(motor_id)
        if lock_res != COMM_SUCCESS:
            print(f"CRITICAL ERROR: Failed to re-lock EEPROM for motor {motor_id}.")
            if motor_id not in failed_ids:
                failed_ids.append(motor_id)
        else:
            print(f"Motor {motor_id}: EEPROM re-locked.")
            if motor_id not in failed_ids:
                successful_writes += 1
        time.sleep(0.05)
        packetHandler.write1ByteTxRx(motor_id, SMS_STS_TORQUE_ENABLE, 1)

    print("\n--- Batch PID Edit Complete ---")
    print(f"Successfully updated {successful_writes} motors.")
    if failed_ids:
        print(f"Failed to update the following motor IDs: {list(set(failed_ids))}")


def get_generic_write_input():
    """Gets and validates address, length, and value from the user."""
    while True:
        try:
            address = int(input("Enter register address to write (e.g., 5 for ID): "))
            if not 0 <= address <= 255:
                print("Address must be between 0 and 255.")
                continue

            length = int(input("Enter data length in bytes (1, 2, or 4): "))
            if length not in [1, 2, 4]:
                print("Error: Length must be 1, 2, or 4.")
                continue

            max_val = (2 ** (length * 8)) - 1
            value = int(input(f"Enter value to write (0 - {max_val}): "))
            if not 0 <= value <= max_val:
                print(
                    f"Error: Value must be between 0 and {max_val} for a {length}-byte length."
                )
                continue

            return address, length, value
        except ValueError:
            print("Invalid input. Please enter numbers only.")


def batch_generic_write(packetHandler):
    """Writes a custom value to a specified register address for all or a subset of motors."""
    print("\n--- Batch Generic Register Write ---")
    available_ids = ping_for_motors(packetHandler)
    if not available_ids:
        print("No motors found to edit.")
        return

    print("This mode allows writing a value to the same register on multiple motors.")
    print(
        "\033[91mWARNING: Writing to incorrect registers can make motors unresponsive or damage them.\033[0m"
    )

    print(f"Available motor IDs: {available_ids}")

    target_ids = []
    while True:
        choice = input("Write to ALL found motors? (y/n): ").strip().lower()
        if choice == "y":
            target_ids = available_ids
            break
        elif choice == "n":
            try:
                id_input = input(
                    "Enter the motor IDs to target, separated by commas (e.g., 1, 3, 5): "
                )
                selected_ids_str = [s.strip() for s in id_input.split(",")]
                # Filter out empty strings that can result from trailing commas
                potential_target_ids = [int(s) for s in selected_ids_str if s]

                invalid_ids = [
                    sid for sid in potential_target_ids if sid not in available_ids
                ]

                if invalid_ids:
                    print(
                        f"Error: The following IDs are not available on the bus: {invalid_ids}"
                    )
                    continue

                if not potential_target_ids:
                    print("Error: No IDs were entered.")
                    continue

                target_ids = potential_target_ids
                break
            except ValueError:
                print("Invalid input. Please enter numbers separated by commas.")
        else:
            print("Invalid choice. Please enter 'y' or 'n'.")

    if not target_ids:
        print("No target motors selected. Operation cancelled.")
        return

    address, length, value = get_generic_write_input()

    print(f"\nTarget Motors:      {target_ids}")
    print(f"  Register Address:   {address} (0x{address:02X})")
    print(f"  Data Length:        {length} byte(s)")
    print(f"  Value to Write:     {value}")

    confirm = input(
        f"\nThis will write this data to the {len(target_ids)} selected motor(s). \nAre you absolutely sure? (y/n): "
    )
    if confirm.lower() != "y":
        print("Operation cancelled.")
        return

    print("\nStarting batch write process...")
    successful_writes = 0
    failed_ids = []
    for motor_id in target_ids:
        print(f"--- Processing Motor ID: {motor_id} ---")
        packetHandler.write1ByteTxRx(motor_id, SMS_STS_TORQUE_ENABLE, 0)
        time.sleep(0.05)

        scs_comm_result, _ = packetHandler.unLockEprom(motor_id)
        if scs_comm_result != COMM_SUCCESS:
            print(f"Error: Failed to unlock EEPROM for motor {motor_id}. Skipping.")
            failed_ids.append(motor_id)
            continue
        print(f"Motor {motor_id}: EEPROM unlocked.")
        time.sleep(0.05)

        write_res = COMM_TX_FAIL
        if length == 1:
            write_res, _ = packetHandler.write1ByteTxRx(motor_id, address, value)
        elif length == 2:
            write_res, _ = packetHandler.write2ByteTxRx(motor_id, address, value)
        elif length == 4:
            write_res, _ = packetHandler.write4ByteTxRx(motor_id, address, value)

        if write_res != COMM_SUCCESS:
            print(f"Error: Failed to write data to motor {motor_id}.")
            failed_ids.append(motor_id)
        else:
            print(f"Motor {motor_id}: Wrote value {value} to address {address}.")

        lock_res, _ = packetHandler.LockEprom(motor_id)
        if lock_res != COMM_SUCCESS:
            print(f"CRITICAL ERROR: Failed to re-lock EEPROM for motor {motor_id}.")
            if motor_id not in failed_ids:
                failed_ids.append(motor_id)
        else:
            print(f"Motor {motor_id}: EEPROM re-locked.")
            if motor_id not in failed_ids:
                successful_writes += 1
        time.sleep(0.05)
        packetHandler.write1ByteTxRx(motor_id, SMS_STS_TORQUE_ENABLE, 1)

    print("\n--- Batch Write Complete ---")
    print(f"Successfully wrote to {successful_writes} motors.")
    if failed_ids:
        print(f"Failed to process the following motor IDs: {list(set(failed_ids))}")


def get_generic_read_input():
    """Gets and validates address and length from the user for a read operation."""
    while True:
        try:
            address = int(
                input("Enter register address to read from (e.g., 56 for Position): ")
            )
            if not 0 <= address <= 255:
                print("Address must be between 0 and 255.")
                continue

            length = int(input("Enter data length in bytes to read (1, 2, or 4): "))
            if length not in [1, 2, 4]:
                print("Error: Length must be 1, 2, or 4.")
                continue

            return address, length
        except ValueError:
            print("Invalid input. Please enter numbers only.")


def batch_generic_read(packetHandler):
    """Reads a value from a specified register address for selected motors."""
    print("\n--- Batch Generic Register Read ---")
    available_ids = ping_for_motors(packetHandler)
    if not available_ids:
        print("No motors found to read from.")
        return

    print(f"Available motor IDs: {available_ids}")
    target_ids = []
    while True:
        choice = input("Read from ALL found motors? (y/n): ").strip().lower()
        if choice == "y":
            target_ids = available_ids
            break
        elif choice == "n":
            try:
                id_input = input(
                    "Enter the motor IDs to target, separated by commas (e.g., 1, 3, 5): "
                )
                selected_ids_str = [s.strip() for s in id_input.split(",")]
                potential_target_ids = [int(s) for s in selected_ids_str if s]

                invalid_ids = [
                    sid for sid in potential_target_ids if sid not in available_ids
                ]
                if invalid_ids:
                    print(
                        f"Error: The following IDs are not available on the bus: {invalid_ids}"
                    )
                    continue

                if not potential_target_ids:
                    print("Error: No IDs were entered.")
                    continue

                target_ids = potential_target_ids
                break
            except ValueError:
                print("Invalid input. Please enter numbers separated by commas.")
        else:
            print("Invalid choice. Please enter 'y' or 'n'.")

    if not target_ids:
        print("No target motors selected. Operation cancelled.")
        return

    address, length = get_generic_read_input()
    print(
        f"\nReading {length} byte(s) from address {address} (0x{address:02X}) for motor(s): {target_ids}\n"
    )

    for motor_id in target_ids:
        read_value, comm_result, error = (None, None, None)

        if length == 1:
            read_value, comm_result, error = packetHandler.read1ByteTxRx(
                motor_id, address
            )
        elif length == 2:
            read_value, comm_result, error = packetHandler.read2ByteTxRx(
                motor_id, address
            )
        elif length == 4:
            read_value, comm_result, error = packetHandler.read4ByteTxRx(
                motor_id, address
            )

        if comm_result == COMM_SUCCESS and error == 0:
            print(f"  Motor ID {motor_id:02d}: Value = {read_value}")
        else:
            comm_status = packetHandler.getTxRxResult(comm_result)
            error_status = packetHandler.getRxPacketError(error)
            print(
                f"  Motor ID {motor_id:02d}: \033[91mFAILED\033[0m. Comm: {comm_status}, Error: {error_status}"
            )


def main():
    """Main program execution."""
    portHandler = PortHandler(PORT_NAME)
    packetHandler = sms_sts(portHandler)

    if not portHandler.openPort():
        print(f"Failed to open the port: {PORT_NAME}")
        return
    print(f"Succeeded to open the port: {PORT_NAME}")

    if not portHandler.setBaudRate(BAUDRATE):
        print(f"Failed to change the baudrate to {BAUDRATE}")
        portHandler.closePort()
        return
    print(f"Baudrate set to: {BAUDRATE}")

    while True:
        os.system("cls" if os.name == "nt" else "clear")
        print("***********************************")
        print("* IRM Hand Motor Utility Script   *")
        print("***********************************")
        print("\nSelect an operation mode:")
        print("  0: Recenter motor(s) (Calibration)")
        print("  1: Batch edit PID gains for all motors")
        print("  2: Batch write to custom register (WARNING: Use with caution)")
        print("  3: Batch read from a custom register")
        print("  q: Quit")

        mode_choice = input("\nEnter your choice: ").strip().lower()

        if mode_choice == "0":
            available_ids = ping_for_motors(packetHandler)
            if not available_ids:
                input("\nNo motors found. Press Enter to return to the main menu...")
                continue
            motor_id_to_calibrate = select_motor_from_list(available_ids)
            if motor_id_to_calibrate is not None:
                calibrate_motor(packetHandler, motor_id_to_calibrate)

        elif mode_choice == "1":
            batch_edit_pid(packetHandler)

        elif mode_choice == "2":
            batch_generic_write(packetHandler)

        elif mode_choice == "3":
            batch_generic_read(packetHandler)

        elif mode_choice == "q":
            break

        else:
            print("Invalid choice. Please try again.")

        input("\nOperation complete. Press Enter to return to the main menu...")

    print("Closing port.")
    portHandler.closePort()


if __name__ == "__main__":
    main()
