#!/usr/bin/env python3

import time

# Renamed import for irmhand_utils
import irmhand_utils.irmhand_utils as lhu
import numpy as np
import rclpy

# Renamed import for ft_motor_client
from FT_client import *
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Service definitions now come from this package
from irmhand_driver.srv import LeapPosition, LeapPosVelEff, LeapVelocity

# IRM Hand (LEAP API) conventions:
# The API uses radians.
# A value of pi (3.14) corresponds to the flat-out home pose for index/middle/ring MCPs.
# The FT motors themselves use a [0, 4096] tick range, which the client
# maps to [0, 2*pi] radians.

# Joint numbering is 0-15 (Index, Middle, Ring, Thumb)
# E.g., Index MCP-Side (0), Index MCP-Fwd (1), Index PIP (2), Index DIP (3)


class IRMHandInterpNode(Node):
    def __init__(self):
        super().__init__("irmhand_node_interp")
        # Parameters for PID gains
        self.kP = self.declare_parameter("kP", 25.0).get_parameter_value().double_value
        self.kI = self.declare_parameter("kI", 0.0).get_parameter_value().double_value
        self.kD = self.declare_parameter("kD", 50.0).get_parameter_value().double_value
        # NOTE: Current limit is declared in launch file but not supported by the FT_client API.
        self.curr_lim = (
            self.declare_parameter("curr_lim", 350.0).get_parameter_value().double_value
        )

        # Position is in RADIANS.
        self.curr_pos = np.ones(16) * np.pi
        self.curr_pos[13] += 1.61  # Thumb offset
        self.target_pos = self.prev_pos = self.pos = self.curr_pos

        # Interpolation parameters
        self.interp_enable = True
        self.interp_T = 1.0  # Interpolation time in seconds
        self.interp_start_time = time.perf_counter()
        self.first_loop = True

        # Subscribes to command topics
        self.create_subscription(JointState, "cmd_irm", self._receive_pose, 10)
        self.create_subscription(
            JointState, "cmd_irm_interp", self._receive_pose_interp, 10
        )
        self.create_subscription(JointState, "cmd_allegro", self._receive_allegro, 10)
        self.create_subscription(JointState, "cmd_ones", self._receive_ones, 10)

        # Creates services to provide hand state
        self.create_service(LeapPosition, "irm_position", self.pos_srv)
        self.create_service(LeapVelocity, "irm_velocity", self.vel_srv)
        self.create_service(LeapPosVelEff, "irm_pos_vel", self.pos_vel_srv)

        self.motors = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        self.positions = []
        self.velocities = []

        # --- FTClient Instantiation ---
        # Search for the hand on the first 3 USB ports
        for i in range(3):
            port = f"/dev/ttyUSB{i}"
            try:
                self.get_logger().info(
                    f"Attempting to connect to IRM Hand on port {port}..."
                )
                # Instantiate FTClient, use_degrees=False for RADIANS
                self.ft_client = FTClient(
                    self.motors, port, baudrate=1000000, use_degrees=False
                )
                self.ft_client.connect()
                self.get_logger().info(
                    f"Successfully connected to IRM Hand on port {port}."
                )
                break
            except Exception as e:
                self.get_logger().warn(f"Failed to connect on {port}: {e}")
                if i == 2:
                    self.get_logger().error(
                        "Could not connect to IRM Hand on any port."
                    )
                    raise

        # Enable torque
        self.ft_client.set_torque_enabled(self.motors, True)

        # Set PID gains
        self.ft_client.set_pid_gains(p=int(self.kP), i=int(self.kI), d=int(self.kD))

        # --- State Publisher ---
        self.state_hz = (
            self.declare_parameter("state_hz", 100.0).get_parameter_value().double_value
        )
        # Publishes the current joint states of the hand
        self.leap_pos_vel_pub = self.create_publisher(
            JointState, "/irmhand/joint_states", 10
        )
        # Timer to publish state and run interpolation step
        self.create_timer(1.0 / self.state_hz, self._publish_state_and_step)

    def _receive_pose(self, msg):
        # Direct command, disable interpolation
        self.interp_enable = False
        pose = msg.position
        self.curr_pos = np.array(pose)
        self.ft_client.write_desired_pos_simple(self.motors, self.curr_pos)

    def _receive_pose_interp(self, msg):
        # Interpolated command
        self.interp_enable = True
        pose = msg.position
        self.prev_pos = self.positions  # Start from current read position
        self.target_pos = np.array(pose)
        self.interp_start_time = time.perf_counter()

    def _receive_allegro(self, msg):
        # Allegro compatibility, disable interpolation
        self.interp_enable = False
        pose = lhu.allegro_to_LEAPhand(msg.position, zeros=False)
        self.curr_pos = np.array(pose)
        self.ft_client.write_desired_pos_simple(self.motors, self.curr_pos)

    def _receive_ones(self, msg):
        # Sim compatibility, disable interpolation
        self.interp_enable = False
        pose = lhu.sim_ones_to_LEAPhand(np.array(msg.position))
        self.curr_pos = np.array(pose)
        self.ft_client.write_desired_pos_simple(self.motors, self.curr_pos)

    def _publish_state_and_step(self):
        # This function runs at self.state_hz
        # 1. Read current state from motors
        try:
            positions, velocities = self.ft_client.read_pos_vel()
        except Exception as e:
            print(f"read_pos_vel failed: {e}")
            return

        self.positions = positions
        self.velocities = velocities
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"irm_joint_{m}" for m in self.motors]
        msg.position = positions.tolist()
        msg.velocity = velocities.tolist()

        # 2. Publish the current state
        self.leap_pos_vel_pub.publish(msg)

        if self.first_loop:
            self.prev_pos = self.positions
            self.first_loop = False

        # 3. Perform one step of interpolation if enabled
        elapsed_time = time.perf_counter() - self.interp_start_time
        if self.interp_enable and elapsed_time <= self.interp_T:
            t = min(elapsed_time / self.interp_T, 1.0)
            # Smoothstep interpolation (6t^5 - 15t^4 + 10t^3)
            progress = 10 * t**3 - 15 * t**4 + 6 * t**5
            pos_cmd = self.prev_pos + progress * (self.target_pos - self.prev_pos)
            self.ft_client.write_desired_pos_simple(self.motors, pos_cmd)

    # Service that returns the last read pos in radians.
    def pos_srv(self, request, response):
        response.position = self.positions.tolist()
        return response

    # Service that returns the last read vel in rad/s.
    def vel_srv(self, request, response):
        response.velocity = self.velocities.tolist()
        return response

    # Combined service
    def pos_vel_srv(self, request, response):
        response.position = self.positions.tolist()
        response.velocity = self.velocities.tolist()
        # Effort not supported, return zeros
        response.effort = np.zeros_like(self.positions).tolist()
        return response


def main(args=None):
    rclpy.init(args=args)
    irmhand_node = IRMHandInterpNode()
    rclpy.spin(irmhand_node)
    irmhand_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
