#!/usr/bin/env python3


import numpy as np
import rclpy
import utils.leap_utils as lhu
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


class IRMHandNode(Node):
    def __init__(self):
        super().__init__("irmhand_node_base")
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
        self.prev_pos = self.pos = self.curr_pos

        # Subscribes to command topics
        self.create_subscription(JointState, "cmd_irm", self._receive_pose, 10)
        self.create_subscription(JointState, "cmd_allegro", self._receive_allegro, 10)
        self.create_subscription(JointState, "cmd_ones", self._receive_ones, 10)

        # Creates services to provide hand state
        self.create_service(LeapPosition, "irm_position", self.pos_srv)
        self.create_service(LeapVelocity, "irm_velocity", self.vel_srv)
        self.create_service(LeapPosVelEff, "irm_pos_vel", self.pos_vel_srv)

        # Motor IDs
        self.motors = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]

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

        # Set initial position
        self.ft_client.write_desired_pos_simple(self.motors, self.curr_pos)

    # Receive IRM pose and directly control the robot.
    def _receive_pose(self, msg):
        pose = msg.position
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.ft_client.write_desired_pos_simple(self.motors, self.curr_pos)

    # Allegro compatibility
    def _receive_allegro(self, msg):
        pose = lhu.allegro_to_LEAPhand(msg.position, zeros=False)
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.ft_client.write_desired_pos_simple(self.motors, self.curr_pos)

    # Sim compatibility
    def _receive_ones(self, msg):
        pose = lhu.sim_ones_to_LEAPhand(np.array(msg.position))
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.ft_client.write_desired_pos_simple(self.motors, self.curr_pos)

    # Service that reads and returns the pos of the robot in radians.
    def pos_srv(self, request, response):
        response.position = self.ft_client.read_pos().tolist()
        return response

    # Service that reads and returns the vel of the robot in rad/s.
    def vel_srv(self, request, response):
        response.velocity = self.ft_client.read_vel().tolist()
        return response

    # Combined service to save latency
    def pos_vel_srv(self, request, response):
        pos, vel = self.ft_client.read_pos_vel()
        response.position = pos.tolist()
        response.velocity = vel.tolist()
        # Effort not supported by FT client, return zeros
        response.effort = np.zeros_like(pos).tolist()
        return response


def main(args=None):
    rclpy.init(args=args)
    irmhand_node = IRMHandNode()
    rclpy.spin(irmhand_node)
    irmhand_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
