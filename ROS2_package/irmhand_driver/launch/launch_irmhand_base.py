from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="irmhand_driver",
                executable="irmhand_node_base.py",
                name="irmhand_node_base",
                emulate_tty=True,
                output="screen",
                parameters=[
                    {"kP": 25.0},
                    {"kI": 0.0},
                    {"kD": 50.0},
                    {
                        "curr_lim": 500.0
                    },  # Note: curr_lim is declared but not used in client
                ],
            )
        ]
    )
