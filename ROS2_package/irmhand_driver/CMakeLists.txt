cmake_minimum_required(VERSION 3.5)
project(irmhand_driver)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LeapVelocity.srv"
  "srv/LeapPosition.srv"
  "srv/LeapEffort.srv"
  "srv/LeapPosVelEff.srv"
  DEPENDENCIES std_msgs
)

###################################
## ament specific configuration  ##
###################################
ament_package()

#############
## Install ##
#############
# Install the Python utility module/directory
# (Assuming you rename 'scripts/leap_hand_utils' to 'scripts/irmhand_utils')
install(DIRECTORY
  scripts/irmhand_utils
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/irmhand_node_base.py
  scripts/irmhand_node_interp.py
  scripts/FT_client.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
