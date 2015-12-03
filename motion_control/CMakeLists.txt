cmake_minimum_required(VERSION 2.8.3)
project(motion_control)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  message_generation
  rospy
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MotorAngularWheelVelocities.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  WaypointCmd.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS hardware_interface rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)