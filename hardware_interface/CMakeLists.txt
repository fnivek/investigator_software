cmake_minimum_required(VERSION 2.8.3)
project(hardware_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
	FILES
	MotorPWM.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  CommSrv.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)