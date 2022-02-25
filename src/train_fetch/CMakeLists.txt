cmake_minimum_required(VERSION 3.0.2)
project(train_fetch)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  gazebo_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES move_base
#  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/test.py scripts/send_msgs.py scripts/arm.py scripts/read_link_states.py scripts/*.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

