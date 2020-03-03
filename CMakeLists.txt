cmake_minimum_required(VERSION 2.8.3)
project(keti_robot_master)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  dsr_msgs  
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(PROGRAMS scripts/dsr_robot_test_keyop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/dsr_robot_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/robot_module/robot_master.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/robot_module/robot/robot_slave.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

