cmake_minimum_required(VERSION 2.8.3)
project(test_equipment_drivers)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    rospy
    std_msgs
  DEPENDS pthread
)

catkin_python_setup()

###########
## Build ##
###########


#############
## Testing ##
#############
