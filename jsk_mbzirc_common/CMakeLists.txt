cmake_minimum_required(VERSION 2.8.3)
project(jsk_mbzirc_common)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS gazebo_ros std_msgs)


# Depend on system install of Gazebo and SDFormat
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS
)

# build plugins
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
add_library(mbzirc_gazebo_truck_plugin src/mbzirc_gazebo_truck_plugin.cpp)
target_link_libraries(mbzirc_gazebo_truck_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############


#############
## Testing ##
#############
