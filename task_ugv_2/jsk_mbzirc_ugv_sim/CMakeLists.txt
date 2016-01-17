cmake_minimum_required(VERSION 2.8.3)
project(jsk_mbzirc_ugv_sim)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs gazebo_ros)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ugv_teleop_keyboard src/ugv_teleop_keyboard.cpp) 
target_link_libraries(ugv_teleop_keyboard ${catkin_LIBRARIES})


#############
## Install ##
#############
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############
