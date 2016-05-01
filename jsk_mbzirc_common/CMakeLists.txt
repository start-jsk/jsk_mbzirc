cmake_minimum_required(VERSION 2.8.3)
project(jsk_mbzirc_common)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS gazebo_ros std_msgs geometry_msgs)


# Depend on system install of Gazebo and SDFormat
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread random)

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

add_library(mbzirc_gazebo_panel_plugin src/mbzirc_gazebo_panel_plugin.cpp)
target_link_libraries(mbzirc_gazebo_panel_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(mbzirc_gazebo_treasure_plugin src/mbzirc_gazebo_treasure_plugin.cpp)
target_link_libraries(mbzirc_gazebo_treasure_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############
install(TARGETS
  mbzirc_gazebo_truck_plugin mbzirc_gazebo_panel_plugin mbzirc_gazebo_treasure_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY launch gazebo_model
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)


#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_python()
  roslint_add_test()
  roslaunch_add_file_check(launch/mbzirc_arena.launch)
  roslaunch_add_file_check(launch/mbzirc_arena_1.launch)
  roslaunch_add_file_check(launch/mbzirc_arena_2.launch)
  roslaunch_add_file_check(launch/mbzirc_arena_3.launch)
endif()
