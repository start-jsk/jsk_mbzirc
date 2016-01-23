cmake_minimum_required(VERSION 2.8.3)
project(jsk_mbzirc_tasks)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(uav_teleop_keyboard src/uav_teleop_keyboard.cpp)
target_link_libraries(uav_teleop_keyboard ${catkin_LIBRARIES})

#############
## Install ##
#############
install(DIRECTORY launch urdf config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)


#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
  # https://github.com/ros/ros_comm/pull/730
  set(roslaunch_check_script ${PROJECT_SOURCE_DIR}/script/roslaunch-check)
  roslaunch_add_file_check(launch/jsk_mbzirc_task_1.launch)
  roslaunch_add_file_check(launch/jsk_mbzirc_task_2.launch)
  set(roslaunch_check_script ${roslaunch_DIR}/../scripts/roslaunch-check)
  #
  add_rostest(test/jsk_mbzirc_task_1.test)
  add_rostest(test/jsk_mbzirc_task_2.test)
endif()
