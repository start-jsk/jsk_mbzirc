cmake_minimum_required(VERSION 2.8.3)
project(jsk_mbzirc_tasks)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS
)

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
  roslaunch_add_file_check(launch/jsk_mbzirc_task_1.launch)
  roslaunch_add_file_check(launch/jsk_mbzirc_task_2.launch)
  add_rostest(test/jsk_mbzirc_task_1.test)
endif()
