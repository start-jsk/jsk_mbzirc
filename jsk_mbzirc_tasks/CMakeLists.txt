cmake_minimum_required(VERSION 2.8.3)
project(jsk_mbzirc_tasks)
#set(CMAKE_BUILD_TYPE Debug)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
      roscpp cv_bridge image_transport tf message_filters nav_msgs sensor_msgs jsk_mbzirc_msgs)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

add_service_files(
  FILES
  NonMaximumSuppression.srv)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  jsk_mbzirc_msgs)

#add gpu support
set(ENABLE_CUDA OFF)

find_package(OpenMP)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")


###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS
)

set(PCL_INCLUDE_DIRS "/usr/include/pcl-1.7/")
set(EIGEN_INCLUDE_DIRS "/usr/include/eigen3/")

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

if(ENABLE_CUDA)
    find_package(CUDA REQUIRED)
    set(CUDA_INCLUDE_DIRS "/usr/local/cuda/")
    include_directories(${CUDA_INCLUDE_DIRS})
    #add gpu support for uav_img2pointcloud node
    add_definitions(-DGPU_EN)
    cuda_add_library(UAV_IMG2POINTCLOUD_CU_LIB ${LIB_TYPE} src/img2pointcloud/uav_img2pointcloud.cu)
    add_executable(uav_img2pointcloud src/img2pointcloud/uav_img2pointcloud.cpp)
    add_dependencies(uav_img2pointcloud UAV_IMG2POINTCLOUD_CU_LIB)
    target_link_libraries(uav_img2pointcloud ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CUDA_LIBRARIES} UAV_IMG2POINTCLOUD_CU_LIB)
else()
    add_executable(uav_img2pointcloud src/img2pointcloud/uav_img2pointcloud.cpp)
    target_link_libraries(uav_img2pointcloud ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
endif(ENABLE_CUDA)

#uav_teleop_keyboard node
add_executable(uav_teleop_keyboard src/teleop/uav_teleop_keyboard.cpp)
target_link_libraries(uav_teleop_keyboard ${catkin_LIBRARIES})
#uav_teleop_joy node
add_executable(uav_teleop_joy src/teleop/uav_teleop_joy.cpp)
target_link_libraries(uav_teleop_joy ${catkin_LIBRARIES})

if (OpenCV_VERSION VERSION_GREATER "3.0")
  add_executable(uav_tracking_node
    src/tracking/uav_tracking_node.cpp
    src/tracking/CMT.cpp
    src/tracking/keypoint_classifier.cpp
    )

  target_link_libraries(uav_tracking_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    )

#build for landing site detection
  add_executable(uav_detect_landing_region_node
    src/uav_detect_landing_region_node.cpp
    src/histogram_of_oriented_gradients.cpp
    src/uav_detect_landing_region_trainer.cpp
)
  target_link_libraries(uav_detect_landing_region_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    )


endif()

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
