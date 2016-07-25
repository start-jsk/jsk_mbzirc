cmake_minimum_required(VERSION 2.8.3)
project(netusb_camera_driver)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  dynamic_reconfigure
  image_transport
  nodelet
  roscpp
  sensor_msgs
)

generate_dynamic_reconfigure_options(
  cfg/NETUSBCamera.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure nodelet roscpp
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
find_package(udev REQUIRED)
find_package(NETUSBCAM REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(NETUSBCamera src/NETUSBCamera.cpp)
target_link_libraries(NETUSBCamera ${NETUSBCAM_LIBRARIES} ${catkin_LIBRARIES})

add_library(NETUSBCameraNodelet src/netusb_camera_nodelet.cpp)
target_link_libraries(NETUSBCameraNodelet NETUSBCamera ${catkin_LIBRARIES})
add_dependencies(NETUSBCameraNodelet ${PROJECT_NAME}_gencfg)

add_executable(netusb_camera_node src/netusb_camera_node.cpp)
target_link_libraries(netusb_camera_node NETUSBCamera ${catkin_LIBRARIES})
add_dependencies(netusb_camera_node ${PROJECT_NAME}_gencfg)

install(TARGETS
  NETUSBCamera
  NETUSBCameraNodelet
  netusb_camera_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
  ${NETUSBCAM_LIBRARIES}
  ${LIBUDEV_LIBRARIES}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

