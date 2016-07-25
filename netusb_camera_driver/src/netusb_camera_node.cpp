/*
 * netusb_camera_node.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "netusb_camera_node");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  nodelet.load(ros::this_node::getName(),
               "netusb_camera_driver/NETUSBCameraNodelet",
               remap, nargv);

  ros::spin();

  return 0;
}
