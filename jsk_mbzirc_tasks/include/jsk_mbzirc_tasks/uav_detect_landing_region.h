
#pragma once
#ifndef _UAV_DETECT_LANDING_REGION_H_
#define _UAV_DETECT_LANDING_REGION_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/video/background_segm.hpp>

#include <boost/thread/mutex.hpp>

#include <geometry_msgs/PolygonStamped.h>
#include <jsk_recognition_msgs/Rect.h>

#include <jsk_mbzirc_tasks/histogram_of_oriented_gradients.h>

#include <omp.h>

class UAVLandingRegion {

 private:
    cv::Mat createLandingTemplate(const cv::Size);
    void detect(cv::Mat &);
 protected:
    ros::NodeHandle pnh_;
    ros::Publisher pub_image_;
    ros::Publisher pub_rect_;
    ros::Subscriber sub_image_;
    ros::Subscriber sub_screen_pt_;
   
    void onInit();
    void subscribe();
    void unsubscribe();
    
 public:
    UAVLandingRegion();
    virtual void imageCB(const sensor_msgs::Image::ConstPtr &);
};


#endif  // _UAV_DETECT_LANDING_REGION_H_
