
#ifndef _UAV_SELF_FILTER_H_
#define _UAV_SELF_FILTER_H_

#include <ros/ros.h>
#include <ros/console.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

class UAVSelfFilter {

private:
    int thresh_;
    cv::Mat mask_;
    bool is_mask_;
    
protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();

    ros::NodeHandle pnh_;
    ros::Subscriber sub_image_;
    ros::Publisher pub_image_;
    
 public:
    UAVSelfFilter();
    virtual void imageCB(const sensor_msgs::Image::ConstPtr &);
    void filter(cv::Mat &,  cv::Mat &);
};


#endif  // _UAV_SELF_FILTER_H_
