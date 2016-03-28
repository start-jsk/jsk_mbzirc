/*
 jsk_mbzirc_task
 */

// Author: Chen

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//msg headers.
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <iostream>


class uav_img2pointcloud
{
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo,nav_msgs::Odometry> MySyncPolicy;

private:
    ros::Publisher pointcloud_pub_;
    sensor_msgs::PointCloud2 cloud_msg;
    message_filters::Subscriber<sensor_msgs::Image> *img_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *camera_info_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> *uav_odom_sub_;
    ros::NodeHandle nh_;
    message_filters::Synchronizer<MySyncPolicy> *sync;
public:
    void init()
    {
        //publish pointcloud msg
        std::string topic = nh_.resolveName("imagetoground");
        //cloud_msg.header.frame_id = "downward_cam_optical_frame";
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(topic, 1);
        //for test the image
        cv::namedWindow("view");
        cv::startWindowThread();
        img_sub_  = new message_filters::Subscriber<sensor_msgs::Image>(nh_,"/downward_cam/camera/image",10);
        camera_info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_,"/downward_cam/camera/camera_info", 10);
        uav_odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,"/ground_truth/state",10);
        sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *img_sub_, *camera_info_sub_, *uav_odom_sub_);
        //message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> sync(*img_sub_, *camera_info_sub_, *uav_odom_sub_, 10);
        sync->registerCallback(boost::bind(&uav_img2pointcloud::imageCallback,this,_1,_2,_3));
    }
    //call back, for processing
    void imageCallback(const sensor_msgs::ImageConstPtr& img,
                       const sensor_msgs::CameraInfoConstPtr& cam_info,
                       const nav_msgs::OdometryConstPtr& odom);
    //process to pointcloud
    void p2p(const sensor_msgs::ImageConstPtr& img,
             const sensor_msgs::CameraInfoConstPtr& cam_info,
             const nav_msgs::OdometryConstPtr& odom);
    // class's family...
    ~uav_img2pointcloud()
    {    }

};

void uav_img2pointcloud::p2p(const sensor_msgs::ImageConstPtr& img,
                             const sensor_msgs::CameraInfoConstPtr& cam_info,
                             const nav_msgs::OdometryConstPtr& odom)
{
    tf::StampedTransform transform;
    cloud_msg.header.frame_id = img->header.frame_id;
    cloud_msg.height = img->height;
    cloud_msg.width = img->width;
    //pinv of projection matrix...
    cv::Mat P(3,4,CV_32FC1);
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 4; j++){
            P.at<float>(i,j) = cam_info->P.at(i*4+j);
            std::cout << P.at<float>(i,j) << " ";
        }
        std::cout << std::endl;
    }
    cv::Mat P_inv(4,3,CV_32FC1);
    P_inv = P.inv(cv::DECOMP_SVD);
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 3; j++){
            std::cout << P_inv.at<float>(i,j) << " ";
        }
        std::cout << std::endl;
    }
    //get tf
    tf::TransformListener listener;
    try{
        listener.lookupTransform(
                    "/downward_cam_optical_frame",
                    "/world",
                    ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
        ros::Duration(0.1).sleep();
    }


}



void uav_img2pointcloud::imageCallback(const sensor_msgs::ImageConstPtr& img,
                                       const sensor_msgs::CameraInfoConstPtr& cam_info,
                                       const nav_msgs::OdometryConstPtr& odom)
{
    try
    {
        cv::imshow("view", cv_bridge::toCvShare(img,"mono8")->image);
        //process to pointcloud
        p2p(img,cam_info,odom);
        cv::waitKey(30);
    }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_to_pointcloud");
    uav_img2pointcloud u_i2p;
    u_i2p.init();

    ros::spin();
    cv::destroyWindow("view");
}
