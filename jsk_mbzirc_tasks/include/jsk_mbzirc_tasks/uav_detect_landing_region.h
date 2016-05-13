
#pragma once
#ifndef _UAV_DETECT_LANDING_REGION_H_
#define _UAV_DETECT_LANDING_REGION_H_

#include <omp.h>

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

#include <geometry_msgs/PolygonStamped.h>
#include <jsk_recognition_msgs/Rect.h>
#include <jsk_recognition_msgs/VectorArray.h>
#include <jsk_mbzirc_tasks/histogram_of_oriented_gradients.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/mutex.hpp>

namespace jsk_msgs = jsk_recognition_msgs;

class UAVLandingRegion {

 private:
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    message_filters::Subscriber<sensor_msgs::Image> sub_mask_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;

    int num_threads_;
    cv::Mat templ_img_;
    int down_size_;
    float ground_plane_;

    float track_width_;
    boost::shared_ptr<HOGFeatureDescriptor> hog_;
    cv::Ptr<cv::ml::SVM> svm_;
   
 protected:
    ros::NodeHandle pnh_;
    ros::Publisher pub_image_;
    ros::Publisher pub_rect_;
   
    void onInit();
    void subscribe();
    void unsubscribe();

    enum {
       EVEN, ODD};
   
    
 public:
    UAVLandingRegion();
    virtual void imageCB(const sensor_msgs::Image::ConstPtr &,
                         const sensor_msgs::Image::ConstPtr &);
    void slidingWindowDetect(cv::Mat &, const cv::Mat);
    void skeletonization(cv::Mat &);
    void iterativeThinning(cv::Mat&, int);
    void traceandDetectLandingMarker(const cv::Mat,
                                     const cv::Mat,
                                     const cv::Size);
    cv::Mat convertImageToMat(const sensor_msgs::Image::ConstPtr &,
                              std::string);
    cv::Size getSlidingWindowSize(const cv::Mat,
                                  const jsk_msgs::VectorArray);

    //! Training
    cv::Mat extractFeauture(cv::Mat &);
    bool trainTrackDetector(const std::string,
                            const std::string);
    void trainSVM(const cv::Mat, const cv::Mat, std::string);
};


#endif  // _UAV_DETECT_LANDING_REGION_H_
