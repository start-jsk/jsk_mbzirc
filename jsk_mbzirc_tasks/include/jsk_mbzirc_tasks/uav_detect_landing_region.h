
#pragma once
#ifndef _UAV_DETECT_LANDING_REGION_H_
#define _UAV_DETECT_LANDING_REGION_H_

#include <omp.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <jsk_recognition_msgs/Rect.h>
#include <jsk_recognition_msgs/VectorArray.h>

#include <jsk_mbzirc_tasks/uav_detect_landing_region_trainer.h>
#include <jsk_mbzirc_tasks/NonMaximumSuppression.h>

namespace jsk_msgs = jsk_recognition_msgs;
namespace jsk_tasks = jsk_mbzirc_tasks;

class UAVLandingRegion: public UAVLandingRegionTrainer {

 private:
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image, jsk_msgs::VectorArray> SyncPolicy;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    message_filters::Subscriber<sensor_msgs::Image> sub_mask_;
    message_filters::Subscriber<jsk_msgs::VectorArray> sub_proj_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;

    int num_threads_;
    cv::Mat templ_img_;
    int down_size_;
    int min_wsize_;
    float nms_thresh_;
   
    float track_width_;
    float landing_marker_width_;
    float ground_plane_;
    
 protected:
    ros::NodeHandle pnh_;
    ros::Publisher pub_image_;
    ros::Publisher pub_point_;
    ros::ServiceClient nms_client_;
   
    void onInit();
    void subscribe();
    void unsubscribe();
    
 public:
    UAVLandingRegion();
    virtual void imageCB(const sensor_msgs::Image::ConstPtr &,
                         const sensor_msgs::Image::ConstPtr &,
                         const jsk_msgs::VectorArray::ConstPtr &);
    cv::Point2f traceandDetectLandingMarker(cv::Mat, const cv::Mat,
                                            const cv::Size);
    cv::Mat convertImageToMat(const sensor_msgs::Image::ConstPtr &,
                              std::string);
    cv::Size getSlidingWindowSize(const jsk_msgs::VectorArray);
    float EuclideanDistance(const cv::Point3_<float> *);
    geometry_msgs::PointStamped pointToWorldCoords(
       const jsk_msgs::VectorArray, const float, const float);
};


#endif  // _UAV_DETECT_LANDING_REGION_H_
