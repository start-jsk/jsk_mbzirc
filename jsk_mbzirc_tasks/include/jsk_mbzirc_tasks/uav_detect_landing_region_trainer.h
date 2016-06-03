
#pragma once
#ifndef _UAV_DETECT_LANDING_REGION_TRAINER_H_
#define _UAV_DETECT_LANDING_REGION_TRAINER_H_

#include <ros/ros.h>
#include <ros/console.h>

#include <jsk_mbzirc_tasks/histogram_of_oriented_gradients.h>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/mutex.hpp>

class UAVLandingRegionTrainer {

 private:
    int stride_;
    std::string positive_data_path_;
    std::string negative_data_path_;
    std::string data_directory_;
   
 public:
    UAVLandingRegionTrainer();
    void trainUAVLandingRegionDetector(const std::string, const std::string,
                                       const std::string, const std::string);
    void getTrainingDataset(cv::Mat &, cv::Mat &, const std::string);
    void uploadDataset(const std::string);
    cv::Mat extractFeauture(cv::Mat &);
    void trainSVM(const cv::Mat, const cv::Mat, std::string);

    boost::shared_ptr<HOGFeatureDescriptor> hog_;
    cv::Ptr<cv::ml::SVM> svm_;
    cv::Size sliding_window_size_;
    std::string svm_save_path_;
};


#endif  // _UAV_DETECT_LANDING_REGION_TRAINER_H_
