
#ifndef _KEYPOINT_CLASSIFIER_H_
#define _KEYPOINT_CLASSIFIER_H_

#include <ros/ros.h>
#include <ros/console.h>

#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>

class KeyPointClassifier {

private:
    cv::Mat features_fore_;
    cv::Mat fore_labels_;
    cv::Mat features_back_;
    cv::Mat back_labels_;

    bool buildTrainingFeatures(cv::Mat &, cv::Mat &);
    float predict(const cv::Mat &);
    
protected:
    cv::Ptr<cv::ml::SVM> svm_;
    
public:
    KeyPointClassifier();
    bool setForeGroundFeatures(const cv::Mat &, const int = 1);
    bool setBackGroundFeatures(const cv::Mat &, const int = -1);
    bool trainClassifier();
    void filterProbableOutliers(cv::Mat, cv::Mat &, std::vector<cv::KeyPoint> &);
};


#endif  // _KEYPOINT_CLASSIFIER_H_
