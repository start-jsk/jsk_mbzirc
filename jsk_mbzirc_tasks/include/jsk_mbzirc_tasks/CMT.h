#ifndef CMT_H
#define CMT_H

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <jsk_mbzirc_tasks/keypoint_classifier.h>

class CMT {

private:
    boost::shared_ptr<KeyPointClassifier> classifier_;
    std::string mask_image_filename_;
    
public:
    std::string detectorType;
    std::string descriptorType;
    std::string matcherType;
    int descriptorLength;
    int thrOutlier;
    float thrConf;
    float thrRatio;

    bool estimateScale;
    bool estimateRotation;

#if CV_MAJOR_VERSION < 3
    cv::Ptr<cv::FeatureDetector> detector;
#elif CV_MAJOR_VERSION >= 3
    cv::Ptr<cv::BRISK> detector;
#endif
    cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;
    cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;

    cv::Mat selectedFeatures;
    std::vector<int> selectedClasses;
    cv::Mat featuresDatabase;
    std::vector<int> classesDatabase;

    std::vector<std::vector<float> > squareForm;
    std::vector<std::vector<float> > angles;

    cv::Point2f topLeft;
    cv::Point2f topRight;
    cv::Point2f bottomRight;
    cv::Point2f bottomLeft;

    cv::Rect_<float> boundingbox;
    bool hasResult;


    cv::Point2f centerToTopLeft;
    cv::Point2f centerToTopRight;
    cv::Point2f centerToBottomRight;
    cv::Point2f centerToBottomLeft;

    std::vector<cv::Point2f> springs;

    cv::Mat im_prev;
    std::vector<std::pair<cv::KeyPoint,int> > activeKeypoints;
    std::vector<std::pair<cv::KeyPoint,int> > trackedKeypoints;

    unsigned int nbInitialKeypoints;

    std::vector<cv::Point2f> votes;

    std::vector<std::pair<cv::KeyPoint, int> > outliers;

    CMT();
    void initialise(cv::Mat im_gray0, cv::Point2f topleft, cv::Point2f bottomright);
    void estimate(const std::vector<std::pair<cv::KeyPoint, int> >& keypointsIN, cv::Point2f& center, float& scaleEstimate, float& medRot, std::vector<std::pair<cv::KeyPoint, int> >& keypoints);
    void processFrame(cv::Mat im_gray);

    void track(cv::Mat im_prev, cv::Mat im_gray,
               const std::vector<std::pair<cv::KeyPoint, int> >& keypointsIN,
               std::vector<std::pair<cv::KeyPoint, int> >& keypointsTracked,
               std::vector<unsigned char>& status, int THR_FB = 20);
    
    void pyramidKeypoints(std::vector<cv::KeyPoint> &,
                          std::vector<cv::KeyPoint> &,
                          const cv::Mat &image, const float scale, const int level,
                          const cv::Point2f topleft, const cv::Point2f bottomright);
    void maskUAVKeyPoints(std::vector<cv::KeyPoint> &, const std::string);
};

class Cluster
{
public:
    int first, second;//cluster id
    float dist;
    int num;
};

void inout_rect(const std::vector<cv::KeyPoint>& keypoints, cv::Point2f topleft, cv::Point2f bottomright, std::vector<cv::KeyPoint>& in, std::vector<cv::KeyPoint>& out);
// void track(cv::Mat im_prev, cv::Mat im_gray, const std::vector<std::pair<cv::KeyPoint, int> >& keypointsIN, std::vector<std::pair<cv::KeyPoint, int> >& keypointsTracked, std::vector<unsigned char>& status, int THR_FB = 20);
cv::Point2f rotate(cv::Point2f p, float rad);
#endif // CMT_H
