
#include <jsk_mbzirc_tasks/keypoint_classifier.h>

KeyPointClassifier::KeyPointClassifier() {

}

bool KeyPointClassifier::setForeGroundFeatures(
    const cv::Mat &features, const int label) {
    if (features.empty()) {
        ROS_ERROR("Foreground Feature is Empty");
        return false;
    }
    this->features_fore_ = cv::Mat(features.rows, features.cols, CV_32F);
    this->fore_labels_ = cv::Mat(features.rows, sizeof(char), CV_32S);
    for (int j = 0; j < features.rows; j++) {
        for (int i = 0; i < features.cols; i++) {
            this->features_fore_.at<float>(j, i) = static_cast<float>(
                features.at<uchar>(j, i));
            if (isnan(features.at<float>(j, i))) {
                this->features_fore_.at<float>(j, i) = 0.0f;
            }
        }
        cv::normalize(this->features_fore_.row(j), this->features_fore_.row(j),
                      0, 1, cv::NORM_MINMAX, -1, cv::Mat());        
        this->fore_labels_.at<int>(j, 0) = label;
    }
    return true;
}

bool KeyPointClassifier::setBackGroundFeatures(
    const cv::Mat &features, const int label) {
    if (features.empty()) {
        ROS_ERROR("Background Feature is Empty");
        return false;
    }
    this->features_back_ = cv::Mat(features.rows, features.cols, CV_32F);
    this->back_labels_ = cv::Mat(features.rows, sizeof(char), CV_32S);
    for (int j = 0; j < features.rows; j++) {
        for (int i = 0; i < features.cols; i++) {
            this->features_back_.at<float>(j, i) = static_cast<float>(
                features.at<uchar>(j, i));
            if (isnan(features.at<float>(j, i))) {
                this->features_back_.at<float>(j, i) = 0.0f;
            }
        }
        cv::normalize(this->features_back_.row(j), this->features_back_.row(j),
                      0, 1, cv::NORM_MINMAX, -1, cv::Mat());        
        this->back_labels_.at<int>(j, 0) = label;
    }
    return true;
}

bool KeyPointClassifier::buildTrainingFeatures(
    cv::Mat &features, cv::Mat &labels) {
    if (this->features_fore_.cols != this->features_back_.cols) {
        ROS_ERROR("FEATURES FOR TRAINING ARE NOT EQUAL DIMENSION");
        return false;
    }
    cv::vconcat(this->features_fore_, this->features_back_, features);
    cv::vconcat(this->fore_labels_, this->back_labels_, labels);
    
    if (features.rows != (this->features_back_.rows + this->features_fore_.rows)
        || features.cols != this->features_back_.cols) {
        ROS_ERROR("FEATURE CONCATENATION FAILED");
        return false;
    }
    return true;
}

bool KeyPointClassifier::trainClassifier() {
    this->svm_ = cv::ml::SVM::create();
    this->svm_->setType(cv::ml::SVM::C_SVC);
    this->svm_->setKernel(cv::ml::SVM::INTER);
    this->svm_->setDegree(0.0);
    this->svm_->setGamma(0.90);
    this->svm_->setCoef0(0.70);
    this->svm_->setC(100);
    this->svm_->setNu(0.70);
    this->svm_->setP(1.0);
    // this->svm_->setClassWeights(cv::Mat());
    cv::TermCriteria term_crit  = cv::TermCriteria(
        cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
        static_cast<int>(1e5), FLT_EPSILON);
    this->svm_->setTermCriteria(term_crit);
    cv::Ptr<cv::ml::ParamGrid> param_grid = new cv::ml::ParamGrid();
    param_grid->minVal = 0;
    param_grid->maxVal = 0;
    param_grid->logStep = 1;

    cv::Mat features;
    cv::Mat labels;
    if (this->buildTrainingFeatures(features, labels)) {
        this->svm_->train(features, cv::ml::ROW_SAMPLE, labels);
    } else {
        return false;
    }
}

float KeyPointClassifier::predict(const cv::Mat &descriptor) {
    if (descriptor.empty() || descriptor.cols != this->features_fore_.cols) {
        ROS_ERROR("INCORRECT TEST DESCRIPTOR TYPE");
        return std::numeric_limits<float>::infinity();
    }
    return this->svm_->predict(descriptor);
}

void KeyPointClassifier::filterProbableOutliers(
    cv::Mat image, cv::Mat &descriptor, std::vector<cv::KeyPoint> &keypoints) {
    if (descriptor.rows != keypoints.size()) {
        ROS_ERROR("NOT EQUAL SIZE");
        return;
    }
    int indices[static_cast<int>(keypoints.size())];
#ifdef _OPENMP
#pragma omp parallel for num_threads(8) shared(indices)
#endif
    for (int i = 0; i < descriptor.rows; i++) {
        cv::Mat query = cv::Mat(1, descriptor.row(i).cols, CV_32F);
        for (int j = 0; j < query.cols; j++) {
            query.at<float>(0, j) = static_cast<float>(
                descriptor.row(i).at<uchar>(0, j));
            if (isnan(query.at<float>(0, j))) {
                query.at<float>(0, j) = 0.0f;
            }
        }
        cv::normalize(query, query, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());        
        float response = this->predict(query);
        indices[i] = static_cast<int>(response);
    }
    cv::Mat inliers_desc;
    std::vector<cv::KeyPoint> inliers_key;
    for (int i = 0; i < descriptor.rows; i++) {
        if (indices[i] == 1) {
            inliers_desc.push_back(descriptor.row(i));
            inliers_key.push_back(keypoints.at(i));
        }
    }

    cv::cvtColor(image, image, CV_GRAY2BGR);
    cv::drawKeypoints(image, inliers_key, image);
    cv::namedWindow("inlier", cv::WINDOW_NORMAL);
    cv::imshow("inlier", image);
    cv::waitKey(3);
    
    keypoints.clear();
    descriptor.release();

    descriptor = inliers_desc.clone();
    keypoints = inliers_key;
}
