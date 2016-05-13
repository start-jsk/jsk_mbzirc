
#include <jsk_mbzirc/tasks/uav_detect_landing_region_trainer.h>

UAVLandingRegionTrainer::UAVLandingRegionTrainer() :
    stride_(8) {
    this->hog_ = boost::shared_ptr<HOGFeatureDescriptor>(
       new HOGFeatureDescriptor());

    //! set window size
    this->sliding_wsize_ = cv::Size(0, 0);
    
    //! get paths
    this->positive_data_path_ = "";
    this->negative_data_path_ = "";
    this->svm_save_path_ = "";
}

void UAVLandingRegionTrainer::uploadDataset(
    const std::string dpath) {
    if (!dpath.empty()) {
       ROS_ERROR("PROVIDE DATASET TEXT FILE PATH");
       return;
    }
    // read text file and extract features and labels
    cv::Mat feature_vector;
    cv::Mat labels;
    this->getTrainingDataset(feature_vector, labels, this->positive_data_path_);
    this->getTrainingDataset(feature_vector, labels, this->negative_data_path_);

    // train
    this->trainSVM(feature_vector, labels, this->svm_save_path_);
    
    // generate manifest
    
}

void UAVLandingRegionTrainer::getTrainingDataset(
    cv::Mat &feature_vector, cv::Mat &labels, const std::string directory) {
    if (directory.empty()) {
      std::cout << "PROVIDE DATASET TEXT FILE PATH"  << "\n";
      return;
    }
    std::cout << "Reading Training Dataset......" << std::endl;
    char buffer[255];
    std::ifstream in_file;
    in_file.open((directory).c_str(), std::ios::in);
    if (!in_file.eof()) {
       while (in_file.good()) {
          in_file.getline(buffer, 255);
          std::string read_line(buffer);
          if (!read_line.empty()) {
             std::istringstream iss(read_line);
             std::string img_path;
             iss >> img_path;
             std::string l;
             iss >> l;
             cv::Mat img = imread(img_path, CV_LOAD_IMAGE_GRAYSCALE);
             if (img.data) {
                cv::resize(img, img, this->sliding_wsize_);
                cv::Mat desc = this->extractFeauture(img);
                if (desc.data) {
                   feature_vector.push_back(desc);
                   float lab = std::atof(l.c_str());
                   labels.push_back(lab);
                }
             }
          }
       }
    }
    std::cout << "Training Dataset Reading Completed......" << std::endl;
}


cv::Mat UAVLandingRegionTrainer::extractFeauture(
    cv::Mat &image) {
    if (image.empty()) {
      return cv::Mat();
    }
    cv::Mat desc = this->hog_->computeHOG(image);
    return desc;
}

void UAVLandingRegionTrainer::trainSVM(
    const cv::Mat feature_vector, cv::Mat labels, std::string save_path) {
    if (feature_vector.empty() || feature_vector.size() != labels.size()) {
       ROS_ERROR("TRAINING FAILED DUE TO UNEVEN DATA");
       return;
    }
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
    this->svm_->train(feature_vector, cv::ml::ROW_SAMPLE, labels);

    if (!save_path.empty()) {
       this->svm_->save(static_cast<std::string>(save_path));
    }
    ROS_INFO("\033[34mSVM SUCCESSFULLY TRAINED AND SAVED TO\033[0m");
}
