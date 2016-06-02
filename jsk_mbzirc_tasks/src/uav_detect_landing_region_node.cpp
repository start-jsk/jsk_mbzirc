
#include <jsk_mbzirc_tasks/uav_detect_landing_region.h>

UAVLandingRegion::UAVLandingRegion() :
    down_size_(2), ground_plane_(0.0), track_width_(3.0f),
    landing_marker_width_(1.5f), min_wsize_(8), nms_thresh_(0.01f) {

    this->nms_client_ = this->pnh_.serviceClient<
       jsk_tasks::NonMaximumSuppression>("non_maximum_suppression");
   
    //! svm load or save path
    std::string svm_path = "/home/krishneel/Desktop/mbzirc/data/svm.xml";
    // this->pnh_.getParam("/uav_detect_landing_region/svm_path", svm_path);
    if (svm_path.empty()) {
       ROS_ERROR("NOT SVM DETECTOR PATH. PROVIDE A VALID PATH");
       return;
    }
     
    //! train svm
    bool is_train = !true;
    // this->pnh_.getParam("/uav_detect_landing_region/train_detector",
    // is_train);
    if (is_train) {
       std::string object_data_path;
       std::string background_dataset_path;
       this->pnh_.getParam("/uav_detect_landing_region/object_dataset_path",
                           object_data_path);
       this->pnh_.getParam("/uav_detect_landiang_region/object_dataset_path",
                           background_dataset_path);

       object_data_path = "/home/krishneel/Desktop/mbzirc/data/positive.txt";
       background_dataset_path =
          "/home/krishneel/Desktop/mbzirc/data/negative.txt";
       
       this->trainUAVLandingRegionDetector(object_data_path,
                                           background_dataset_path, svm_path);
       
       ROS_INFO("\033[34m-- SVM DETECTOR SUCCESSFULLY TRAINED \033[0m");
    }
    this->svm_->load(static_cast<std::string>(svm_path));
    ROS_INFO("\033[34m-- SVM DETECTOR SUCCESSFULLY LOADED \033[0m");
    
    this->onInit();
}

void UAVLandingRegion::onInit() {
    this->subscribe();
    this->pub_image_ = pnh_.advertise<sensor_msgs::Image>(
       "/image", sizeof(char));
    this->pub_point_ = pnh_.advertise<geometry_msgs::PointStamped>(
       "/uav_landing_region/output/point", sizeof(char));
}

void UAVLandingRegion::subscribe() {
    this->sub_image_.subscribe(this->pnh_, "input_image", 1);
    this->sub_mask_.subscribe(this->pnh_, "input_mask", 1);
    this->sub_proj_.subscribe(this->pnh_, "input_proj_mat", 1);
    this->sync_ = boost::make_shared<message_filters::Synchronizer<
       SyncPolicy> >(100);
    this->sync_->connectInput(
       this->sub_image_, this->sub_mask_, this->sub_proj_);
    this->sync_->registerCallback(
      boost::bind(
         &UAVLandingRegion::imageCB, this, _1, _2, _3));
}

void UAVLandingRegion::unsubscribe() {
    this->sub_image_.unsubscribe();
    this->sub_mask_.unsubscribe();
}

void UAVLandingRegion::imageCB(
    const sensor_msgs::Image::ConstPtr &image_msg,
    const sensor_msgs::Image::ConstPtr &mask_msg,
    const jsk_msgs::VectorArray::ConstPtr &proj_mat_msg) {
    
    cv::Mat image = this->convertImageToMat(image_msg, "bgr8");
    cv::Mat im_mask = this->convertImageToMat(mask_msg, "mono8");
    if (image.empty() || im_mask.empty()) {
       ROS_ERROR("EMPTY IMAGE. SKIP LANDING SITE DETECTION");
       return;
    }

    cv::Size im_downsize = cv::Size(image.cols/this->down_size_,
                                    image.rows/this->down_size_);
    cv::resize(image, image, im_downsize);
    cv::resize(im_mask, im_mask, im_downsize);
    
    cv::Mat detector_path = im_mask.clone();

    cv::Size wsize = this->getSlidingWindowSize(*proj_mat_msg);
    if (wsize.width < this->min_wsize_) {
       ROS_WARN("HIGH ALTITUDE. SKIPPING DETECTION");
       return;
    }
    
    cv::Point2f marker_point = this->traceandDetectLandingMarker(
       image, im_mask, wsize);
    if (marker_point.x == -1) {
       return;
    }
    geometry_msgs::PointStamped ros_point = this->pointToWorldCoords(
       *proj_mat_msg, marker_point.x, marker_point.y);
    ros_point.header = image_msg->header;
    this->pub_point_.publish(ros_point);
    
    cv_bridge::CvImagePtr pub_msg(new cv_bridge::CvImage);
    pub_msg->header = image_msg->header;
    pub_msg->encoding = sensor_msgs::image_encodings::BGR8;
    pub_msg->image = image.clone();
    this->pub_image_.publish(pub_msg);

    cv::waitKey(5);
}

cv::Point2f UAVLandingRegion::traceandDetectLandingMarker(
    cv::Mat img, const cv::Mat marker, const cv::Size wsize) {
    if (img.empty() || marker.empty() || img.size() != marker.size()) {
        ROS_ERROR("EMPTY INPUT IMAGE FOR DETECTION");
        return cv::Point2f(-1, -1);
    }
    // cv::Mat image = marker.clone();
    cv::Mat image = img.clone();
    if (image.type() != CV_8UC1) {
       cv::cvtColor(image, image, CV_BGR2GRAY);
    }
    cv::Mat im_edge;
    cv::Canny(image, im_edge, 50, 100);
    cv::Mat weight = img.clone();

    jsk_tasks::NonMaximumSuppression nms_srv;
    //! 1 - detect
#ifdef _OPENMP
#pragma omp parallel for num_threads(8)
#endif
    for (int j = 0; j < im_edge.rows; j += 2) {
       for (int i = 0; i < im_edge.cols; i += 2) {
          if (static_cast<int>(im_edge.at<uchar>(j, i)) != 0) {
             cv::Rect rect = cv::Rect(i, j, wsize.width, wsize.height);
            if (rect.x + rect.width < image.cols &&
                rect.y + rect.height < image.rows) {
                cv::Mat roi = img(rect).clone();
                cv::resize(roi, roi, this->sliding_window_size_);
                
                cv::Mat desc = this->extractFeauture(roi);
                float response = this->svm_->predict(desc);
                if (response == 1) {
                   jsk_msgs::Rect bbox;
                   bbox.x = rect.x;
                   bbox.y = rect.y;
                   bbox.width = rect.width;
                   bbox.height = rect.height;
                   nms_srv.request.rect.push_back(bbox);
                   nms_srv.request.probabilities.push_back(response);
                   
                   // cv::rectangle(weight, rect, cv::Scalar(0, 255, 0), 1);
                }
            }
          }
       }
    }
    nms_srv.request.threshold = this->nms_thresh_;

    //! 2 - non_max_suprresion
    cv::Point2f center = cv::Point2f(-1, -1);
    if (this->nms_client_.call(nms_srv)) {
       for (int i = 0; i < nms_srv.response.bbox_count; i++) {
          cv::Rect_<int> rect = cv::Rect_<int>(
             nms_srv.response.bbox[i].x,
             nms_srv.response.bbox[i].y,
             nms_srv.response.bbox[i].width,
             nms_srv.response.bbox[i].height);
          
          center.x = rect.x + rect.width / 2;
          center.y = rect.y + rect.height / 2;

          // for viz
          cv::Point2f vert1 = cv::Point2f(center.x, center.y - wsize.width);
          cv::Point2f vert2 = cv::Point2f(center.x, center.y + wsize.width);
          cv::Point2f hori1 = cv::Point2f(center.x - wsize.width, center.y);
          cv::Point2f hori2 = cv::Point2f(center.x + wsize.width, center.y);
          cv::line(weight, vert1, vert2, cv::Scalar(0, 0, 255), 1);
          cv::line(weight, hori1, hori2, cv::Scalar(0, 0, 255), 1);
          cv::rectangle(weight, rect, cv::Scalar(0, 255, 0), 1);
       }
    } else {
       ROS_FATAL("NON-MAXIMUM SUPPRESSION SRV NOT CALLED");
       return cv::Point2f(-1, -1);
    }
    
    // 3 - return bounding box
    // TODO(REMOVE OTHER FALSE POSITIVES): HERE?
    
    std::string wname = "result";
    cv::namedWindow(wname, cv::WINDOW_NORMAL);
    cv::imshow(wname, weight);

    return center;
}

cv::Size UAVLandingRegion::getSlidingWindowSize(
    const jsk_msgs::VectorArray projection_matrix) {
    float A[2][2];
    float bv[2];

    const int NUM_POINTS = 2;
    const float pixel_lenght = 10;
    float init_point = 10;
    cv::Point2f point[NUM_POINTS];
    point[0] = cv::Point2f(init_point, init_point);
    point[1] = cv::Point2f(init_point + pixel_lenght,
                           init_point + pixel_lenght);

    cv::Point3_<float> world_coords[NUM_POINTS];
    for (int k = 0; k < NUM_POINTS; k++) {
       int i = static_cast<int>(point[k].y);
       int j = static_cast<int>(point[k].x);
          
       A[0][0] = j * projection_matrix.data.at(8) -
          projection_matrix.data.at(0);
       A[0][1] = j * projection_matrix.data.at(9) -
          projection_matrix.data.at(1);
       A[1][0] = i * projection_matrix.data.at(8) -
          projection_matrix.data.at(4);
       A[1][1] = i * projection_matrix.data.at(9) -
          projection_matrix.data.at(5);
       bv[0] = projection_matrix.data.at(2)*ground_plane_ +
          projection_matrix.data.at(3) - j*projection_matrix.data.at(
             10)*ground_plane_ - j*projection_matrix.data.at(11);
       bv[1] = projection_matrix.data.at(4)*ground_plane_ +
          projection_matrix.data.at(7) - i*projection_matrix.data.at(
             10)*ground_plane_ - i*projection_matrix.data.at(11);
       float dominator = A[1][1] * A[0][0] - A[0][1] * A[1][0];

       world_coords[k].x = (A[1][1]*bv[0]-A[0][1]*bv[1]) / dominator;
       world_coords[k].y = (A[0][0]*bv[1]-A[1][0]*bv[0]) / dominator;
       world_coords[k].z = this->ground_plane_;
    }
    
    float world_distance = this->EuclideanDistance(world_coords);
    float wsize = (pixel_lenght * landing_marker_width_) / world_distance;

    return cv::Size(static_cast<int>(wsize), static_cast<int>(wsize));
}

float UAVLandingRegion::EuclideanDistance(
    const cv::Point3_<float> *world_coords) {
    float x = world_coords[1].x - world_coords[0].x;
    float y = world_coords[1].y - world_coords[0].y;
    float z = world_coords[1].z - world_coords[0].z;
    return std::sqrt((std::pow(x, 2) + (std::pow(y, 2)) + (std::pow(z, 2))));
}

cv::Mat UAVLandingRegion::convertImageToMat(
    const sensor_msgs::Image::ConstPtr &image_msg, std::string encoding) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
       cv_ptr = cv_bridge::toCvCopy(image_msg, encoding);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
    return cv_ptr->image.clone();
}

geometry_msgs::PointStamped UAVLandingRegion::pointToWorldCoords(
    const jsk_msgs::VectorArray projection_matrix,
    const float x, const float y) {
    float A[2][2];
    float bv[2];
    int i = static_cast<int>(y);
    int j = static_cast<int>(x);
    A[0][0] = j * projection_matrix.data.at(8) -
       projection_matrix.data.at(0);
    A[0][1] = j * projection_matrix.data.at(9) -
       projection_matrix.data.at(1);
    A[1][0] = i * projection_matrix.data.at(8) -
       projection_matrix.data.at(4);
    A[1][1] = i * projection_matrix.data.at(9) -
       projection_matrix.data.at(5);
    bv[0] = projection_matrix.data.at(2)*ground_plane_ +
       projection_matrix.data.at(3) - j*projection_matrix.data.at(
             10)*ground_plane_ - j*projection_matrix.data.at(11);
    bv[1] = projection_matrix.data.at(4)*ground_plane_ +
       projection_matrix.data.at(7) - i*projection_matrix.data.at(
          10)*ground_plane_ - i*projection_matrix.data.at(11);
    float dominator = A[1][1] * A[0][0] - A[0][1] * A[1][0];

    geometry_msgs::PointStamped world_coords;
    world_coords.point.x = (A[1][1]*bv[0]-A[0][1]*bv[1]) / dominator;
    world_coords.point.y = (A[0][0]*bv[1]-A[1][0]*bv[0]) / dominator;
    world_coords.point.z = this->ground_plane_;
    return world_coords;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "jsk_mbzirc_tasks");
    UAVLandingRegion ulr;
    ros::spin();
    return 0;
}
