
#include <jsk_mbzirc_tasks/uav_detect_landing_region.h>

UAVLandingRegion::UAVLandingRegion() {
    
    std::string templ_path;
    pnh_.getParam("/uav_detect_landing_region/templ_path", templ_path);
    this->templ_img_ = cv::imread(templ_path, CV_LOAD_IMAGE_COLOR);
    if (this->templ_img_.empty()) {
        ROS_ERROR("TEMPLATE NOT FOUND");
        return;
    }
    this->onInit();
}

void UAVLandingRegion::onInit() {
    this->subscribe();
    this->pub_image_ = pnh_.advertise<sensor_msgs::Image>(
       "/image", sizeof(char));
    this->pub_rect_ = pnh_.advertise<jsk_recognition_msgs::Rect>(
       "/rect", sizeof(char));
}

void UAVLandingRegion::subscribe() {
    this->sub_image_ = pnh_.subscribe(
       "input", 1, &UAVLandingRegion::imageCB, this);
}

void UAVLandingRegion::unsubscribe() {
    this->sub_image_.shutdown();
}

void UAVLandingRegion::imageCB(
    const sensor_msgs::Image::ConstPtr &image_msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
       cv_ptr = cv_bridge::toCvCopy(
           image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat image = cv_ptr->image;
    
    // this->detect(image);    

    // cv::resize(image, image, cv::Size(64, 128));

    // HOGFeatureDescriptor hog;
    // cv::Mat desc = hog.computeHOG(image);
    // cv::Mat hog_viz = hog.visualizeHOG(image, desc, image.size(),
    // 1, 5);

    cv::Mat weight;
    this->slidingWindowDetect(weight, image);
    
    // std::cout << desc  << "\n";
    // std::cout << "Desc: " << desc.size()  << "\n";

    // cv::namedWindow("template", cv::WINDOW_NORMAL);
    // cv::imshow("template", hog_viz);
    cv::waitKey(5);
    
    cv_bridge::CvImagePtr pub_msg(new cv_bridge::CvImage);
    pub_msg->header = image_msg->header;
    pub_msg->encoding = sensor_msgs::image_encodings::BGR8;
    pub_msg->image = image.clone();
    this->pub_image_.publish(pub_msg);
}

void UAVLandingRegion::detect(cv::Mat &image) {
    if (image.empty()) {
        ROS_ERROR("EMPTY INPUT IMAGE FOR DETECTION");
        return;
    }
    cv::Mat im_gray;
    cv::cvtColor(image, im_gray, CV_BGR2GRAY);
    cv::GaussianBlur(im_gray, im_gray, cv::Size(3, 3), 0, 0);
    // cv::Canny(im_gray, im_gray, 10, 50, 3);

    
    
    
    std::string wname = "edges";
    cv::namedWindow(wname, cv::WINDOW_NORMAL);
    cv::imshow(wname, im_gray);
}

void UAVLandingRegion::slidingWindowDetect(
    cv::Mat &weight, const cv::Mat image) {
    if (image.empty()) {
        ROS_ERROR("EMPTY INPUT IMAGE FOR SLIDING WINDOW DETECTION");
        return;
    }
    cv::resize(templ_img_, templ_img_, cv::Size(32, 32));
    const int step_stride = templ_img_.cols/4;
    HOGFeatureDescriptor hog;
    cv::Mat templ_desc = hog.computeHOG(this->templ_img_);

    // weight = cv::Mat::ones(image.size(), CV_32F);
    weight = image.clone();
    cv::Rect r_rect;
    double dist = 100;
#ifdef _OPENMP
#pragma omp parallel for num_threads(8)
#endif
    for (int i = 0; i < image.rows; i += step_stride) {
        for (int j = 0; j < image.cols; j += step_stride) {
            cv::Rect rect = cv::Rect(j, i, templ_img_.cols, templ_img_.rows);
            if (rect.x + rect.width < image.cols &&
                rect.y + rect.height < image.rows) {
                cv::Mat roi = image(rect).clone();
                cv::Mat desc = hog.computeHOG(roi);
                double d = cv::compareHist(
                    templ_desc, desc, CV_COMP_BHATTACHARYYA);
                
                if (isnan(d)) {
                    d = 1;
                }

#ifdef _OPENMP
#pragma omp critical 
#endif
                {
                if (d < dist && d < 0.5) {
                r_rect = rect;
                dist = d;
            }
            }
                
            }
        }
    }

    std::cout << "Dist: " << dist  << "\n";
    
    cv::rectangle(weight, r_rect, cv::Scalar(0, 255,0), 3);
    std::string wname = "dist";
    cv::namedWindow(wname, cv::WINDOW_NORMAL);
    cv::imshow(wname, weight);
    
    // std::cout <<  weight << "\n";
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "jsk_mbzirc_tasks");
    UAVLandingRegion ulr;
    ros::spin();
    return 0;
}
