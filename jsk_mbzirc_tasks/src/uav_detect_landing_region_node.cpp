
#include <jsk_mbzirc_tasks/uav_detect_landing_region.h>

UAVLandingRegion::UAVLandingRegion() {
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
    
    this->detect(image);
    

    // cv::resize(image, image, cv::Size(64, 128));
    HOGFeatureDescriptor hog;
    cv::Mat desc = hog.computeHOG(image);
    cv::Mat hog_viz = hog.visualizeHOG(image, desc, image.size(), 2, 5);    

    // std::cout << desc  << "\n";

    std::cout << "Desc: " << desc.size()  << "\n";
    
    
    // TODO: 
    // cv::Mat templ_img = this->createLandingTemplate(image.size());
    cv::namedWindow("template", cv::WINDOW_NORMAL);
    cv::imshow("template", hog_viz);
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
    cv::Canny(im_gray, im_gray, 10, 50, 3);

    std::string wname = "edges";
    cv::namedWindow(wname, cv::WINDOW_NORMAL);
    cv::imshow(wname, im_gray);
}

cv::Mat UAVLandingRegion::createLandingTemplate(const cv::Size size) {
    if (size.height < 5 || size.width < 5) {
        ROS_ERROR("VERY SMALL TEMPLATE");
        return cv::Mat();
    }
    cv::Mat templ = cv::Mat::zeros(size, CV_8U);
    cv::Point center = cv::Point(size.width/2, size.height/2);
    float radius = (std::min(size.width, size.height)/2) - 50;
    cv::circle(templ, center, radius, cv::Scalar(255), 5);
    return templ;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "jsk_mbzirc_tasks");
    UAVLandingRegion ulr;
    ros::spin();
    return 0;
}
