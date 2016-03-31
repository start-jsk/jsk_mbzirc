
#include <jsk_mbzirc_tasks/uav_self_filter.h>

UAVSelfFilter::UAVSelfFilter() :
    thresh_(10), is_mask_(true) {
    this->onInit();
}

void UAVSelfFilter::onInit() {
    this->subscribe();
    this->pub_image_ = pnh_.advertise<sensor_msgs::Image>(
        "target", 1);
}


void UAVSelfFilter::subscribe() {
    this->sub_image_ = this->pnh_.subscribe(
        "input", 1, &UAVSelfFilter::imageCB, this);
}

void UAVSelfFilter::unsubscribe() {
    this->sub_image_.shutdown();
}

void UAVSelfFilter::imageCB(
    const sensor_msgs::Image::ConstPtr &image_msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(
            image_msg, sensor_msgs::image_encodings::BGR8);

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat image = cv_ptr->image.clone();
    if (image.empty()) {
        return;
    }

    if (is_mask_) {
        this->filter(image, this->mask_);
        cv::imwrite("uav_self_filter_mask.jpg", this->mask_);
        is_mask_ = false;
    }
    
    cv_bridge::CvImagePtr pub_msg(new cv_bridge::CvImage);
    pub_msg->header = image_msg->header;
    pub_msg->encoding = sensor_msgs::image_encodings::MONO8;
    pub_msg->image = this->mask_.clone();
    this->pub_image_.publish(pub_msg);
}

void UAVSelfFilter::filter(cv::Mat &image, cv::Mat &mask) {
    cv::Mat gray;
    cv::cvtColor(image, gray, CV_BGR2GRAY);
    mask = cv::Mat::zeros(image.size(), CV_8U);
    this->pnh_.getParam("/uav_self_filter/mask_thresh", this->thresh_);
    for (int i = 0; i < gray.rows; i++) {
        for (int j = 0; j < gray.cols; j++) {
            if (static_cast<int>(gray.at<uchar>(i, j)) < this->thresh_) {
                image.at<cv::Vec3b>(i, j)[0] = 0;
                image.at<cv::Vec3b>(i, j)[1] = 0;
                image.at<cv::Vec3b>(i, j)[2] = 0;
            } else {
                mask.at<uchar>(i, j) = 255;
            }
        }
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "jsk_mbzirc_tasks");
    UAVSelfFilter usf;
    ros::spin();
}
