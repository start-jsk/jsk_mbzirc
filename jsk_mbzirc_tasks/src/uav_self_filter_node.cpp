
#include <jsk_mbzirc_tasks/uav_self_filter.h>


UAVSelfFilter::UAVSelfFilter() {
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
    
    cv_bridge::CvImagePtr pub_msg(new cv_bridge::CvImage);
    pub_msg->header = image_msg->header;
    pub_msg->encoding = sensor_msgs::image_encodings::BGR8;
    pub_msg->image = image.clone();
    this->pub_image_.publish(pub_msg);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "jsk_mbzirc_tasks");
    UAVSelfFilter usf;
    ros::spin();
}
