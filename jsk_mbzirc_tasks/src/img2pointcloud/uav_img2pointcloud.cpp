/*
 jsk_mbzirc_task
 */

// Author: Chen

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/tf.h>
//msg headers.
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_mbzirc_msgs/ProjectionMatrix.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <iostream>
//for test
#include <tf/transform_broadcaster.h>
//pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <ctime>


//cuda
float  process_in_cuda(double *_a, double *_b, double *_c, cv::Mat *dev_img,
                                                   pcl::PointCloud<pcl::PointXYZRGB> *PC);

class uav_img2pointcloud
{
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo,nav_msgs::Odometry> MySyncPolicy;

private:
    ros::Publisher pointcloud_pub_;
    ros::Publisher param_matrix_pub_;
    sensor_msgs::PointCloud2 cloud_msg;
    message_filters::Subscriber<sensor_msgs::Image> *img_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *camera_info_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> *uav_odom_sub_;
    ros::NodeHandle nh_;
    message_filters::Synchronizer<MySyncPolicy> *sync;
    tf::Transform BaseToCamera;
#define Ground_Z 0.0
   //test
    tf::TransformBroadcaster br;
public:
    void init()
    {
        //publish pointcloud msgs:
        std::string topic = nh_.resolveName("imagetoground");
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(topic, 1);
        std::string topic2 = nh_.resolveName("projection_matrix");
        param_matrix_pub_ = nh_.advertise<jsk_mbzirc_msgs::ProjectionMatrix>(topic2,1);
        //for test the image
        cv::namedWindow("view");
        cv::startWindowThread();
        img_sub_  = new message_filters::Subscriber<sensor_msgs::Image>(nh_,"/downward_cam/camera/image",10);
        camera_info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_,"/downward_cam/camera/camera_info", 10);
        uav_odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,"/ground_truth/state",10);
        sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(3), *img_sub_, *camera_info_sub_, *uav_odom_sub_);
        sync->registerCallback(boost::bind(&uav_img2pointcloud::imageCallback,this,_1,_2,_3));
        /*
        if(!nh_.getParam("enableGPU",GPUFLAG))
            std::cout<<"fail to load the param enableGPU, Using CPU instead"<<std::endl;
        else
            std::cout<<"With GPU support flag = " << GPUFLAG<<std::endl;
            */
        //initialize base_link to camera optical link
        BaseToCamera.setOrigin(tf::Vector3(0.0,0.0,-0.2));
        BaseToCamera.setRotation(tf::Quaternion(0.707, -0.707, 0.000, -0.000));
    }
    //call back, for processing
    void imageCallback(const sensor_msgs::ImageConstPtr& img,
                       const sensor_msgs::CameraInfoConstPtr& cam_info,
                       const nav_msgs::OdometryConstPtr& odom);
    //process to pointcloud
    void p2p(const sensor_msgs::ImageConstPtr& img,
             const sensor_msgs::CameraInfoConstPtr& cam_info,
             const nav_msgs::OdometryConstPtr& odom);
    // class's family...
    ~uav_img2pointcloud()
    {    }
};

void uav_img2pointcloud::p2p(const sensor_msgs::ImageConstPtr& img,
                             const sensor_msgs::CameraInfoConstPtr& cam_info,
                             const nav_msgs::OdometryConstPtr& odom)
{
    tf::Transform extrisic;
    cv::Mat P(3,4,CV_64FC1);
    cv::Mat P_Mat_G(3,4,CV_64FC1);
    tf::Pose tfpose;
    tfScalar extrisic_data[4*4];
    pcl::PointCloud<pcl::PointXYZRGB> Pointcloud;
    jsk_mbzirc_msgs::ProjectionMatrix param_vector;
    std_msgs::MultiArrayDimension dim;
    dim.size = 3;dim.label = "height";
    param_vector.layout.dim.push_back(dim);
    dim.size = 4;dim.label = "width";
    param_vector.layout.dim.push_back(dim);
    Pointcloud.header.frame_id = "/world";
    Pointcloud.height = img->height; Pointcloud.width = img->width;
    Pointcloud.resize(img->height*img->width);
    Pointcloud.is_dense = true;
    cv::Mat cvimg = cv_bridge::toCvShare(img,"bgr8")->image.clone();
    tf::poseMsgToTF(odom->pose.pose,tfpose);
    extrisic = BaseToCamera*tfpose.inverse();
    //to test if the tf is correct, create testframe_to_camera
    //br.sendTransform(tf::StampedTransform(extrisic, ros::Time::now(), "/testframe_to_camera", "/world"));
    //pinv of projection matrix...
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 4; j++)
            P.at<double>(i,j) = cam_info->P.at(i*4+j);
    //however, this P is in camera coordinate..
    extrisic.getOpenGLMatrix(extrisic_data);
    cv::Mat E_MAT(4,4,CV_64FC1,extrisic_data);
    P_Mat_G = P*(E_MAT.t());
    // now is the ground, namely, world coordinate
    double a[4],b[4],c[4];
    a[0] = P_Mat_G.at<double>(0,0); a[1] = P_Mat_G.at<double>(0,1); a[2] = P_Mat_G.at<double>(0,2); a[3] = P_Mat_G.at<double>(0,3);
    b[0] = P_Mat_G.at<double>(1,0); b[1] = P_Mat_G.at<double>(1,1); b[2] = P_Mat_G.at<double>(1,2); b[3] = P_Mat_G.at<double>(1,3);
    c[0] = P_Mat_G.at<double>(2,0); c[1] = P_Mat_G.at<double>(2,1); c[2] = P_Mat_G.at<double>(2,2); c[3] = P_Mat_G.at<double>(2,3);
    std::clock_t start;
    double duration;
    start = std::clock();

    //gpu
#if defined(GPU_EN)
    process_in_cuda(a, b, c, &cvimg, &Pointcloud);
#else
    //cpu
    for(int i=0;i<Pointcloud.height;i++)
        for(int j=0;j<Pointcloud.width;j++)
        {
            /*  using matrix (so slow!!!!!!!!!!!!)
            cv::Mat A(2,2,CV_64FC1),A_inv(2,2,CV_64FC1),bv(2,1,CV_64FC1);
            A.at<double>(0,0) = j*c[0] - a[0]; A.at<double>(0,1) = j*c[1] - a[1];
            A.at<double>(1,0) = i*c[0] - b[0]; A.at<double>(1,1) = i*c[1] - b[1];
            bv.at<double>(0,0) = a[2]*Ground_Z + a[3] - j*c[2]*Ground_Z - j*c[3];
            bv.at<double>(1,0) = b[2]*Ground_Z + b[3] - i*c[2]*Ground_Z - i*c[3];
            cv::invert(A,A_inv,cv::DECOMP_LU);
            cv::Mat Point = A_inv*bv;
            Pointcloud.points[i*Pointcloud.width+j].x = (float)Point.at<double>(0,0);
            Pointcloud.points[i*Pointcloud.width+j].y = (float)Point.at<double>(1,0);
            Pointcloud.points[i*Pointcloud.width+j].z = (float)Ground_Z;
            */
            //directly calculation make it faster, next step is to parallize it
            float A[2][2],bv[2];
            A[0][0] = j*c[0] - a[0]; A[0][1] = j*c[1] - a[1];
            A[1][0] = i*c[0] - b[0]; A[1][1] = i*c[1] - b[1];
            bv[0]= a[2]*Ground_Z + a[3] - j*c[2]*Ground_Z - j*c[3];
            bv[1] = b[2]*Ground_Z + b[3] - i*c[2]*Ground_Z - i*c[3];
            float DomA = A[1][1]*A[0][0]-A[0][1]*A[1][0];
            Pointcloud.points[i*Pointcloud.width+j].x = (A[1][1]*bv[0]-A[0][1]*bv[1])/DomA;
            Pointcloud.points[i*Pointcloud.width+j].y = (A[0][0]*bv[1]-A[1][0]*bv[0])/DomA;
            Pointcloud.points[i*Pointcloud.width+j].z = (float)Ground_Z;

            //fill the color info
            uint8_t rgb[4];
            rgb[0] = cvimg.at<cv::Vec3b>(i,j)[0];
            rgb[1] = cvimg.at<cv::Vec3b>(i,j)[1];
            rgb[2] = cvimg.at<cv::Vec3b>(i,j)[2];
            rgb[3] = 0;
            Pointcloud.points[i*Pointcloud.width+j].rgb = *(float *)(rgb);
        }
#endif
    //get the duration....
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"process_time is "<< duration << " second" <<'\n';
    //publish pointcloud
    pcl::toROSMsg(Pointcloud,cloud_msg);
    pointcloud_pub_.publish(cloud_msg);
    //publish param matrix 4*3 = 12
    param_vector.header = img->header;
    for(int i = 0; i < 4; i++)
        param_vector.data.push_back(a[i]);
    for(int i = 0; i < 4; i++)
        param_vector.data.push_back(b[i]);
    for(int i = 0; i < 4; i++)
        param_vector.data.push_back(c[i]);

    param_matrix_pub_.publish(param_vector);

}



void uav_img2pointcloud::imageCallback(const sensor_msgs::ImageConstPtr& img,
                                       const sensor_msgs::CameraInfoConstPtr& cam_info,
                                       const nav_msgs::OdometryConstPtr& odom)
{
    try
    {
        cv::imshow("view", cv_bridge::toCvShare(img,"bgr8")->image);
        //process to pointcloud
        p2p(img,cam_info,odom);
        cv::waitKey(10);
    }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_to_pointcloud");
    uav_img2pointcloud u_i2p;
    u_i2p.init();

    ros::spin();
    cv::destroyWindow("view");
}
