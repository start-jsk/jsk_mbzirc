/*
 * netusb_camera_nodelet.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <netusb_camera_driver/netusb_camera_nodelet.h>

namespace netusb_camera_driver
{
  void ConvertBGR2BRG(cv::Mat &mat) {
    for(int y = 0; y < mat.rows; ++y) {
      for (int x = 0; x < mat.cols; ++x) {
        uint8_t b = mat.data[y * mat.step + x * mat.elemSize() + 0];
        uint8_t g = mat.data[y * mat.step + x * mat.elemSize() + 1];
        uint8_t r = mat.data[y * mat.step + x * mat.elemSize() + 2];
        mat.data[y * mat.step + x * mat.elemSize() + 0] = r;
        mat.data[y * mat.step + x * mat.elemSize() + 1] = b;
        mat.data[y * mat.step + x * mat.elemSize() + 2] = g;
      }
    }
  }

  NETUSBCameraNodelet::NETUSBCameraNodelet() {}

  NETUSBCameraNodelet::~NETUSBCameraNodelet()
  {
    if (pub_thread_)
    {
      NODELET_INFO("interrupting polling thread");
      pub_thread_->interrupt();
      pub_thread_->join();
    }

    try
    {
      NODELET_INFO("stopping netusb camera");
      cam_.stop();
      NODELET_INFO("disconnecting netusb camera");
      cam_.disconnect();
    }
    catch (std::runtime_error &err)
    {
      NODELET_ERROR("%s", err.what());
    }
  }

  void NETUSBCameraNodelet::configCallback(Config &config, const uint32_t level)
  {
    NODELET_INFO("configCallback called");
    if (!cam_.isConnected()) cam_.connect();
    boost::mutex::scoped_lock slock(cfg_mutex_);
    if (level != NETUSBCamera::RECONFIGURE_RUNNING) {
      try {
        bool wasRunning = !cam_.isStopped();
        if (wasRunning) cam_.stop();
        cam_.setVideoMode((NETUSBCamera::Mode)config.video_mode);
        if (wasRunning) {
          while (cam_.isStopped() && ros::ok()) {
            try {
              cam_.start();
            } catch (std::runtime_error &se) {
              NODELET_ERROR("failed to start camera: %s", se.what());
              ros::Duration(1.0).sleep();
            }
          }
        }
      } catch (std::runtime_error &e) {
        NODELET_ERROR("failed to set config: %s", e.what());
      }
    } else {
      try {
        cam_.setParameter(NETUSBCamera::BRIGHTNESS, config.brightness);
        cam_.setParameter(NETUSBCamera::CONTRAST, config.contrast);
        cam_.setParameter(NETUSBCamera::GAMMA, config.gamma);
        cam_.setParameter(NETUSBCamera::BLACKLEVEL, config.blacklevel);
        cam_.setExposure(config.exposure_time);
        cam_.setParameter(NETUSBCamera::EXPOSURE_TARGET, config.exposure_target);
        cam_.setParameter(NETUSBCamera::GAIN, config.gain);
        cam_.setParameter(NETUSBCamera::PLL, config.pll);
        cam_.setParameter(NETUSBCamera::RED, config.red);
        cam_.setParameter(NETUSBCamera::GREEN, config.green);
        cam_.setParameter(NETUSBCamera::BLUE, config.blue);
        cam_.setParameter(NETUSBCamera::MEASURE_FIELD_AE, config.measure_field_ae);
        cam_.setParameter(NETUSBCamera::SHUTTER, config.shutter);
        cam_.setBoolParameter(NETUSBCamera::FLIPPED_V, config.flipped_v);
        cam_.setBoolParameter(NETUSBCamera::FLIPPED_H, config.flipped_h);
        if (config.white_balance) cam_.setWhiteBalance();
        if (config.reset) resetConfig();
      } catch (std::runtime_error &e) {
        NODELET_ERROR("failed to set config: %s", e.what());
      }
    }

    try {
      getConfig(config, level);
    } catch (std::runtime_error &e) {
      NODELET_ERROR("failed to sync config: %s", e.what());
    }
  }

  void NETUSBCameraNodelet::resetConfig()
  {
    cam_.resetParameter(NETUSBCamera::BRIGHTNESS);
    cam_.resetParameter(NETUSBCamera::CONTRAST);
    cam_.resetParameter(NETUSBCamera::GAMMA);
    cam_.resetParameter(NETUSBCamera::BLACKLEVEL);
    cam_.resetExposure();
    cam_.resetParameter(NETUSBCamera::EXPOSURE_TARGET);
    cam_.resetParameter(NETUSBCamera::GAIN);
    cam_.resetParameter(NETUSBCamera::PLL);
    cam_.resetParameter(NETUSBCamera::RED);
    cam_.resetParameter(NETUSBCamera::GREEN);
    cam_.resetParameter(NETUSBCamera::BLUE);
    cam_.resetParameter(NETUSBCamera::MEASURE_FIELD_AE);
    cam_.resetParameter(NETUSBCamera::SHUTTER);
    cam_.resetParameter(NETUSBCamera::FLIPPED_V);
    cam_.resetParameter(NETUSBCamera::FLIPPED_H);
  }

  void NETUSBCameraNodelet::getConfig(Config &config, const uint32_t &level)
  {
    if(level == NETUSBCamera::RECONFIGURE_RUNNING) {
      config.brightness = cam_.getParameter(NETUSBCamera::BRIGHTNESS);
      config.contrast = cam_.getParameter(NETUSBCamera::CONTRAST);
      config.gamma = cam_.getParameter(NETUSBCamera::GAMMA);
      config.blacklevel = cam_.getParameter(NETUSBCamera::BLACKLEVEL);
      config.exposure_time = cam_.getExposure();
      config.exposure_target = cam_.getParameter(NETUSBCamera::EXPOSURE_TARGET);
      config.gain = cam_.getParameter(NETUSBCamera::GAIN);
      config.pll = cam_.getParameter(NETUSBCamera::PLL);
      config.red = cam_.getParameter(NETUSBCamera::RED);
      config.green = cam_.getParameter(NETUSBCamera::GREEN);
      config.blue = cam_.getParameter(NETUSBCamera::BLUE);
      config.measure_field_ae = cam_.getParameter(NETUSBCamera::MEASURE_FIELD_AE);
      config.shutter = cam_.getParameter(NETUSBCamera::SHUTTER);
      config.flipped_v = cam_.getBoolParameter(NETUSBCamera::FLIPPED_V);
      config.flipped_h = cam_.getBoolParameter(NETUSBCamera::FLIPPED_H);
      config.white_balance = false;
      config.reset = false;
    }
    config.video_mode = cam_.getVideoMode();
  }

  void NETUSBCameraNodelet::connectCallback()
  {
    NODELET_INFO("connectCallback");
    boost::mutex::scoped_lock slock(conn_mutex_);
    if (pub_.getNumSubscribers() > 0 && cam_.isStopped()) {
      while (!cam_.isConnected() && ros::ok()) {
        try {
          NODELET_INFO("Connecting to netusb camera");
          cam_.connect();
        } catch (std::runtime_error &e) {
          NODELET_ERROR("failed to connect: %s", e.what());
          ros::Duration(1.0).sleep();
        }
      }
      while (cam_.isStopped() && ros::ok()) {
        try {
          NODELET_INFO("Starting netusb camera");
          image_width_ = cam_.getWidth();
          image_height_ = cam_.getHeight();
          cam_.start();
        } catch (std::runtime_error &e) {
          NODELET_ERROR("failed to start: %s", e.what());
          ros::Duration(1.0).sleep();
        }
      }
      pub_thread_.reset(new boost::thread(
                          boost::bind(&NETUSBCameraNodelet::imagePoll, this)));
    } else {
      NODELET_INFO("current subscriber: %d -> %d",
                   pub_.getNumSubscribers()-1, pub_.getNumSubscribers());
    }
  }

  void NETUSBCameraNodelet::disconnectCallback()
  {
    boost::mutex::scoped_lock slock(conn_mutex_);
    if (pub_.getNumSubscribers() == 0)
    {
      try
      {
        NODELET_INFO("Stopping netusb camera");
        pub_thread_->interrupt();
        pub_thread_->join();
        cam_.stop();
      }
      catch (std::runtime_error &e)
      {
        NODELET_ERROR("failed to stop: %s", e.what());
        ros::Duration(1.0).sleep();
      }
    } else {
      NODELET_INFO("current subscriber: %d -> %d",
                   pub_.getNumSubscribers()+1, pub_.getNumSubscribers());
    }
  }

  void NETUSBCameraNodelet::imagePoll()
  {
    NODELET_INFO("image polling thread started");
    uint32_t img_seq = 0;
    ros::Time prev_time = ros::Time::now();
    uint32_t fps_count = 0;
    while(!boost::this_thread::interruption_requested())
    {
      if(pub_.getNumSubscribers() == 0)
        continue;
      try
      {
        sensor_msgs::ImagePtr imgmsg(new sensor_msgs::Image);
        bool newImage = cam_.getImage(imgmsg->data);
        if (!newImage) continue;

        imgmsg->header.seq = img_seq++;
        imgmsg->header.frame_id = frame_id_;
        imgmsg->header.stamp = ros::Time::now(); // FIXME: use device time
        imgmsg->encoding = image_encoding_;
        imgmsg->height = image_height_;
        imgmsg->width = image_width_;
        imgmsg->step = imgmsg->data.size() / image_height_;
        imgmsg->is_bigendian = 0;

        // camera info
        cam_info_.reset(new sensor_msgs::CameraInfo(cim_->getCameraInfo()));
        cam_info_->header.seq = imgmsg->header.seq;
        cam_info_->header.stamp = imgmsg->header.stamp;
        cam_info_->header.frame_id = imgmsg->header.frame_id;

        // convert to bgr image
        cv::Mat cvimg(imgmsg->data, false);
        ConvertBGR2BRG(cvimg);

        // FIXME: ROI

        pub_.publish(imgmsg, cam_info_);

        // fps
        ++fps_count;
        ros::Time now = ros::Time::now();
        if ((now - prev_time).toSec() > 1.0) {
          ROS_INFO_STREAM("publishing " << cam_.getName() << " camera image (" << fps_count << " fps.)");
          prev_time = now;
          fps_count = 0;
        }
      }
      catch (CameraTimeoutException &err)
      {
        NODELET_WARN("%s", err.what());
      }
      catch (std::runtime_error &err)
      {
        NODELET_ERROR("%s", err.what());
      }
    }
    NODELET_INFO("image polling thread ended");
  }

  void NETUSBCameraNodelet::onInit()
  {
    // prevent calling connectCallback before onInit
    boost::mutex::scoped_lock slock(conn_mutex_);

    nh_ = getMTNodeHandle();
    pnh_ = getMTPrivateNodeHandle();

    pnh_.param<int>("device_number", cam_index_, 0);
    pnh_.param<double>("conenction_timeout", conn_timeout_, 0.0);
    pnh_.param<std::string>("frame_id", frame_id_, "camera");

    image_encoding_ = sensor_msgs::image_encodings::BAYER_GRBG8;

    // setup camera info manager
    std::string cam_info_url;
    pnh_.param<std::string>("camera_info_url", cam_info_url, "");
    std::string cam_name = cam_.getName();
    cim_.reset(new camera_info_manager::CameraInfoManager(nh_, cam_name, cam_info_url));

    // setup dynamic reconfigure server
    srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> > (pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&NETUSBCameraNodelet::configCallback, this, _1, _2);
    srv_->setCallback(f);

    // setup publishers
    it_.reset(new image_transport::ImageTransport(nh_));
    image_transport::SubscriberStatusCallback it_conn_cb =
      boost::bind(&NETUSBCameraNodelet::connectCallback, this);
    image_transport::SubscriberStatusCallback it_disconn_cb =
      boost::bind(&NETUSBCameraNodelet::disconnectCallback, this);
    pub_ = it_->advertiseCamera("image_raw", 5, it_conn_cb, it_disconn_cb);
  }
} // namespace netusb_camera_driver

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(netusb_camera_driver::NETUSBCameraNodelet, nodelet::Nodelet);
