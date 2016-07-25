/*
 * NETUSBCamera.h
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef NETUSBCAMERA_H__
#define NETUSBCAMERA_H__

#include <NETUSBCAM_API.h>

#include <boost/thread.hpp>
#include <string>
#include <ros/ros.h> // just for logging
#include <netusb_camera_driver/exceptions.h>

namespace netusb_camera_driver {

class NETUSBCamera
{
public:

  enum Level {
    RECONFIGURE_RUNNING = 0,
    RECONFIGURE_STOPPED = 1
  };

  enum Mode {
    QVGA = 0,
    VGA = 1,
    WVGA = 2,
    SVGA = 3,
    XGA = 4,
    SXGA = 5,
    UXGA = 6,
    QXGA = 7,
    QSXGA = 8,
    WQUXGA = 9
  };

  enum Result {
    SUCCESS = 0,
    ERROR = 1,
    IF_NOT_OPEN = -1,
    WRONG_PARAM = -2,
    OUT_OF_MEMORY = -3,
    ALREADY_DONE = -4,
    WRONG_CLOCK_VAL = -5,
    COM_LIB_INIT = -6,
    NOT_IF_STARTED = -7
  };

  enum Toggle {
    ON = 0,
    OFF = 1
  };

  enum ParameterType {
    BRIGHTNESS = 1, // all models
    CONTRAST = 2, // all models
    GAMMA = 3, // all models
    FLIPPED_V = 4, // (ON||OFF); OFF (default)
    FLIPPED_H = 5, // (ON||OFF); OFF (default)
    WHITE_BALANCE = 6, // (one push)
    EXPOSURE_TARGET = 8, // sets the target-value for the auto exposure algorithm 
    RED = 9, // only for color models; RGB Gain value
    GREEN = 10, // only for color models; RGB Gain value
    BLUE = 11, // only for color models; RGB Gain value
    BLACKLEVEL = 12, // sensor blacklevel
    GAIN = 13, // sensor gain
    COLOR = 117,
    PLL = 15, // all models
    STROBE_LENGTH = 16, // length of strobe pulse output (msec)
    STROBE_DELAY = 17, // delay before strobe pulse is executed (msec)
    TRIGGER_DELAY = 18, // delay before hardware trigger is executed (msec)

    MEASURE_FIELD_AE = 22, // measure field for auto exposure
    SHUTTER = 26, // (4133CU/BU,4133CU)
    ROI_ID = 27, // (4133CU/BU,4133CU,fpga board cameras)
    ROI_CYCLE = 28, // (4133CU/BU,4133CU) number of repetitions of the particular roi

    SENSOR_TIMING = 123 // (4133CU/BU,4133CU)
  };

  NETUSBCamera();
  virtual ~NETUSBCamera();

  inline std::string getName() const {
    return camera_name_;
  };

  void connect();
  void disconnect();

  void start();
  void stop();

  inline bool isStopped() const {
    return is_stopped_;
  };
  inline bool isConnected() const {
    return is_connected_;
  }

  bool getImage(std::vector<uint8_t> &buffer);

  int  getWidth() const {
    return image_width_;
  }
  int getHeight() const {
    return image_height_;
  };

  void setVideoMode(const Mode &mode);
  Mode getVideoMode() const;

  void setBoolParameter(const ParameterType &type, const bool &value);
  void setParameter(const ParameterType &type, const int &value);
  void resetParameter(const ParameterType &type);
  bool getBoolParameter(const ParameterType &type) const;
  int getParameter(const ParameterType &type) const;

  void  setExposure(const double &value);
  double getExposure() const;
  void resetExposure() const;

  void setWhiteBalance();

  void  ImageCallback(void *buffer, unsigned int buffersize);

  void checkResult(const int &result, const std::string &message) const;

private:
  bool is_stopped_;
  bool is_connected_;
  int  camera_index_;
  int  camera_num_;
  std::string camera_name_;
  std::string camera_serial_;
  std::vector<Mode> available_modes_;
  uint8_t* latest_buffer_;
  unsigned int latest_buffersize_;
  int   image_width_,  image_height_;
  boost::mutex mutex_;
};
};

#endif // NETUSBCAMERA_H__
