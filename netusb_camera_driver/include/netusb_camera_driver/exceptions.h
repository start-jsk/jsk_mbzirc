/*
 * exceptions.h
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef EXCEPTIONS_H__
#define EXCEPTIONS_H__

#include <stdexcept>

class CameraTimeoutException: public std::runtime_error
{
public:
  CameraTimeoutException(): runtime_error("Dropping Camera Frame") {}
  CameraTimeoutException(std::string msg): runtime_error(msg.c_str()) {}
};

class CameraNotRunningException: public std::runtime_error
{
public:
  CameraNotRunningException(): runtime_error("Camera is not running. Please start the capture.") {}
  CameraNotRunningException(std::string msg): runtime_error(msg.c_str()) {}
};

class CameraNotFoundException: public std::runtime_error
{
public:
  CameraNotFoundException(): runtime_error("No Camera Found.") {}
  CameraNotFoundException(std::string msg): runtime_error(msg.c_str()) {}
};

class InvalidParameterException: public std::runtime_error
{
public:
  InvalidParameterException(): runtime_error("Invalid Parameter") {}
  InvalidParameterException(std::string msg): runtime_error(msg.c_str()) {}
};

#endif // EXCEPTIONS_H__
