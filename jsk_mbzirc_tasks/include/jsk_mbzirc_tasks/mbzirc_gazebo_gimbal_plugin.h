/*
 * Copyright (c) 2016, JSK Robotics Laboratory, The University of Tokyo
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef JSK_MBZIRC_TASKS_MBZIRC_GAZEBO_GIMBAL_PLUGIN_H
#define JSK_MBZIRC_TASKS_MBZIRC_GAZEBO_GIMBAL_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/math/Quaternion.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <cmath>
#include <string>

namespace gazebo
{
class GazeboGimbal : public ModelPlugin
{
public:
  GazeboGimbal();
  virtual ~GazeboGimbal();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();
  virtual void Reset();

private:
  struct Servo
  {
    std::string name;
    math::Vector3 axis;
    physics::JointPtr joint;
    float filter_command;
    float i_term;
    float velocity;
    float angle;
    Servo() : velocity() {}
  }
    servos_[3];  // the special style of roslint

  ros::NodeHandle* node_handle_;

  ros::Subscriber imu_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Publisher joint_state_pub_;
  tf::TransformListener* transform_listener_;

  ros::CallbackQueue callback_queue_;
  common::Time prev_update_time_;
  event::ConnectionPtr update_connection_;

  physics::WorldPtr world_;
  physics::LinkPtr link_;
  std::string link_name_;

  math::Quaternion rotation_;
  geometry_msgs::QuaternionStamped::ConstPtr current_cmd_;
  sensor_msgs::JointState joint_state_;

  ros::Time state_stamp_;
  math::Pose pose_;
  math::Vector3 euler_;

  std::string namespace_;
  std::string imu_topic_;
  std::string state_topic_;

  // parameters
  std::string robot_name_space_;
  std::string topic_name_;
  std::string joint_state_name_;
  common::Time control_period_;

  float proportional_controller_gain_;
  float derivative_controller_gain_;
  float integral_controller_gain_;
  double maximum_velocity_;
  float maximum_torque_;
  double time_constant_;

  unsigned int count_of_servos_;
  unsigned int order_of_axes_[3];

  void calculateVelocities(double dt);
  void publish_joint_states();
  float updatePID(Servo* servo, float new_input, float x, float dx, float dt);

  void imuCallback(const sensor_msgs::ImuConstPtr&);
  void stateCallback(const nav_msgs::OdometryConstPtr&);
};
}  // namespace gazebo

#endif  // JSK_MBZIRC_TASKS_MBZIRC_GAZEBO_GIMBAL_PLUGIN_H
