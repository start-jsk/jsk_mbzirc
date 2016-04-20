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

#ifndef MBZIRC_GAZEBO_DRONE_PLUGIN_H
#define MBZIRC_GAZEBO_DRONE_PLUGIN_H

#include <boost/bind.hpp>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/RayShape.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/common.hh>

#include <stdio.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>


namespace gazebo
{

class GazeboDrone : public ModelPlugin
{
public:
  GazeboDrone();
  virtual ~GazeboDrone();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();
  virtual void Reset();

private:

  physics::WorldPtr world_;
  physics::LinkPtr link_;
  physics::ModelPtr model_;

  std::string link_name_;
  std::string namespace_;

  ros::NodeHandle* node_handle_;
  ros::Publisher  pub_gt_z_, pub_ee_z_; 
  ros::Subscriber sub_drone_state_;
  ros::ServiceClient motor_client_;

  ros::Time state_stamp_;

  boost::mutex lock;

  float landing_height_;
  bool takeoff_flag_;
bool terminated_flag_;

  event::ConnectionPtr update_connection_;


  void DroneStateCallback(const nav_msgs::OdometryConstPtr& state_msg);

};

}

#endif
