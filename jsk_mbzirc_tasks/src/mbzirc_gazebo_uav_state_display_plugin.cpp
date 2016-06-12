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

#include <jsk_mbzirc_tasks/mbzirc_gazebo_uav_state_display_plugin.h>
#include <string>

namespace gazebo
{

GazeboUavState::GazeboUavState()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboUavState::~GazeboUavState()
{
  event::Events::DisconnectWorldUpdateBegin(update_connection_);
  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboUavState::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world_ = _model->GetWorld();
  link_ = _model->GetLink();
  link_name_ = link_->GetName();

  // default parameters
  robot_name_space_.clear();
  state_topic_.clear();
  ground_truth_topic_.clear();

  // load parameters from sdf
  if (_sdf->HasElement("robotNamespace"))
    robot_name_space_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  if (_sdf->HasElement("stateTopic"))
    state_topic_ = _sdf->GetElement("stateTopic")->Get<std::string>();
  if (_sdf->HasElement("groundTruthTopic"))
    ground_truth_topic_ = _sdf->GetElement("groundTruthTopic")->Get<std::string>();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  }

  node_handle_ = new ros::NodeHandle(robot_name_space_);

  if (!state_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<nav_msgs::Odometry>(
      state_topic_, 1,
      boost::bind(&GazeboUavState::stateCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    state_sub_ = node_handle_->subscribe(ops);
  }

  if (!ground_truth_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<nav_msgs::Odometry>(
      ground_truth_topic_, 1,
      boost::bind(&GazeboUavState::groundTruthCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    ground_truth_sub_ = node_handle_->subscribe(ops);
  }

  string_state_pub_ = node_handle_->advertise<std_msgs::String>("string_state", 1);
  string_ground_truth_pub_ = node_handle_->advertise<std_msgs::String>("string_ground_truth", 1);

  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboUavState::Update, this));
}

////////////////////////////////////////////////////////////////////////////////
// Callbacks
void GazeboUavState::stateCallback(const nav_msgs::OdometryConstPtr& state)
{
  std::stringstream ss;
  std_msgs::String msg_state;
  /* right now, just pub height(z) */
  ss << state->pose.pose.position.z;
  msg_state.data = "[" + robot_name_space_ + "state] z: " + ss.str() + "[m]";
  string_state_pub_.publish(msg_state);
}

void GazeboUavState::groundTruthCallback(const nav_msgs::OdometryConstPtr& state)
{
  std::stringstream ss;
  std_msgs::String msg_state;
  /* right now, just pub height(z) */
  ss << state->pose.pose.position.z;
  msg_state.data = "[" + robot_name_space_ + "ground_truth] z: " + ss.str() + "[m]";
  string_ground_truth_pub_.publish(msg_state);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboUavState::Update()
{
  // handle callbacks
  callback_queue_.callAvailable();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboUavState)

}  // namespace gazebo

