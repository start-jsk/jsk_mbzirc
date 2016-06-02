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

#include <jsk_mbzirc_common/mbzirc_gazebo_drone_additional_plugin.h>

namespace gazebo {

GazeboDrone::GazeboDrone()
{
}

GazeboDrone::~GazeboDrone()
{
  event::Events::DisconnectWorldUpdateBegin(update_connection_);

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboDrone::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gzwarn << "The GazeboDrone plugin is DEPRECATED in ROS hydro." << std::endl;

  model_ = _model;
  world_ = _model->GetWorld();
  link_ = _model->GetLink();
  link_name_ = link_->GetName();
  namespace_.clear();

  landing_height_ = -0.1; //init dummy height
  takeoff_flag_ = false;
  terminated_flag_ = false;


  // load parameters from sdf
  if (_sdf->HasElement("robotNamespace")) namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();

  if (_sdf->HasElement("bodyName") && _sdf->GetElement("bodyName")->GetValue()) {
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
    link_ = _model->GetLink(link_name_);
  }

  if (!link)
  {
    ROS_FATAL("gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }


  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  pub_ee_z_ = node_handle_->advertise<std_msgs::String>("drone_estimated_height", 1);
  pub_gt_z_ = node_handle_->advertise<std_msgs::String>("drone_groundtruth_height", 1);
  sub_drone_state_ = node_handle_->subscribe<nav_msgs::Odometry>("/state", 1, &GazeboDrone::DroneStateCallback, this, ros::TransportHints().tcpNoDelay()); 
  motor_client_ = node_handle_->serviceClient<std_srvs::Empty>("/shutdown");
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
                                                              boost::bind(&GazeboDrone::Update, this));
}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboDrone::Update()
{
  boost::mutex::scoped_lock scoped_lock(lock);

  if(terminated_flag_) return;


  if(!takeoff_flag_)
    {//TODO: check contact with model "arena" is better
      if(landing_height_ < 0) landing_height_  = model_->GetWorldPose().pos.z;
      if(model_->GetWorldPose().pos.z > landing_height_ + 0.05) //0.05m is a margin
        takeoff_flag_ = true;
    }
  else
    {//TODO: check contact with model "arena" is better
      std::stringstream ss;
      std_msgs::String msg_time;
      ss << model_->GetWorldPose().pos.z - landing_height_;
      msg_time.data = "height(ground truth):" + ss.str();
      pub_gt_z_.publish(msg_time);


      if(model_->GetWorldPose().pos.z < landing_height_ + 0.05) 
        {
          ROS_WARN("Hit the ground");
          std_srvs::Empty srv;

          if (motor_client_.call(srv))
            {
              terminated_flag_ = true;
            }
          else
            ROS_ERROR("Failed to call service motor_shutdown");
        }
    }

}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboDrone::Reset()
{
  state_stamp_ = ros::Time();
}

/////////////////////////////////////////////////////////////////////////
/// Callback func for drone state
void GazeboDrone::DroneStateCallback(const nav_msgs::OdometryConstPtr& state_msg)
{
  std::stringstream ss;
  std_msgs::String msg_time;
  double z = (state_msg->pose.pose.position.z >= 0)? state_msg->pose.pose.position.z: 0;
  ss << z;
  msg_time.data = "height(estimation):" + ss.str();
  pub_ee_z_.publish(msg_time);
}


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboDrone)

} // namespace gazebo
