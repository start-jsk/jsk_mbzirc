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

#include <jsk_mbzirc_tasks/mbzirc_gazebo_gimbal_plugin.h>
#include <string>

#if (GAZEBO_MAJOR_VERSION > 1) || (GAZEBO_MINOR_VERSION >= 2)
  #define RADIAN Radian
#else
  #define RADIAN GetAsRadian
#endif

namespace gazebo
{

enum
{
  FIRST = 0, SECOND = 1, THIRD = 2
};

enum
{
  xyz, zyx
};


GazeboGimbal::GazeboGimbal()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboGimbal::~GazeboGimbal()
{
  event::Events::DisconnectWorldUpdateBegin(update_connection_);
  delete transform_listener_;
  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboGimbal::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world_ = _model->GetWorld();
  link_ = _model->GetLink();
  link_name_ = link_->GetName();

  // default parameters
  robot_name_space_.clear();
  imu_topic_.clear();
  state_topic_.clear();
  joint_state_name_ = "joint_states";
  proportional_controller_gain_ = 8.0;
  derivative_controller_gain_ = 1.0;
  integral_controller_gain_ = 0.1;
  maximum_velocity_ = 0.0;
  maximum_torque_ = 5;  // test
  count_of_servos_ = 0;
  time_constant_ = 0;


  // load parameters from sdf
  if (_sdf->HasElement("robotNamespace"))
    robot_name_space_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  if (_sdf->HasElement("imuTopic"))
    imu_topic_ = _sdf->GetElement("imuTopic")->Get<std::string>();
  if (_sdf->HasElement("stateTopic"))
    state_topic_ = _sdf->GetElement("stateTopic")->Get<std::string>();

  if (_sdf->HasElement("jointStateName"))
    joint_state_name_ = _sdf->Get<std::string>("jointStateName");
  if (_sdf->HasElement("firstServoName"))
    servos_[FIRST].name = _sdf->Get<std::string>("firstServoName");
  if (_sdf->HasElement("firstServoAxis"))
    servos_[FIRST].axis = _sdf->Get<math::Vector3>("firstServoAxis");
  if (_sdf->HasElement("secondServoName"))
    servos_[SECOND].name = _sdf->Get<std::string>("secondServoName");
  if (_sdf->HasElement("secondServoAxis"))
    servos_[SECOND].axis = _sdf->Get<math::Vector3>("secondServoAxis");
  if (_sdf->HasElement("thirdServoName"))
    servos_[THIRD].name = _sdf->Get<std::string>("thirdServoName");
  if (_sdf->HasElement("thirdServoAxis"))
    servos_[THIRD].axis = _sdf->Get<math::Vector3>("thirdServoAxis");
  if (_sdf->HasElement("proportionalControllerGain"))
    proportional_controller_gain_ = _sdf->Get<double>("proportionalControllerGain");
  if (_sdf->HasElement("derivativeControllerGain"))
    derivative_controller_gain_ = _sdf->Get<double>("derivativeControllerGain");
  if (_sdf->HasElement("IntegralControllerGain"))
    integral_controller_gain_ = _sdf->Get<double>("IntegralControllerGain");
  if (_sdf->HasElement("maxVelocity"))
    maximum_velocity_ = _sdf->Get<double>("maxVelocity");
  if (_sdf->HasElement("torque"))
    maximum_torque_ = _sdf->Get<double>("torque");
  if (_sdf->HasElement("timeConstant"))
    time_constant_ = _sdf->Get<double>("maxVelocitytimeConstant");

  double control_rate = 0.0;
  if (_sdf->HasElement("controlRate")) control_rate = _sdf->Get<double>("controlRate");
  control_period_ = control_rate > 0.0 ? 1.0/control_rate : 0.0;

  servos_[FIRST].joint  = _model->GetJoint(servos_[FIRST].name);
  servos_[SECOND].joint = _model->GetJoint(servos_[SECOND].name);
  servos_[THIRD].joint  = _model->GetJoint(servos_[THIRD].name);


  if (!servos_[FIRST].joint)
    gzthrow("The controller couldn't get first joint");

  count_of_servos_ = 1;
  if (servos_[SECOND].joint)
    {
      count_of_servos_ = 2;
      if (servos_[THIRD].joint) count_of_servos_ = 3;
    }
  else
    {
      if (servos_[THIRD].joint)
          gzthrow("The controller couldn't get second joint, but third joint was loaded");
    }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  }

  node_handle_ = new ros::NodeHandle(robot_name_space_);

  transform_listener_ = new tf::TransformListener();
  transform_listener_->setExtrapolationLimit(ros::Duration(1.0));

  if (!imu_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::Imu>(
      imu_topic_, 1,
      boost::bind(&GazeboGimbal::imuCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    imu_subscriber_ = node_handle_->subscribe(ops);

    ROS_INFO_NAMED("gimbal servo", "Using imu information on topic %s as source of orientation and angular velocity.",
                   imu_topic_.c_str());
  }

  if (!state_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<nav_msgs::Odometry>(
      state_topic_, 1,
      boost::bind(&GazeboGimbal::stateCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    state_subscriber_ = node_handle_->subscribe(ops);

    ROS_INFO_NAMED("gimbal servo", "Using state information on topic %s as source of state information.",
                   state_topic_.c_str());
  }

  if (!joint_state_name_.empty())
    joint_state_pub_ = node_handle_->advertise<sensor_msgs::JointState>(joint_state_name_, 10);

  joint_state_.header.frame_id = transform_listener_->resolve(_model->GetLink()->GetName());

  Reset();

  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboGimbal::Update, this));
}

////////////////////////////////////////////////////////////////////////////////
// Callbacks
void GazeboGimbal::imuCallback(const sensor_msgs::ImuConstPtr& imu)
{
  pose_.rot.Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
  euler_ = pose_.rot.GetAsEuler();
}

void GazeboGimbal::stateCallback(const nav_msgs::OdometryConstPtr& state)
{
  if (imu_topic_.empty())
    {
      pose_.rot.Set(state->pose.pose.orientation.w, state->pose.pose.orientation.x,
                    state->pose.pose.orientation.y, state->pose.pose.orientation.z);
      euler_ = pose_.rot.GetAsEuler();
    }
}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboGimbal::Update()
{
  // handle callbacks
  callback_queue_.callAvailable();

  common::Time step_time_;
  step_time_ = world_->GetSimTime() - prev_update_time_;

  if (imu_topic_.empty())
    {
      pose_ = link_->GetWorldPose();
      euler_ = pose_.rot.GetAsEuler();
    }

  if (control_period_ == 0.0 || step_time_ > control_period_)
    {
      calculateVelocities(step_time_.Float());
      publish_joint_states();
      prev_update_time_ = world_->GetSimTime();
    }

  /* velocty feedback control */
  servos_[FIRST].joint->SetVelocity(0, servos_[FIRST].velocity);
  if (count_of_servos_ > 1)
    {
      servos_[SECOND].joint->SetVelocity(0, servos_[SECOND].velocity);
      if (count_of_servos_ > 2)
        servos_[THIRD].joint->SetVelocity(0, servos_[THIRD].velocity);
    }
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboGimbal::Reset()
{
  servos_[FIRST].velocity = 0;
  servos_[SECOND].velocity = 0;
  servos_[THIRD].velocity = 0;

  servos_[FIRST].i_term = 0;
  servos_[SECOND].i_term = 0;
  servos_[THIRD].i_term = 0;

  servos_[FIRST].angle = 0;
  servos_[SECOND].angle = 0;
  servos_[THIRD].angle = 0;

  servos_[FIRST].filter_command = 0;
  servos_[SECOND].filter_command = 0;
  servos_[THIRD].filter_command = 0;

  servos_[FIRST].joint->SetMaxForce(0, maximum_torque_);
  if (count_of_servos_ > 1)
    {
      servos_[SECOND].joint->SetMaxForce(0, maximum_torque_);
      if (count_of_servos_ > 2)
        servos_[THIRD].joint->SetMaxForce(0, maximum_torque_);
    }

  prev_update_time_ = world_->GetSimTime();

  pose_.Reset();
  euler_.Set();
  state_stamp_ = ros::Time();
}

  float GazeboGimbal::updatePID(Servo* servo, float new_input, float x, float dx, float dt)
{
  /* filter command */
  /* dinput = (new_input - input) / (dt + time_constant); */
  servo->filter_command  = (dt * new_input + time_constant_ * servo->filter_command)
    / (dt + time_constant_);

  float p_term = servo->filter_command - x;
  // d = dinput - dx;
  float d_term = - dx;
  servo->i_term += dt * p_term;

  float output = proportional_controller_gain_ * p_term
    + derivative_controller_gain_ * d_term + integral_controller_gain_ * servo->i_term;
  return output;
}


void GazeboGimbal::calculateVelocities(double dt)
{
  // double des_angle[3];
  double actual_angle[3] = {0.0, 0.0, 0.0};
  double actual_vel[3] = {0.0, 0.0, 0.0};

  servos_[0].angle = -euler_.x;
  servos_[1].angle = -euler_.y;
  servos_[2].angle = -euler_.z;  // Right?

  actual_angle[FIRST] = servos_[FIRST].joint->GetAngle(0).RADIAN();
  actual_vel[FIRST] = servos_[FIRST].joint->GetVelocity(0);

  // servos_[FIRST].velocity = updatePID(servos_[0].filter_command, servos_[0].angle,
  //                                     actual_angle[FIRST], actual_vel[FIRST], servos_[FIRST].i_term, dt);

  servos_[FIRST].velocity = updatePID(&servos_[0], servos_[0].angle,
                                      actual_angle[FIRST], actual_vel[FIRST], dt);


  if (maximum_velocity_ > 0.0 && fabs(servos_[FIRST].velocity) > maximum_velocity_)
    servos_[FIRST].velocity = (servos_[FIRST].velocity > 0 ? maximum_velocity_ : -maximum_velocity_);

  if (count_of_servos_ > 1)
    {
      actual_angle[SECOND] = servos_[SECOND].joint->GetAngle(0).RADIAN();
      actual_vel[SECOND] = servos_[SECOND].joint->GetVelocity(0);
      servos_[SECOND].velocity = updatePID(&servos_[1], servos_[1].angle,
                                           actual_angle[SECOND], actual_vel[SECOND], dt);
      if (maximum_velocity_ > 0.0 && fabs(servos_[SECOND].velocity) > maximum_velocity_)
        servos_[SECOND].velocity = (servos_[SECOND].velocity > 0 ? maximum_velocity_ : -maximum_velocity_);

      if (count_of_servos_ == 3)
        {
          actual_angle[THIRD] = servos_[THIRD].joint->GetAngle(0).RADIAN();
          actual_vel[THIRD] = servos_[THIRD].joint->GetVelocity(0);
          servos_[THIRD].velocity = updatePID(&servos_[2], servos_[2].angle,
                                              actual_angle[THIRD], actual_vel[THIRD], dt);
          if (maximum_velocity_ > 0.0 && fabs(servos_[THIRD].velocity) > maximum_velocity_)
            servos_[THIRD].velocity = (servos_[THIRD].velocity > 0 ? maximum_velocity_ : -maximum_velocity_);
        }
    }
}

void GazeboGimbal::publish_joint_states()
{
  if (!joint_state_pub_) return;

  joint_state_.header.stamp.sec = (world_->GetSimTime()).sec;
  joint_state_.header.stamp.nsec = (world_->GetSimTime()).nsec;

  joint_state_.name.resize(count_of_servos_);
  joint_state_.position.resize(count_of_servos_);
  joint_state_.velocity.resize(count_of_servos_);

  for (unsigned int i = 0; i < count_of_servos_; i++) {
    joint_state_.name[i] = servos_[i].joint->GetName();
    joint_state_.position[i] = servos_[i].joint->GetAngle(0).RADIAN();
    joint_state_.velocity[i] = servos_[i].joint->GetVelocity(0);
  }

  joint_state_pub_.publish(joint_state_);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboGimbal)

}  // namespace gazebo
