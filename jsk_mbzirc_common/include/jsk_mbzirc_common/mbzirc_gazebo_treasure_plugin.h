#ifndef MBZIRC_GAZEBO_TREASURE_PLUGIN_H
#define MBZIRC_GAZEBO_TREASURE_PLUGIN_H

#include <boost/bind.hpp>
#include <boost/random.hpp>
#include <boost/random/random_device.hpp>

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

namespace gazebo
{

class GazeboTreasure : public ModelPlugin
{
public:
  GazeboTreasure();
  virtual ~GazeboTreasure();

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
  ros::Publisher pub_score_;
  ros::Time state_stamp_;

  event::ConnectionPtr update_connection_;

  double vel_x, vel_y, vel_yaw;
  common::Time last_time_;
  bool terminated_;
  bool static_object_;
};

}

#endif
