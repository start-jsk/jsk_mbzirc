
#ifndef MBZIRC_GAZEBO_TRUCK_PLUGIN_H
#define MBZIRC_GAZEBO_TRUCK_PLUGIN_H

#include <boost/bind.hpp>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/common.hh>

#include <stdio.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

namespace gazebo
{

class GazeboTruck : public ModelPlugin
{
public:
  GazeboTruck();
  virtual ~GazeboTruck();

  static const float DISTANCE_THRE = 14.14214; // the end of stright road(x or y coordinate)
  static const float CIRCLE_CENTER = 28.28427; 
  static const float VELOCITY = 4.16667; 

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

  ros::Time state_stamp_;

  event::ConnectionPtr update_connection_;

float x_;
float y_;
float direction_;
bool cicle_;

};

}

#endif
