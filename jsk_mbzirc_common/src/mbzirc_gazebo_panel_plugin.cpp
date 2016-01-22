#include <jsk_mbzirc_common/mbzirc_gazebo_panel_plugin.h>

namespace gazebo {

GazeboPanel::GazeboPanel()
{
}

GazeboPanel::~GazeboPanel()
{
  event::Events::DisconnectWorldUpdateBegin(update_connection_);

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboPanel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gzwarn << "The GazeboPanel plugin is DEPRECATED in ROS hydro." << std::endl;

  model_ = _model;
  world_ = _model->GetWorld();
  link_ = _model->GetLink();
  link_name_ = link_->GetName();
  namespace_.clear();

  terminated_ = false;

  // load parameters from sdf
  if (_sdf->HasElement("robotNamespace")) namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();

  if (_sdf->HasElement("bodyName") && _sdf->GetElement("bodyName")->GetValue()) {
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
    link_ = _model->GetLink(link_name_);
  }

  if (_sdf->HasElement("jointName") && _sdf->GetElement("jointName")->GetValue()) {
    std::string joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
    joint_ = _model->GetJoint(joint_name_);
  } else {
    joint_ = _model->GetJoint("stem_joint");
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
  pub_score_ = node_handle_->advertise<std_msgs::String>("score", 1, true); // set latch true
  pub_time_ = node_handle_->advertise<std_msgs::String>("remaining_time", 1);
  ros::NodeHandle param_handle(*node_handle_, "controller");



  update_connection_ = event::Events::ConnectWorldUpdateBegin(
                                                              boost::bind(&GazeboPanel::Update, this));
}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboPanel::Update()
{

  boost::mutex::scoped_lock scoped_lock(lock);

  if ( terminated_ ) {
    return;
  }

  common::Time current_time = world_->GetSimTime();

  // check score
  double angle = joint_->GetAngle(0).Radian();
  if ( fmod(current_time.Double(), 1) == 0 ) {
    ROS_INFO_STREAM("Current time is " << current_time.Double() << ", Current angle is = " << angle);
  }

  // void Entity::GetNearestEntityBelow(double &_distBelow,  std::string &_entityName)
  //
  std::stringstream ss;
  std_msgs::String msg_time;
  ss << 20*60 - current_time.Double();
  msg_time.data = ss.str();
  pub_time_.publish(msg_time);
  if ( fabs(angle) > 3.14) {
    std_msgs::String msg_score, msg_time;
    msg_score.data = "Mission Completed";
    ROS_INFO_STREAM("Remaining time is " << msg_time.data << "[sec], Score is " << msg_score.data);
    pub_score_.publish(msg_score);
    terminated_ = true;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboPanel::Reset()
{
  state_stamp_ = ros::Time();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboPanel)

} // namespace gazebo
