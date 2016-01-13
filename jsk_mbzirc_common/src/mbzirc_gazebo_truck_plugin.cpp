#include <jsk_mbzirc_common/mbzirc_gazebo_truck_plugin.h>

namespace gazebo {

GazeboTruck::GazeboTruck()
{
}

GazeboTruck::~GazeboTruck()
{
  event::Events::DisconnectWorldUpdateBegin(update_connection_);

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboTruck::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gzwarn << "The GazeboTruck plugin is DEPRECATED in ROS hydro." << std::endl;

  model_ = _model;
  world_ = _model->GetWorld();
  link_ = _model->GetLink();
  link_name_ = link_->GetName();
  namespace_.clear();

  x_ = 0;
  y_ = 0;
  direction_ = 1;
  circle_ = false;

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
  ros::NodeHandle param_handle(*node_handle_, "controller");



  update_connection_ = event::Events::ConnectWorldUpdateBegin(
                                                              boost::bind(&GazeboTruck::Update, this));
}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboTruck::Update()
{

  if(x_ >= DISTANCE_THRE)
  {
    if(circle_ = false)
    {
      circle_ = true;
      direction_ *= -1;
    }

  }
  else if(x_ <= -DISTANCE_THRE)
  {
    if(circle_ = false)
    {
      circle_ = true;
      direction_ *= -1;
    }
  }
  else
  {
    circle_ = false;

    if(direction_ = 1)
      model_->SetLinearVel(math::Vector3(VELOCITY/sqrt(2), VELOCITY/sqrt(2), 0));



  }




}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboTruck::Reset()
{
  state_stamp_ = ros::Time();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboTruck)

} // namespace gazebo
