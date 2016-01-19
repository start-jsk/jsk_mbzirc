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

  traversed_ = 0;
  last_time_ = world_->GetSimTime();
  terminated_ = false;

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
  pub_score_ = node_handle_->advertise<std_msgs::String>("score", 1, true); // set latch true
  pub_time_ = node_handle_->advertise<std_msgs::String>("remaining_time", 1);
  ros::NodeHandle param_handle(*node_handle_, "controller");



  update_connection_ = event::Events::ConnectWorldUpdateBegin(
                                                              boost::bind(&GazeboTruck::Update, this));
}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboTruck::Update()
{
  boost::mutex::scoped_lock scoped_lock(lock);

  if ( terminated_ ) {
    return;
  }
  //std::cerr << link_->GetWorldPose() << std::endl;
  common::Time current_time = world_->GetSimTime();
  double delta_time = (current_time-last_time_).Double();
  double theta = acos(CIRCLE_RADIUS/(CIRCLE_DISTANCE/2));
  double x, y, yaw;
  double l1 = CIRCLE_DISTANCE*sin(theta);
  double l2 = 2*CIRCLE_RADIUS*(M_PI-theta);
  double all_l = l1 + l2 + l1 + l2;

  //static const float VELOCITY = 4.16667;  //  4.16667 m/sec = 15 km/h, 2.77778 m/sec = 10 km/h, 1.38889 = 5 km/h
  if ( current_time.Double() < 6*60 ) {
    traversed_ += 4.16667 * delta_time;
  } else if ( current_time.Double() < 12*60 ) {
    traversed_ += 2.77778 * delta_time;
  } else if ( current_time.Double() < 20*60 ) {
    traversed_ += 1.38889 * delta_time;
  } else {
    ROS_FATAL("Time's up, Your challenge was over");
    terminated_ = true;
  }
  double l = fmod(traversed_, all_l);
  ROS_DEBUG_STREAM("time: " << current_time.Double() << ", traversed: " << traversed_);

  if ( l < l1  ) {
    x =  (l - l1/2)*sin(theta);
    y = -(l - l1/2)*cos(theta);
    yaw = theta-M_PI/2;
  } else if ( l < l1 + l2 ) {
    l = l - l1;
    x = -CIRCLE_RADIUS * cos(l/l2*(2*M_PI-theta*2)+theta) + CIRCLE_DISTANCE/2;
    y = -CIRCLE_RADIUS * sin(l/l2*(2*M_PI-theta*2)+theta);
    yaw = (l/l2*(2*M_PI-theta*2)+theta)-M_PI/2;
  } else if ( l < l1 + l2 + l1 ) {
    l = l - (l1 + l2);
    x = -(l - l1/2)*sin(theta);
    y = -(l - l1/2)*cos(theta);
    yaw =-M_PI/2-theta;
  } else {
    l = l - (l1 + l2 + l1);
    x =  CIRCLE_RADIUS * cos(l/l2*(2*M_PI-theta*2)+theta) - CIRCLE_DISTANCE/2;
    y = -CIRCLE_RADIUS * sin(l/l2*(2*M_PI-theta*2)+theta);
    yaw = -(l/l2*(2*M_PI-theta*2)+theta)-M_PI/2;
  }
  model_->SetLinkWorldPose(math::Pose(x, y, 0.595, 0, 0, yaw), link_);
  last_time_ = world_->GetSimTime();

  // check score
  // void Entity::GetNearestEntityBelow(double &_distBelow,  std::string &_entityName)

  gazebo::physics::RayShapePtr rayShape = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(world_->GetPhysicsEngine()->CreateShape("ray", gazebo::physics::CollisionPtr()));

  double distAbove;
  std::string entityName;
  math::Box box = model_->GetLink("heliport")->GetCollisionBoundingBox();
  math::Vector3 start = model_->GetLink("heliport")->GetWorldPose().pos;
  math::Vector3 end = start;
  start.z = box.max.z + 0.00001;
  end.z += 1000;
  rayShape->SetPoints(start, end);
  rayShape->GetIntersection(distAbove, entityName);
  distAbove -= 0.00001;

  //
  std::stringstream ss;
  std_msgs::String msg_time;
  ss << 20*60 - current_time.Double();
  msg_time.data = ss.str();
  pub_time_.publish(msg_time);
  if ( entityName != "" && distAbove < 1.0 ) {
    std_msgs::String msg_score, msg_time;
    msg_score.data = "Mission Completed";
    ROS_INFO_STREAM("Remaining time is " << msg_time.data << "[sec], Score is " << msg_score.data);
    pub_score_.publish(msg_score);
    terminated_ = true;
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
