/*
Vehicle State class - Holds vehicle state data, obstacle information needed for planning
 */
#include "VehicleState.h"

namespace fub_motion_planner{
  /** Class Constructor and Destructor
  **
  ** @param Currently None
  */
  VehicleState::VehicleState(){
    //ROS_INFO("VehicleState Constructor");
  }
  VehicleState::~VehicleState(){
    //ROS_INFO("VehicleState Destructor");
  }
  /*//This is needed if this state should be a seperate nodelet
  void VehicleState::onInit(){
      NODELET_INFO("VehicleState - %s", __FUNCTION__);
  }*/

  /** Setup function - generally called from init of the Nodelet to setup this class.
  **
  ** @param nodehandle
  */
  void VehicleState::setup(ros::NodeHandle & nh)
  {
      ROS_INFO("Vehicle_State setup");
      m_subscribe_odom  = nh.subscribe("/odom", 32, &VehicleState::odometryCallback, this, ros::TransportHints().tcpNoDelay());
      m_subscribe_obstacles = nh.subscribe("obstacles", 1, &VehicleState::callbackObstacles, this,ros::TransportHints().tcpNoDelay());
      //ROS_INFO("Vehicle state setup");
  }

  /** Callback for Odometry messages.
  **
  ** @param msg
  */
  void VehicleState::odometryCallback(const nav_msgs::OdometryConstPtr & msg){
    //ROS_INFO("odom_received");
    m_ego_state_pose = msg->pose;
    m_current_speed_front_axle_center = (double) msg->twist.twist.linear.x;
    tf::pointMsgToTF(m_ego_state_pose.pose.position, m_vehicle_position);
    //ROS_INFO("x,y: %f, %f,x,y %f, %f", m_vehicle_position[0],m_vehicle_position[1], msg->pose.pose.position.x,msg->pose.pose.position.y);
    m_last_odom_time_stamp_received = msg->header.stamp;
  }

  /** Callback for Obstacle messages.
  **
  ** @param msg
  */
  void VehicleState::callbackObstacles(const autonomos_obstacle_msgs::ObstaclesConstPtr & msg)
  {
    m_obstacle_msg = msg;
    //std::cout << "Callback obstacles " << m_obstacle_msg->obstacles[0].abs_velocity.twist.linear.x << '\n';
  }

  /** Function to compute vehicle yaw from the odom pose orientation
  **
  ** @param None
  */
  double VehicleState::getVehicleYaw() const{
      double yaw = tf::getYaw(m_ego_state_pose.pose.orientation);// * radians;
      return std::isnan(yaw)?0:yaw;
  }

} // namespace sample_nodelet_ns

//PLUGINLIB_EXPORT_CLASS(fub_motion_planner::VehicleState, nodelet::Nodelet)
