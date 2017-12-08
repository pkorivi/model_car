/*

 */
#include "VehicleState.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace fub_motion_planner{
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

  void VehicleState::setup(ros::NodeHandle & nh)
  {
      // TODO: increase odom queue to at least 32
      //TODO change d_odom to odom when subscribing to proper node
      ROS_INFO("Vehicle_State setup");
      m_subscribe_odom         = nh.subscribe("/d_odom", 1, &VehicleState::odometryCallback, this, ros::TransportHints().tcpNoDelay());
      //ROS_INFO("Vehicle state setup");
  }

  void VehicleState::odometryCallback(const nav_msgs::OdometryConstPtr & msg){
    //ROS_INFO("odom_received");
    m_ego_state_pose = msg->pose;
    m_current_speed_front_axle_center = (double) msg->twist.twist.linear.x;
    tf::pointMsgToTF(m_ego_state_pose.pose.position, m_vehicle_position);
    //ROS_INFO("x: %f, y: %f, %f", m_vehicle_position[0],m_vehicle_position[1], msg->pose.pose.position.x);
    m_last_odom_time_stamp_received = msg->header.stamp;
  }

  double VehicleState::getVehicleYaw() const{
      return tf::getYaw(m_ego_state_pose.pose.orientation);// * radians;
  }

} // namespace sample_nodelet_ns

//PLUGINLIB_EXPORT_CLASS(fub_motion_planner::VehicleState, nodelet::Nodelet)
