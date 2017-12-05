#include "ros/ros.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "spline.h"
#include "visualization.h"
#include "geometry_msgs/PoseStamped.h"

void pub_odom(ros::Publisher&  odom_pub){
  ros::Time current_time = ros::Time::now();
  tf::TransformBroadcaster odom_broadcaster;
  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = 0;
  odom_trans.transform.translation.y = 0;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = 0;
  odom.pose.pose.position.y = 0;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = 0;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = 0;

  //publish the message
  odom_pub.publish(odom);

}

void pub_path(ros::Publisher&  s_path_pub){
  nav_msgs::Path ptp;
  geometry_msgs::PoseStamped pose;
  std::vector<geometry_msgs::PoseStamped> plan;
  for (int i=0; i<5; i++){
    pose.pose.position.x = i;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.push_back(pose);
  }

  ptp.poses.resize(plan.size());

  if(!plan.empty()){
      ptp.header.frame_id = "map";
      ptp.header.stamp = ros::Time::now();
  }

  for(unsigned int i=0; i < plan.size(); i++){
      ptp.poses[i] = plan[i];
  }
  s_path_pub.publish(ptp);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy");
  ros::NodeHandle nh;
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("d_odom", 50);
  ros::Publisher s_path_pub = nh.advertise<nav_msgs::Path>("s_path", 3);
  //ros::Publisher disp_pred_path;//,ref_path;
  //disp_pred_path = nh.advertise<visualization_msgs::Marker>("/pred_path_marker", 20);
  //disp_ref_path  = nh.advertise<visualization_msgs::Marker>("/ref_path_marker", 20);

  //TODO Check the required frequency
  ros::Rate loop_rate(5);
  while(ros::ok()){
    pub_odom(odom_pub);
    //pub_path(s_path_pub);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
