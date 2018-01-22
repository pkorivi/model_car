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
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"

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
  odom.pose.pose.position.x = 4.5;
  odom.pose.pose.position.y = -0.26;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = 0.00;
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

/*
car current d, target d, and angle between the road and the car, s_i initial s, sf final s
following as per  'REAL-TIME TRAJECTORY PLANNING FOR AUTONOMOUS URBAN DRIVING paper'
*/
std::vector<double> evaluate_d_coeffs(double d_cur,double d_tgt, double theta, double s_i, double s_f){
  //TODO change the number
  std::cout << "inside func" << '\n';
  clock_t tStart = clock();
  Eigen::MatrixXd A = Eigen::MatrixXd(4,4);
  double si_sq = s_i*s_i;
  double sf_sq = s_f*s_f;
  //Matrix A with s equations
  A << 1, s_i, si_sq , si_sq*s_i,
       0, 1  , 2*s_i , 3*si_sq,
       0, 1  , 2*s_f , 3*sf_sq,
       1, s_f, sf_sq , sf_sq*s_f;

  Eigen::MatrixXd B = Eigen::MatrixXd(4,1);
  B << d_cur,
       theta,
       0,
       d_tgt;
  Eigen::MatrixXd Ai = A.inverse();
  Eigen::MatrixXd C = Ai*B;
  std::vector<double> d_coeffs;
  for (size_t i = 0; i < C.size(); i++) {
    d_coeffs.push_back(C.data()[i]);
  }
  //ROS_INFO("polyfit d: %f\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  return d_coeffs;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy");
  ros::NodeHandle nh;
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Publisher s_path_pub = nh.advertise<nav_msgs::Path>("s_path", 3);
  //ros::Publisher disp_pred_path;//,ref_path;
  //disp_pred_path = nh.advertise<visualization_msgs::Marker>("/pred_path_marker", 20);
  //disp_ref_path  = nh.advertise<visualization_msgs::Marker>("/ref_path_marker", 20);

  //TODO Check the required frequency
  ros::Rate loop_rate(5);
  while(ros::ok()){
    //pub_odom(odom_pub);
    //pub_path(s_path_pub);
    /*
    std::vector<double> coeffs = evaluate_d_coeffs(0.2,-0.2,0,0,3);
    for (size_t i = 0; i < coeffs.size(); i++) {
      std::cout << coeffs[i]<<"  ";
    }
    std::cout << " " << '\n';
    */
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
