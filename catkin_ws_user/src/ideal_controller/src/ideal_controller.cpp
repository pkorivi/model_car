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
#include "geometry_msgs/PoseStamped.h"
#include <fub_trajectory_msgs/Trajectory.h>
#include <tf/tf.h>
#include <ecl/geometry.hpp>
#include <ecl/containers.hpp>


fub_trajectory_msgs::Trajectory mPath;
ecl::CubicSpline mSpline_x;
ecl::CubicSpline mSpline_y;
ecl::CubicSpline mSpline_x_new;
ecl::CubicSpline mSpline_y_new;
bool path_received = false;
bool path_end_reached = true;
double time_to_check = 0.0;
double plan_to_control_latency =0;

ros::Publisher odom_pub;
ros::Publisher mSplineSamplePublisher;
//Global position, yaw and velocity
double x=0,y=0,yaw=0,vel=0,prev_yaw=0, prev_x=0, prev_y=0, prev_vel = 0;

void createSpline(fub_trajectory_msgs::Trajectory const & plan){
    // read path x and ys into separate arrays
    int numberOfPointsIntoSpline = plan.trajectory.size();
    ecl::Array<double> x_set(numberOfPointsIntoSpline);
    ecl::Array<double> y_set_x(numberOfPointsIntoSpline);
    ecl::Array<double> y_set_y(numberOfPointsIntoSpline);

    for (int i = 0 ; i < numberOfPointsIntoSpline; i++) {
        //-0.01 is trail, because the planner starts next cycle, assuming a mid delay of 0.005ms
        x_set[i] = plan.trajectory.at(i).time_from_start.toSec();//-plan_to_control_latency-0.005;
        y_set_x[i] = plan.trajectory.at(i).pose.position.x;
        y_set_y[i] = plan.trajectory.at(i).pose.position.y;
    }

    //calculate velocity vector with regard to global coordinates
    tf::Vector3 frontVelocity;
    tf::vector3MsgToTF(plan.trajectory.front().velocity.linear, frontVelocity);
    tf::Quaternion frontOrientation;
    tf::quaternionMsgToTF(plan.trajectory.front().pose.orientation, frontOrientation);
    frontVelocity = frontVelocity.rotate(frontOrientation.getAxis(), frontOrientation.getAngle());

    tf::Vector3 backVelocity;
    tf::vector3MsgToTF(plan.trajectory.back().velocity.linear, backVelocity);
    tf::Quaternion backOrientation;
    tf::quaternionMsgToTF(plan.trajectory.back().pose.orientation, backOrientation);
    backVelocity = backVelocity.rotate(backOrientation.getAxis(), backOrientation.getAngle());

    //create splines for each dimension from the arrays
    mSpline_x_new = ecl::CubicSpline::ContinuousDerivatives(x_set, y_set_x, frontVelocity.getX(), backVelocity.getX());
    mSpline_y_new = ecl::CubicSpline::ContinuousDerivatives(x_set, y_set_y, frontVelocity.getY(), backVelocity.getY());

    /*
    double velocity_f = plan.trajectory.front().velocity.linear.x;
    double velocity_b = plan.trajectory.back().velocity.linear.x;
    double front_orien = tf::getYaw(plan.trajectory.front().pose.orientation);
    front_orien =  std::isnan(front_orien)?0:front_orien;
    double back_orien = tf::getYaw(plan.trajectory.back().pose.orientation);
    back_orien =  std::isnan(back_orien)?0:back_orien;
    mSpline_x_new = ecl::CubicSpline::ContinuousDerivatives(x_set, y_set_x, velocity_f*cos(front_orien), velocity_b*sin(front_orien));
    mSpline_y_new = ecl::CubicSpline::ContinuousDerivatives(x_set, y_set_y, velocity_b*cos(back_orien), velocity_b*sin(back_orien));
    std::cout << "fv x,y"<<frontVelocity.getX()<<","<<frontVelocity.getY()<<" bv x,y "<<backVelocity.getX()<<","<<backVelocity.getY()<< '\n';
    */
    //Initial values
    //x = mSpline_x_new(0);
    //y = mSpline_y_new(0);
    //vel = tf::Vector3(mSpline_x_new.derivative(0),mSpline_y_new.derivative(0),0).length();
    //yaw = atan2(mSpline_y_new.derivative(0), mSpline_x_new.derivative(0));
    //prev_yaw = yaw;
}

void publishSampledSpline(){
    std::cout << "sampled path" << '\n';
    nav_msgs::Path mSampledSplineDebug;
    mSampledSplineDebug.header.stamp = ros::Time::now();
    mSampledSplineDebug.header.frame_id = "/odom";

    mSampledSplineDebug.poses.clear();

    float xSampleDistance = 0.01; //in m

    for (float i = 0; i < 4.9; i = i + xSampleDistance) {
      try{
        geometry_msgs::PoseStamped examplePose;
        examplePose.pose.position.x = mSpline_x(i);
        examplePose.pose.position.y = mSpline_y(i);
        examplePose.pose.position.z = 0;
        double yaw_local = atan2(mSpline_y.derivative(i), mSpline_x.derivative(i));
        geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(yaw_local);
        // fill orientation with proper orientation of the points - like odom
        examplePose.pose.orientation = pose_quat;
        //push PoseStamped into Path
        mSampledSplineDebug.poses.push_back(examplePose);
      }
      catch(const std::exception &e){
        //std::cout <<"i: "<<i << '\n';
      }

    }
    mSplineSamplePublisher.publish(mSampledSplineDebug);
}


void pub_odom(double odom_at_time){
  if (path_end_reached == false) {
    x = mSpline_x(odom_at_time);
    y = mSpline_y(odom_at_time);

    double dx = round((x-prev_x)*1000)/1000.0;
    double dy = round((y-prev_y)*1000)/1000.0;
    vel = sqrt(dx*dx + dy*dy) / 0.01;
    yaw = atan2(dy,dx);

    //vel = tf::Vector3(mSpline_x.derivative(odom_at_time),mSpline_y.derivative(odom_at_time),0).length();
    //yaw = atan2(mSpline_y.derivative(odom_at_time), mSpline_x.derivative(odom_at_time));

    if (yaw>M_PI)
      yaw=yaw-2*M_PI;
    else if (yaw<-M_PI)
      yaw=yaw+2*M_PI;

    //If chnage is greater than 20 degrees - //TODO change to 15 degrees
    double yaw_diff = yaw -prev_yaw;
    if (yaw_diff>M_PI)
      yaw_diff=yaw_diff-2*M_PI;
    else if (yaw_diff<-M_PI)
      yaw_diff=yaw_diff+2*M_PI;

    if (fabs(yaw_diff)>M_PI/6) {
      ROS_INFO("old_new_yaw_diff--  x,y %.3f,%.3f yaw %.3f , prev_yaw %.3f, vel %.3f",x,y,yaw,prev_yaw,vel);
      yaw = prev_yaw;
    }

    if (fabs(vel-prev_vel)>0.1) {
      vel = (vel+prev_vel)/2;
    }
    //ROS_INFO("t %.3f x,y %.3f,%.3f vel %.3f yaw %.3f , dx,dy %.4f,%.4f, prev x,y,yaw %.3f,%.3f,%.3f",odom_at_time,x,y,vel,yaw, dx,dy,prev_x,prev_y,prev_yaw);
    prev_yaw = yaw;
    prev_x = x;
    prev_y = y;
    prev_vel = vel;
  }
  ros::Time current_time = ros::Time::now();
  tf::TransformBroadcaster odom_broadcaster;
  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);
  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vel;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = 0;

  //publish the message
  odom_pub.publish(odom);
}

void plannedPathCallback(const fub_trajectory_msgs::TrajectoryConstPtr & msg){
    std::cout << "Received the Path "<<'\n';
    mPath = *msg;
    plan_to_control_latency = (ros::Time::now() - mPath.header.stamp).toSec();
    createSpline(mPath);
    path_received = true;
    path_end_reached = false;
    time_to_check = 0;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "Ideal_Controller");
  ros::NodeHandle nh;
  ros::Subscriber mSubscriberPlannedPath = nh.subscribe("/model_car/trajectory", 1, &plannedPathCallback, ros::TransportHints().tcpNoDelay());
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 32);
  mSplineSamplePublisher = nh.advertise<nav_msgs::Path>("/fub_controller_mig/sampled_spline_debug", 10);
  double loop_frequency = 100;
  double loop_period = 1/loop_frequency;
  //TODO Check the required frequency
  ros::Rate loop_rate(loop_frequency);
  while(ros::ok()){
    //End of received path
    if (time_to_check > 4.9) {
      path_received = false;
      path_end_reached = true;
      time_to_check = 0;
    }
    if (path_received==true) {
      mSpline_x = mSpline_x_new;
      mSpline_y = mSpline_y_new;
      publishSampledSpline();
      path_received = false;
    }
    //Publish Odom - from the trajectory received
    //add 10m latency
    pub_odom(time_to_check+0.02);
    time_to_check += loop_period;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
