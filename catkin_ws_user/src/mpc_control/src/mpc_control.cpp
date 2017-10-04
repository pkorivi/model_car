#include "ros/ros.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include <visualization_msgs/Marker.h>
/*Support Functions*/
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

//TODO - find a better place to place this function
//Markers to Display - Predicted Path from MPC
ros::Publisher pred_path_marker_pub;
ros::Publisher ref_path_marker_pub;
// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}


class mpc_control
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;
    ros::Subscriber sub_path_points;
    ros::Subscriber sub_odom_;

    std_msgs::Int16 desired_steering;
    std_msgs::Int16 desired_speed;

  public:
    ros::Publisher pub_steering_;
    ros::Publisher pub_speed_;
    //Path for control node to follow - update in Received Path Node
    vector<double> ptsx = {1,5,10,15};
    vector<double> ptsy = {1,1,1,1};
    //State Variables - Update in odom subscribe node
    double px = 0;
    double py = 0;
    double psi = 0;
    double velocity = 0;

    //TODO Subscribe to some message to update these values or check if they are needed
    double c_st_angle = 0;
    double c_throttle = 0;

    mpc_control(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
      pub_steering_= nh.advertise<std_msgs::Int16>(nh.resolveName("/manual_control/steering"), 1);
      pub_speed_= nh.advertise<std_msgs::Int16>(nh.resolveName("/manual_control/speed"), 1);
      //TODO - check if the speed and orientation is included in odometry
      // Receive the Path from Motion Planner
      sub_path_points =nh_.subscribe( "/motion_planner/Path", 1,&mpc_control::path_callback,this);
      //subscribe to odometry
      sub_odom_ = nh_.subscribe( "/odom", 1, &mpc_control::odom_callback,this);
      ROS_INFO("Started MPC control node.");
    }
    ~mpc_control(){}
    //TODO Create proper Functions
    void path_callback(const nav_msgs::Path::ConstPtr& );
    void odom_callback(const nav_msgs::Odometry::ConstPtr&);
};


//Update the path from motion planner to be used by the control node
void mpc_control::path_callback(const nav_msgs::Path::ConstPtr& mp_path){
  //TODO - check if the order of elements is ok
  //Clear the old vector
  ptsx.clear();
  ptsy.clear();
  //Update with new Path
  for(int i=0;i<sizeof(mp_path->poses);i++){
    ptsx.push_back(mp_path->poses[i].pose.position.x);
    ptsy.push_back(mp_path->poses[i].pose.position.y);
  }
}

//Update the state of the vehicle form Odometry Message
void mpc_control::odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
  px = msg->pose.pose.position.x;
  py = msg->pose.pose.position.y;
  //TODO How to handle orientation
  tf::Pose pose;
  tf::poseMsgToTF(msg->pose.pose, pose);
  //TODO - check correctness
  psi = tf::getYaw(pose.getRotation());
  velocity = msg->twist.twist.linear.x;
}

void visulize_pred_path(std::vector<double> pts_pred){
    visualization_msgs::Marker points, line_strip;
    points.header.frame_id = line_strip.header.frame_id = "/base_link";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "points_and_lines";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    points.id = 0;
    line_strip.id = 1;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.1;
    points.scale.y = 0.1;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < pts_pred.size(); i = i+2)
    {
      geometry_msgs::Point p;
      p.x = pts_pred[i];
      p.y = pts_pred[i+1];
      p.z = 0;
      points.points.push_back(p);
      line_strip.points.push_back(p);
    }
    pred_path_marker_pub.publish(points);
    pred_path_marker_pub.publish(line_strip);

}

void visulize_ref_path(std::vector<double> pts_ref){
    visualization_msgs::Marker points, line_strip;
    points.header.frame_id = line_strip.header.frame_id = "/map";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "points_and_lines";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    points.id = 0;
    line_strip.id = 1;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.1;
    points.scale.y = 0.1;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    // Points are green + blue
    points.color.g = 1.0f;
    points.color.b = 1.0f;
    points.color.a = 1.0;

    // Line strip is green + red
    line_strip.color.g = 1.0;
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;

    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < pts_ref.size(); i = i+2)
    {
      geometry_msgs::Point p;
      p.x = pts_ref[i];
      p.y = pts_ref[i+1];
      p.z = 0;
      points.points.push_back(p);
      line_strip.points.push_back(p);
    }
    ref_path_marker_pub.publish(points);
    ref_path_marker_pub.publish(line_strip);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc_control_node");
  ros::NodeHandle nh;
  mpc_control control1(nh);

  // MPC is initialized here!
  MPC mpc;

  pred_path_marker_pub = nh.advertise<visualization_msgs::Marker>("mpc_pred_points", 20);
  ref_path_marker_pub = nh.advertise<visualization_msgs::Marker>("mpc_ref_points", 20);
  //5 Hz loop rate - for calculating a predicted path and publishing steering and speed
  ros::Rate loop_rate(10);
  while(ros::ok()){
    //Transform received Path to Car Frame of reference
    Eigen::VectorXd t_ptsx(control1.ptsx.size());
    Eigen::VectorXd t_ptsy(control1.ptsy.size());
    //Copy data to local variable as px,py, psi will be updated at very fast rate and might cause trouble in calculation
    //This will help have same set of values for one cycle
    //pts_x/y are not updated at fast rate it should be ok- TODO - check if it needs a copy
    double l_px = control1.px;
    double l_py = control1.py;
    double l_psi = control1.psi;
    //psi is in radians
    for(size_t i=0; i<control1. ptsx.size();i++){
      t_ptsx[i] = (control1.ptsx[i] - l_px)*cos(-l_psi) - (control1.ptsy[i] - l_py)*sin(-l_psi);
      t_ptsy[i] = (control1.ptsx[i] - l_px)*sin(-l_psi) + (control1.ptsy[i] - l_py)*cos(-l_psi);
    }

    //Fit a 3rd order polynomial - fit it from car perspective.
    auto coeffs = polyfit(t_ptsx, t_ptsy,3);
    //polynomial - c3x³+c2x²+c1x+c0;
    //evaluate the error at origin.
    double cte = coeffs[0];
    double epsi = -atan(coeffs[1]);

    Eigen::VectorXd state(8);
    //x,y,psi,v,cte,epsi - x,y,psi = 0 as frame of reference is shifted. TODO check if st_angle and throttle are needed
    state<<0,0,0,control1.velocity,cte,epsi,control1.c_st_angle,control1.c_throttle;
    //cout<<"state :: "<<state<<endl;

    //Solve the mpc equation and return the required values - steer, throttle, mpc x&y
    auto vars = mpc.Solve(state, coeffs);
    std_msgs::Int16 steer_value;
    std_msgs::Int16 speed_value;

    /* TODO - verify these conversions
    1000rpm for motor - then
    wheel speed max =  1000/(5.5*60) = 3.03rot per sec => 3.03*2*3.14*0.031 = 0.5899 mps
    if 1000rpm is wheel speed then
    wheel speed max  =  1000/60 ==> 3.244 mps. From simulation - this is correct
    if 3.24 mps is max velocity, whats the time I want to attain it?
    lets say in 5s then -> 3.24/5 = 0.65 mpss
    min max accc =  [-0.65, 0.65] lets limit it to [-0.5:0.5]
    1m - 5.1366 rotations of wheel.
    */

    //TODO - define the scales properly
    // assuming steering to be in -45 to 45 in radians convert it to degrees and as 90 is straight, this values convert as required by car
    //TODO substrcted from 90 as car was steering in opposite - just check again if it's fine
    steer_value.data  = (int)(90  - 57.2958*vars[0]);
    //TODO - use a proper value as per rate
    float loop_time = 0.1; //period of loop - convert based on loop rate
    //calculate the required velocity - then convert to rotations per min by multiplying with 5.1366 and 60
    speed_value.data = (int)((control1.velocity + vars[1]*loop_time)*5.1366*60);
    //TODO - Come up with better logic
    if (speed_value.data < 50 && speed_value.data > 0) {
      speed_value.data = 50;
    }
    //TODO - can car go reverse ?? set these limits in the solver.
    else if (speed_value.data > -50 && speed_value.data < 0) {
      speed_value.data = -50;
    }
    //As the robot takes negative values
    speed_value.data = -speed_value.data;
    ROS_INFO("pos:: (%0.2f,%0.2f) , steer: %d, throttle : %0.2f speed  %d %0.2f cte : %0.3f, epsi %0.3f",l_px,l_py,steer_value.data, vars[1], speed_value.data, control1.velocity, cte, epsi);
    //cout<< "result st "<< steer_value<<" th"<<throttle_value<<endl;

    //convert points to map coordinates
    //psi is in radians
    std::vector<double> disp_pts;
    for(size_t i=2; i<vars.size();i=i+2){
      //double tx = (vars[i] + l_px)*cos(l_psi) - (vars[i+1] + l_py)*sin(l_psi);
      //double ty = (vars[i] + l_px)*sin(l_psi) + (vars[i+1] + l_py)*cos(l_psi);
      disp_pts.push_back(vars[i]);
      disp_pts.push_back(vars[i+1]);
    }
    //Visulaize the predicted path
    visulize_pred_path(disp_pts);
    //Visulaize reference path
    std::vector<double> ref_pts;
    for(size_t i =0; i< control1.ptsx.size();i++){
      ref_pts.push_back(control1.ptsx[i]);
      ref_pts.push_back(control1.ptsy[i]);
    }
    visulize_ref_path(ref_pts);

    //Publish steering, throttle - for robot to drive
    control1.pub_steering_.publish(steer_value);
    control1.pub_speed_.publish(speed_value);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
