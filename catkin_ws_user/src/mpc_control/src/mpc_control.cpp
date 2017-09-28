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
/*Support Functions*/
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

    ros::Publisher pub_steering_;
    ros::Publisher pub_speed_;


    std_msgs::Int16 desired_steering;
    std_msgs::Int16 desired_speed;

  public:
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
      ROS_ERROR("Started MPC control node.");
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
  psi = tf::getYaw(pose.getRotation());
  //Check if this is ok - Does odom has this value ??
  velocity = msg->twist.twist.linear.x;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc_control_node");
  ros::NodeHandle nh;
  mpc_control control1(nh);
  // MPC is initialized here!
  MPC mpc;
  //5 Hz loop rate - for calculating a predicted path and publishing steering and speed
  ros::Rate loop_rate(5);
  while(ros::ok()){
    //Transform received Path to Car Frame of reference
    Eigen::VectorXd t_ptsx(control1.ptsx.size());
    Eigen::VectorXd t_ptsy(control1.ptsy.size());
    //Copy data to local variable as px,py, psi will be updated at very fast rate and might cause trouble in calculation
    //This will help have same set of values for one cycle
    //pts_x/y are not updated at fast rate it should be ok- TODO - check if it's k for need a copy
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

    //Solve the mpc equation and return the required values - steer, throttle, mpc x&y
    auto vars = mpc.Solve(state, coeffs);
    double steer_value = vars[0];
    double throttle_value = vars[1];
    //ROS_INFO("%d", vars[0]);
    //cout<< "result "<< vars<<endl;

    //TODO - Publish steering, throttle, map transformed points of trajectory

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
