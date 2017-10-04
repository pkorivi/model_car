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


class pid_control
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;
    ros::Subscriber sub_curvature_;
    //subscriber for path points from motion planner
    ros::Subscriber sub_path_points;
    //subscribe to odometry values
    ros::Subscriber sub_odom_;

    std_msgs::Int16 desired_steering;
    std_msgs::Int16 desired_speed;


    float steering_Kp_;
    float steering_Kd_;
    float steering_Ki_;
    float cte_Kp_;
    float cte_Kd_;
    float cte_Ki_;
    float speed_Kp_;
    float maximum_rpm_;
    float minimum_rpm_;
    float maximum_steering_;
    float minimum_steering_;
    float p_cte;
    float i_cte;
    float d_cte;
    float prev_cte;
    float total_cte;
    bool error_initial;

  public:
    ros::Publisher pub_steering_ ;
    ros::Publisher pub_speed_ ;
    //Path for control node to follow - update in Received Path Node
    std::vector<double> ptsx = {1,5,10,15};
    std::vector<double> ptsy = {1,1,1,1};
    //State Variables - Update in odom subscribe node
    double px = 0;
    double py = 0;
    double psi = 0;
    double velocity = 0;

    //TODO Subscribe to some message to update these values or check if they are needed
    double c_st_angle = 0;
    double c_throttle = 0;
    //Remove all curvature stuff*
    void curvatureCallback(const std_msgs::Float32 curvaturee);
    void path_callback(const nav_msgs::Path::ConstPtr& );
    void odom_callback(const nav_msgs::Odometry::ConstPtr&);
    double pid_cte_error(double cte);

    pid_control(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
      priv_nh_.param<float>("steering_Kp", steering_Kp_, 0.7);
      priv_nh_.param<float>("steering_Kd", steering_Kd_, 1.0);
      priv_nh_.param<float>("steering_Ki", steering_Ki_, 0.0);
      priv_nh_.param<float>("speed_Kp", speed_Kp_, 0.05);
      priv_nh_.param<float>("maximum_rpm", maximum_rpm_, 1000);
      priv_nh_.param<float>("minimum_rpm", minimum_rpm_, 20);
      priv_nh_.param<float>("maximum_steering", maximum_steering_, 50);
      priv_nh_.param<float>("minimum_steering", minimum_steering_, -50);
      //TODO update using params
      cte_Kp_ = 0.7;
      cte_Kd_ = 1.0;
      cte_Ki_ = 0.0002;
      sub_curvature_ = nh_.subscribe( "vision/curvature", 1,  &pid_control::curvatureCallback,this);
      pub_steering_= nh.advertise<std_msgs::Int16>(nh.resolveName("/manual_control/steering"), 1);
      pub_speed_= nh.advertise<std_msgs::Int16>(nh.resolveName("/manual_control/speed"), 1);
      sub_path_points =nh_.subscribe( "/motion_planner/Path", 1,&pid_control::path_callback,this);
      sub_odom_ = nh_.subscribe( "/odom", 1, &pid_control::odom_callback,this);
      ROS_INFO("Started PID control node.");
    }
    ~pid_control(){}
};

void pid_control::curvatureCallback(const std_msgs::Float32 curvature){
  //TODO - Dormant Now- check if needed
  /*
  //angle based on curvature
  angle = (90.0 - (atan2(1,curvature.data) /(2 * 3.14) * 360.0));
  int DesiredSteering;
  int DesiredSpeed;
  DesiredSteering=steering_Kp_*angle+steering_Kd_*((angle-last_angle)/sampleTime);
  last_angle = angle;
  if (DesiredSteering>maximum_steering_)
    DesiredSteering=maximum_steering_;
  else if (DesiredSteering<minimum_steering_)
    DesiredSteering=minimum_steering_;

  if ((DesiredSteering<1)&&(-1<DesiredSteering))
    DesiredSpeed=maximum_rpm_;
  else
      DesiredSpeed=minimum_steering_+speed_Kp_*(maximum_steering_/abs(DesiredSteering));

  if  (DesiredSpeed>maximum_rpm_)
      DesiredSpeed=maximum_rpm_;

  desired_steering.data=DesiredSteering;
  desired_speed.data=DesiredSpeed;
  pub_steering_.publish(desired_steering);
  pub_speed_.publish(desired_speed);
  */
}
double pid_control::pid_cte_error(double cte){
  if(error_initial == false){
    error_initial = true;
    prev_cte = cte;
  }
  p_cte = cte;
  i_cte += cte;
  d_cte = cte - prev_cte;
  prev_cte = cte;

  //return total error
  return  cte_Kp_*p_cte + cte_Kd_*d_cte + cte_Ki_*i_cte;
}


//Update the path from motion planner to be used by the control node
void pid_control::path_callback(const nav_msgs::Path::ConstPtr& mp_path){
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
void pid_control::odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
  px = msg->pose.pose.position.x;
  py = msg->pose.pose.position.y;
  //TODO How to handle orientation
  tf::Pose pose;
  tf::poseMsgToTF(msg->pose.pose, pose);
  //TODO - check correctness
  psi = tf::getYaw(pose.getRotation());
  std::cout <<"psi "<<psi<<std::endl;
  velocity = msg->twist.twist.linear.x;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "PID_control_node");
  ros::NodeHandle nh;
  pid_control control1(nh);
  pred_path_marker_pub = nh.advertise<visualization_msgs::Marker>("mpc_pred_points", 20);
  ref_path_marker_pub = nh.advertise<visualization_msgs::Marker>("mpc_ref_points", 20);
  //TODO adjust rate
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
    //evaluate the cte and orientation error at origin.
    double cte = coeffs[0];
    double epsi = -atan(coeffs[1]);

    //Invoke PID control and receive steering to turn
    double error = control1.pid_cte_error(cte);
    //total_error is +ve if car is right of ref, and -ve if car left of ref
    // +ve move left 90+angle, -ve move right - 90-angle - suits good
    std_msgs::Int16 steer_value;
    std_msgs::Int16 speed_value;

    //slowdown if the error is high - indicating high turning angle
    speed_value.data = (int)(350) - abs(200*error);

    //100 is multiplying factor as cte will be in m's and small
    //divide by a factor for speed as car will take sharp turns at high speed - May be the value need to be increased
    //speed/300 is used as dividing factor
    steer_value.data  = (int)(90  - (error*100*300/speed_value.data));
    //Limit the min and max

    if (speed_value.data < 50) {
      speed_value.data = 50;
    }
    else if(speed_value.data > 500){
      speed_value.data = 500;
    }
    speed_value.data = -speed_value.data;
    if(steer_value.data > 140){
      steer_value.data = 140;
    }
    else if(steer_value.data < 40){
      steer_value.data = 40;
    }

    ROS_INFO("pos:: (%0.2f,%0.2f) , steer: %d, speed  %d cte : %0.3f, error: %0.3f epsi %0.3f",l_px,l_py,steer_value.data,
                    speed_value.data, cte,error, epsi);

    //Publish steering, throttle - for robot to drive
    control1.pub_steering_.publish(steer_value);
    control1.pub_speed_.publish(speed_value);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
