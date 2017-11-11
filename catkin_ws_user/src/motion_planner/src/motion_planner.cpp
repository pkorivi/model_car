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
#include "spline.h"
#include "visualization.h"



/*Support Functions*/
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

//TODO remove all the constants - else variables cant be updated
//The variables to hold the global path values
//change the name of this variable to something else-.. the local reference and the global name here is same, It might screw up later
std::vector<double> maps_x;
std::vector<double> maps_y;
//Distance along the map for the points
std::vector<double> maps_s_pts;
//Curvature along the points
const std::vector<double> maps_curv;
//TODO make these global variables to local variables somehow
ros::Publisher disp_ref_path;
//State Variables - Update in odom subscribe node - updated with current values
double curr_px = 0;
double curr_py = 0;
double curr_psi = 0;
double curr_velocity = 0;


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


double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point
  //TODO modify 1000,2000 to adjust to the center of the map
	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,yIt ends at some weird location close to origin for s=0 fir any reference path
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s,\
      const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

//Calculate the equivalent frenet distance for each point and curvature of the point
// https://stackoverflow.com/questions/41144224/calculate-curvature-for-3-points-x-y
// TODO Use the above link to find the curvature of the road at that point with 1pt ahead and 1 pt behind mapping
// or curvature by taking just two forward points
std::vector<double> transform_frenet(const std::vector<double> &maps_x, const std::vector<double> &maps_y){
  //initital distance to itself is zero
  maps_s_pts.push_back(0);
  for(int i = 1; i < maps_x.size(); i++)
  {
    double dist = distance(maps_x[i-1],maps_y[i-1],maps_x[i],maps_y[i]);
    maps_s_pts.push_back(maps_s_pts[i-1]+dist);
  }
  return maps_s_pts;
}


class motion_planner
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
    std::vector<double> ptsx = {1,5,10,15};
    std::vector<double> ptsy = {1,1,1,1};

    //TODO Subscribe to some message to update these values or check if they are needed
    double c_st_angle = 0;
    double c_throttle = 0;

    motion_planner(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
      pub_steering_= nh.advertise<nav_msgs::Path>(nh.resolveName("/motion_planner/Path"), 1);
      //TODO - Finaize on what to receive from behavior layer
      //sub_path_points =nh_.subscribe( "/behavior_layer/Path", 1,&motion_planner::behavior_callback,this);

      //Subscribe to data from the Route Planner
      sub_path_points =nh_.subscribe( "/route_planner/sub_path", 1, &motion_planner::route_planner_callback,this);
      //TODO Subscribe to Predictions '
      //sub_path_points =nh_.subscribe( "/predictions/obstacles", 1, &motion_planner::prediction_callback,this);
      //subscribe to odometry
      sub_odom_ = nh_.subscribe( "/odom", 1, &motion_planner::odom_callback,this);
      ROS_INFO("Started Motion planner node.");
    }
    ~motion_planner(){}
    //TODO Create proper Functions
    void odom_callback(const nav_msgs::Odometry::ConstPtr&);
    //void motion_planner::behavior_callback();
    //Check what and how to pass data
    int cost_of_trajectory();

    void route_planner_callback(const nav_msgs::Path::ConstPtr& msg);
};

//Update the state of the vehicle form Odometry Message
void motion_planner::odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
  curr_px = msg->pose.pose.position.x;
  curr_py = msg->pose.pose.position.y;
  tf::Pose pose;
  tf::poseMsgToTF(msg->pose.pose, pose);
  //TODO - check correctness for the orientation
  curr_psi = tf::getYaw(pose.getRotation());
  curr_velocity = msg->twist.twist.linear.x;
}
//Update the data from behavior layer here
//void motion_planner::behavior_callback(){
//TODO Update this function further for data
//}

void motion_planner::route_planner_callback(const nav_msgs::Path::ConstPtr& msg){
  //Clear old points and lists
  maps_x.clear();
  maps_y.clear();
  maps_s_pts.clear();
  //Receive global path from Route Planner
  for(int i =0; i<msg->poses.size();i++){
    maps_x.push_back(msg->poses.at(i).pose.position.x);
    maps_y.push_back(msg->poses.at(i).pose.position.y);
  }
  std::vector<double>  maps_s = transform_frenet(maps_x,maps_y);
  //print stuff for reference of which path is received
  for (auto i = maps_y.begin(); i != maps_y.end(); ++i)
      std::cout << *i << ' ';
  std::cout << "maps_y"<<std::endl;
  for (auto i = maps_x.begin(); i != maps_x.end(); ++i)
      std::cout << *i << ' ';
  std::cout << "maps_x"<<std::endl;
  for (auto i = maps_s_pts.begin(); i != maps_s_pts.end(); ++i)
      std::cout << *i << ' ';
  std::cout << "maps_s_pts"<<std::endl;

  //see how to pass the publisher here..!!!
  visulize_path(disp_ref_path, maps_x,maps_y, "blue");
}

//void motion_planner::prediction_callback(){
  //TODO Update the function with obstacle data

  //TODO check how to deal with static and dynamic obstacles - How can they be
  //used - Initially static checks need to be done then only dynamic
//}

int motion_planner::cost_of_trajectory(){
//TODO - Update this with function to return the cost of the trajectory
return 0;
}

void create_traj(ros::Publisher&  disp_pred_path){
  tk::spline s;
  std::vector<double> spts;
  std::vector<double> tpts = {0,2,4,6,8};
  std::vector<double> traj_x;
  std::vector<double> traj_y;
  //get current position in frenet frame
  //Adding a condition to perform traj generation when there is path to be followed
  if (maps_s_pts.size()>0) {
    std::cout<<"New evaluation"<<std::endl;
    //estimate trajectory at curent frenet point
    std::vector<double> frenet_val = getFrenet(curr_px,curr_py, curr_psi, maps_x,maps_y);
    for(int i=0;i<5;i++){
      spts.push_back(frenet_val[0] + i*0.3);
      //{0,0.6,1.6,2.4,3.1}
    }
    s.set_points(tpts,spts);    // currently it is required that X is already sorted. evaluating s with respect to time
    for(int i=1;i<5;i++){
      double sv = s(i);
      double dv = 0;
      std::cout<<"s-frame(t,s) : "<<i<<','<<sv;
      std::vector<double> xy = getXY(sv, dv, maps_s_pts,maps_x,maps_y);
      std::cout<<"  (x,y) : "<<xy[0]<<','<<xy[1]<<std::endl;
      traj_x.push_back(xy[0]);
      traj_y.push_back(xy[1]);
    }
    //Publish the predicted path to rviz
    visulize_path(disp_pred_path, traj_x,traj_y, "red");
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_planner_node");
  ros::NodeHandle nh;
  ros::Publisher disp_pred_path;//,ref_path;
  disp_pred_path = nh.advertise<visualization_msgs::Marker>("/pred_path_marker", 20);
  disp_ref_path  = nh.advertise<visualization_msgs::Marker>("/ref_path_marker", 20);
  motion_planner control1(nh);
  //TODO Check the required frequency
  ros::Rate loop_rate(3);
  while(ros::ok()){
    //TODO Update to get a proper trajectory and send to the control unit
    create_traj(disp_pred_path);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
