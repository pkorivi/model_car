#ifndef VEHICLE_PATH_CLASS_H_
#define VEHICLE_PATH_CLASS_H_

#include <nodelet/nodelet.h>
#include <nav_msgs/Odometry.h>
#include "math.h"
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/tf.h>

namespace fub_motion_planner{
  class FrenetCoordinate{
  public:
    double s; //lenth of the road along the center line
    double d; //lateral distance wrt center of lane/road
    double k; //curvature
    FrenetCoordinate(double s, double d, double k){
      this->s = s;
      this->d = d;
      this->k = k;
    }
  };
  class VehiclePath{
    public:
      VehiclePath();
      ~VehiclePath();
      void setup(ros::NodeHandle & nh);
      FrenetCoordinate getFenet(tf::Point xy_pt, double theta);
      tf::Point getXY(FrenetCoordinate frenet_pt);
      //creates a frenet coordinate frame using the m_path and fills up frenet_path
      void transformToXYandFrenet();
      double distance(tf::Point a, tf::Point b){
        return sqrt((b[0]-a[0])*(b[0]-a[0]) + (b[1]-a[1])*(b[1]-a[1]));
      }
      double slope(tf::Point a, tf::Point b){
        return atan2((b[1]-a[1]),(b[0]-a[0]));
      }
    //TODO change to protected or private
    public:
      void RoutePlannerCallback(const nav_msgs::Path & msg);
      //returns slope given two points in radians

      /*returns index of he closest way point to given point*/
      int closestWayPoint(tf::Point pt);
      /*returns index of he next way point to given point*/
      int NextWayPoint(tf::Point pt,double theta);
      double calc_curvature(tf::Point pts0,tf::Point pts1,tf::Point pts2);
    public:
      //Currently planned path
      nav_msgs::Path m_path;
      bool route_path_exists=false;
      //path in points only
      std::vector<tf::Point> xy_path;
      //Frenet Path
      std::vector<FrenetCoordinate> frenet_path;
    private:
      ros::Subscriber m_subscribe_route_planner;
  };
}
#endif /*  vehicle path*/
