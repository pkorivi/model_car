/*
 * sample_nodelet_class.cpp

 */
#include "MotionPlanner.h"
#include <pluginlib/class_list_macros.h>

namespace fub_motion_planner{
  MotionPlanner::MotionPlanner(){
    //ROS_INFO("MotionPlanner Constructor");
  }

  MotionPlanner::~MotionPlanner(){
    //ROS_INFO("MotionPlanner Destructor");
  }

  void MotionPlanner::onInit(){
      NODELET_INFO("MotionPlanner - %s", __FUNCTION__);
      m_vehicle_state.setup(getNodeHandle());
      m_vehicle_path.setup(getNodeHandle());
      //TODO change execution frequency to a bigger value and also parameter of a config file
      //double execution_frequency = 0.02;
      ros::Duration timerPeriod = ros::Duration(25);
      m_timer = getNodeHandle().createTimer(timerPeriod, &MotionPlanner::callbackTimer, this);
  }

  void MotionPlanner::callbackTimer(const ros::TimerEvent &)
  {
      // create a copy of the vehicle state - we do NOT want these values to
      // change while we are working with them
      // TODO: ensure that data does not change during copying
      VehicleState current_vehicle_state = m_vehicle_state;
      //ROS_INFO("timer, pos x %f",m_vehicle_state.m_vehicle_position[0]);
      //TODO Test all the functions implemented in vehicle state and vehicle path
      /* //Vehicle state stuff works
      ROS_INFO("VS : pose %f %f",current_vehicle_state.m_ego_state_pose.pose.position.x,current_vehicle_state.m_ego_state_pose.pose.position.y);
      ROS_INFO("VS : posi %f %f %f", current_vehicle_state.m_vehicle_position[0],current_vehicle_state.m_vehicle_position[1],current_vehicle_state.m_vehicle_position[2]);
      ROS_INFO("VS: time %f ",current_vehicle_state.m_last_odom_time_stamp_received.toSec());
      ROS_INFO("VS : %f, yaw : %f",current_vehicle_state.m_current_speed_front_axle_center,current_vehicle_state.getVehicleYaw());
      */

      //Vehicle Path
      if (m_vehicle_path.route_path_exists == true) {

        tf::Point a =tf::Point{0.0,1.0,0.0};
        tf::Point b =tf::Point{0.0,2.0,0.0};
        for (size_t i = 0; i < m_vehicle_path.xy_path.size(); i++) {
          ROS_INFO("x,y : (%f,%f)  s,d,k : (%f,%f,%f)",m_vehicle_path.xy_path[i][0], \
                  m_vehicle_path.xy_path[i][1],m_vehicle_path.frenet_path[i].s,\
                  m_vehicle_path.frenet_path[i].d,m_vehicle_path.frenet_path[i].k );
        }
        ROS_INFO("slope %f",m_vehicle_path.slope(a,b));
        ROS_INFO("slope %f",m_vehicle_path.slope(b,a));
        ROS_INFO("distance %f",m_vehicle_path.distance(a,b));
        ROS_INFO("closestWayPoint %d",m_vehicle_path.closestWayPoint(a));
        ROS_INFO("closestWayPoint %d",m_vehicle_path.closestWayPoint(b));
        ROS_INFO("NextWayPoint %d",m_vehicle_path.NextWayPoint(a,0));
        ROS_INFO("NextWayPoint %d",m_vehicle_path.NextWayPoint(b,0));
        FrenetCoordinate f1 = m_vehicle_path.getFenet(a,0);
        ROS_INFO("frenet of %f,%f,%f ",f1.s,f1.d,f1.k);
        FrenetCoordinate f2 = m_vehicle_path.getFenet(b,0);
        ROS_INFO("frenet of %f,%f,%f ",f2.s,f2.d,f2.k);
        tf::Point p1 = m_vehicle_path.getXY(FrenetCoordinate(0.0,0,0));
        ROS_INFO("xy of %f,%f,%f ",p1[0],p1[1],p1[2]);
        tf::Point p2 = m_vehicle_path.getXY(FrenetCoordinate(0.0,0.2,0));
        ROS_INFO("xy of %f,%f,%f ",p2[0],p2[1],p2[2]);
        tf::Point p3 = m_vehicle_path.getXY(FrenetCoordinate(2.0,0.2,0));
        ROS_INFO("xy of %f,%f,%f ",p3[0],p3[1],p3[2]);
        tf::Point p4 = m_vehicle_path.getXY(FrenetCoordinate(2.0,0,0));
        ROS_INFO("xy of %f,%f,%f ",p4[0],p4[1],p4[2]);
    }
    else{
      ROS_INFO("waiting for route path");
    }
  }

} // namespace sample_nodelet_ns

PLUGINLIB_EXPORT_CLASS(fub_motion_planner::MotionPlanner, nodelet::Nodelet)
