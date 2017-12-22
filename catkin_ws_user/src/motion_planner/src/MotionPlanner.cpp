/*
 * sample_nodelet_class.cpp

 */
#include "MotionPlanner.h"
#include <pluginlib/class_list_macros.h>
#include "spline.h"
#include "math.h"
#include "CreateTraj.cpp"

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
      ros::Duration timerPeriod = ros::Duration(2);
      m_mp_traj = getNodeHandle().advertise<nav_msgs::Path>("/motionplanner/traj_1", 10);
      m_timer = getNodeHandle().createTimer(timerPeriod, &MotionPlanner::callbackTimer, this);
  }

  void MotionPlanner::callbackTimer(const ros::TimerEvent &){
      // create a copy of the vehicle state - we do NOT want these values to
      // change while we are working with them
      // TODO: ensure that data does not change during copying
      VehicleState current_vehicle_state = m_vehicle_state;
      //Vehicle Path
      if (m_vehicle_path.route_path_exists == true) {
        ros::Time t = ros::Time::now();
        create_traj(current_vehicle_state);
        std::cout<<"elapsed :: "<< ros::Time::now()-t<< '\n';
      }
      else{
        ROS_INFO("waiting for route path");
      }
  }

} // namespace sample_nodelet_ns

PLUGINLIB_EXPORT_CLASS(fub_motion_planner::MotionPlanner, nodelet::Nodelet)
