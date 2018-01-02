/*
 * sample_nodelet_class.cpp

 */
#include "MotionPlanner.h"
#include <pluginlib/class_list_macros.h>
#include "spline.h"
#include "math.h"
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "CreateTraj.cpp"
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>


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
      ros::Duration timerPeriod = ros::Duration(0.25);
      m_mp_traj = getNodeHandle().advertise<nav_msgs::Path>("/motionplanner/traj", 10);
      mp_traj1 = getNodeHandle().advertise<nav_msgs::Path>("/motionplanner/traj_1", 10);
      mp_traj2 = getNodeHandle().advertise<nav_msgs::Path>("/motionplanner/traj_2", 10);
      mp_traj3 = getNodeHandle().advertise<nav_msgs::Path>("/motionplanner/traj_3", 10);
      mp_traj4 = getNodeHandle().advertise<nav_msgs::Path>("/motionplanner/traj_4", 10);
      m_timer = getNodeHandle().createTimer(timerPeriod, &MotionPlanner::callbackTimer, this);
  }

  void MotionPlanner::callbackTimer(const ros::TimerEvent &){
      // create a copy of the vehicle state - we do NOT want these values to
      // change while we are working with them
      // TODO: ensure that data does not change during copying
      //std::cout << "VS in cb tmr " <<m_vehicle_state.m_vehicle_position[0]<<" , "<<m_vehicle_state.m_vehicle_position[1]<< '\n';
      VehicleState current_vehicle_state = m_vehicle_state;
      //Vehicle Path
      if (m_vehicle_path.route_path_exists == true) {
        ros::Time t = ros::Time::now();
        //Amax for profiles TODO : Update the Amax based on current velocity
        double acc[] = {0.15,0,-0.2};
        //TODO min_max Update this values from map
        double v_max = 0.6;
        double v_min = 0; // stand still, no negative speeds
        //target values
        //V_ Target indicated by behavioral layer
        double v_target = 0.6;
        //TODO a_tgt and d_tgt - part of matrix
        double a_target = acc[0];
        double d_target = 0.2;
        int polynomial_order = 3;
        //create_traj_spline(current_vehicle_state,mp_traj1,v_target,a_target,d_target,v_max,v_min,polynomial_order);
        create_traj_spline(current_vehicle_state,m_prev_vehicle_state,mp_traj1,v_target,a_target,d_target,v_max,v_min,polynomial_order);
        /*a_target = 0;
        a_target = -0.15;
        v_target = 0;
        create_traj(current_vehicle_state,mp_traj3,v_target,a_target,d_target,v_max,v_min,polynomial_order);
        a_target = -0.3;
        v_target = 0;
        create_traj(current_vehicle_state,mp_traj4,v_target,a_target,d_target,v_max,v_min,polynomial_order);*/
        std::cout<<"elapsed :: "<< (ros::Time::now()-t).toSec()<< '\n';
      }
      else{
        ROS_INFO("waiting for route path");
      }
      //Store the current state to previous planning state
      m_prev_vehicle_state = current_vehicle_state;
  }

} // namespace sample_nodelet_ns

PLUGINLIB_EXPORT_CLASS(fub_motion_planner::MotionPlanner, nodelet::Nodelet)
