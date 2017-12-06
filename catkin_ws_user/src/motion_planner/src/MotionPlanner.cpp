/*
 * sample_nodelet_class.cpp
 *
 *  Created on: 2016/09/18
 *      Author: cryborg21
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
      //TODO change execution frequency to a bigger value and also parameter of a config file
      int execution_frequency = 1;
      ros::Duration timerPeriod = ros::Duration(1.0 / execution_frequency);
      m_timer = getNodeHandle().createTimer(timerPeriod, &MotionPlanner::callbackTimer, this);
  }

  void MotionPlanner::callbackTimer(const ros::TimerEvent &)
  {
      // create a copy of the vehicle state - we do NOT want these values to
      // change while we are working with them
      // TODO: ensure that data does not change during copying
      VehicleState current_vehicle_state = m_vehicle_state;
      //ROS_INFO("timer, pos x %f",m_vehicle_state.m_vehicle_position[0]);
  }

} // namespace sample_nodelet_ns

PLUGINLIB_EXPORT_CLASS(fub_motion_planner::MotionPlanner, nodelet::Nodelet)
