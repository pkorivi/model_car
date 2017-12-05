/*
 * sample_nodelet_class2.cpp
 *
 *  Created on: 2016/09/18
 *      Author: cryborg21
 */
#include "Vehicle_state.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace fub_motion_planner{
  Vehicle_state::Vehicle_state(){
    ROS_INFO("Vehicle_state Constructor");
  }

  Vehicle_state::~Vehicle_state(){
    ROS_INFO("Vehicle_state Destructor");
  }

  void Vehicle_state::onInit(){
      NODELET_INFO("Vehicle_state - %s", __FUNCTION__);
  }
} // namespace sample_nodelet_ns

PLUGINLIB_EXPORT_CLASS(fub_motion_planner::Vehicle_state, nodelet::Nodelet)
