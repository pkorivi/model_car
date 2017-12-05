/*
 * sample_nodelet_class.cpp
 *
 *  Created on: 2016/09/18
 *      Author: cryborg21
 */
#include "motion_planner.h"
#include <pluginlib/class_list_macros.h>

namespace fub_motion_planner{
  motion_planner::motion_planner(){
    ROS_INFO("motion_planner Constructor");
  }

  motion_planner::~motion_planner(){
    ROS_INFO("motion_planner Destructor");
  }

  void motion_planner::onInit(){
      NODELET_INFO("motion_planner - %s", __FUNCTION__);
  }
} // namespace sample_nodelet_ns

PLUGINLIB_EXPORT_CLASS(fub_motion_planner::motion_planner, nodelet::Nodelet)
