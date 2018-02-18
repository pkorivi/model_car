/*
 * sample_nodelet_class.cpp

 */
#include "ObstaclePublisher.h"
#include <pluginlib/class_list_macros.h>
#include "math.h"
#include <vector>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "time.h"
#include <autonomos_obstacle_msgs/Obstacles.h>
#include <autonomos_obstacle_msgs/Obstacle.h>
#include <fstream>
#include <cmath>
namespace fub_obstacle_publisher{
  ObstaclePublisher::ObstaclePublisher(){
    //Currently nothing
  }
  ObstaclePublisher::~ObstaclePublisher(){
    //Shoudl do something here in future
  }
  void ObstaclePublisher::onInit(){
    NODELET_INFO("MotionPlanner - %s", __FUNCTION__);
    m_vehicle_path.setup(getNodeHandle());
    ros::Duration timerPeriod = ros::Duration(1);
    m_timer = getNodeHandle().createTimer(timerPeriod, &ObstaclePublisher::callbackTimer, this);
  }

  void ObstaclePublisher::callbackTimer(const ros::TimerEvent &){
      std::cout <<" debug " <<m_vehicle_path.frenet_path[4].s<<'\n';
  }


}
PLUGINLIB_EXPORT_CLASS(fub_obstacle_publisher::ObstaclePublisher, nodelet::Nodelet)
