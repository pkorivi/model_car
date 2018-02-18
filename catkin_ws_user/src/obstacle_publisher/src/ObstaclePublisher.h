/*
 */
#ifndef OBSTACLE_PUBLISHER_H_
#define OBSTACLE_PUBLISHER_H_
#include <nodelet/nodelet.h>
#include "VehiclePath.h"
#include <autonomos_obstacle_msgs/Obstacles.h>
#include <autonomos_obstacle_msgs/Obstacle.h>
#include <ecl/geometry.hpp>
#include <ecl/containers.hpp>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int16.h>

namespace fub_obstacle_publisher{

class ObstaclePublisher : public nodelet::Nodelet{
    public:
        ObstaclePublisher();
        ~ObstaclePublisher();
        virtual void onInit();
    protected:
      VehiclePath m_vehicle_path;
      //Pushlishers

      //Publishers to vizualize obstacle paths
      ros::Publisher obst_path_1;
      ros::Publisher obst_path_2;
      ros::Publisher obst_path_3;

      /** The callback for the timer that triggers the update.
      */
      void callbackTimer(const ros::TimerEvent&);
      void convert_path_to_obst_pos();
      ros::Timer m_timer;
    private:
      //Constants
  };
} // namespace fub_obstacle_publisher

#endif /*  */
