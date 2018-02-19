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
  class obstacle_def{
    public:
      double obst_start;
      double obst_end;
      double obst_cur_s;
      double obst_d;
      double obst_vel;
      double obst_yaw;
      double obst_width;
      double obst_length;
      int obst_id;

      //Constructor
      obstacle_def(double obst_start,double obst_end,double obst_d,double obst_vel, \
        double obst_yaw, int obst_id, double obst_width, double obst_length){
         this->obst_start = obst_start;
         this->obst_end = obst_end;
         this->obst_d = obst_d;
         this->obst_vel = obst_vel;
         this->obst_id = obst_id;
         this->obst_yaw = obst_yaw;
         this->obst_cur_s = obst_start;
         this->obst_width = obst_width;
         this->obst_length = obst_length;
      }

  };

class ObstaclePublisher : public nodelet::Nodelet{
    public:
        ObstaclePublisher();
        ~ObstaclePublisher();
        virtual void onInit();
    protected:
      VehiclePath m_vehicle_path;
      //Pushlishers
      ros::Publisher obstacles_list;
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

      std::vector<obstacle_def> obst_to_publish;
      unsigned int gSeqNum=0;
      double time_period_loop=0;
      //Constants
  };
} // namespace fub_obstacle_publisher

#endif /*  */
