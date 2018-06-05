/*
 * sample_nodelet_class.cpp

 */
#include "ObstaclePublisher.h"
#include <pluginlib/class_list_macros.h>
#include "math.h"
#include <vector>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include "nav_msgs/Odometry.h"
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
    time_period_loop = 0.25; //
    ros::Duration timerPeriod = ros::Duration(time_period_loop);
    m_timer = getNodeHandle().createTimer(timerPeriod, &ObstaclePublisher::callbackTimer, this);
    obstacles_list = getNodeHandle().advertise<autonomos_obstacle_msgs::Obstacles>("/obstacles",10);
    int obst_id =0;
    // obst_start_s, obst_end_s, obst_d, obst_vel, obst_yaw wrt to road, obst_id, obst width, obst length
    //Dynamic
    //in oppsite direction
    //obst_to_publish.push_back(obstacle_def(7.0,0.1,-0.2,0.8,0,obst_id++, 0.20,0.25));

    //two dynamic obstacles in same direction
    //obst_to_publish.push_back(obstacle_def(1.0,6,-0.17,0.6,0,obst_id++, 0.15,0.25));
    //obst_to_publish.push_back(obstacle_def(1.0,6,0.17,0.6,0,obst_id++, 0.15,0.25));

    //slow moving in same lane
    /*
    obst_to_publish.push_back(obstacle_def(1.5,20,-0.17,0.5,0,obst_id++, 0.15,0.25));
    obst_to_publish.push_back(obstacle_def(6.0,20,-0.17,0.5,0,obst_id++, 0.15,0.25));
    */

    //set of static obstacles - blocked lane
    /*
    obst_to_publish.push_back(obstacle_def(7.5,7.5,0.27,0.0,0,obst_id++, 0.1,0.1));
    obst_to_publish.push_back(obstacle_def(7.5,7.5,0.20,0.0,0,obst_id++, 0.1,0.1));
    obst_to_publish.push_back(obstacle_def(7.5,7.5,0.13,0.0,0,obst_id++, 0.1,0.1));
    obst_to_publish.push_back(obstacle_def(7.5,7.5,0.05,0.0,0,obst_id++, 0.1,0.1));
    obst_to_publish.push_back(obstacle_def(7.5,7.5,-0.05,0.0,0,obst_id++, 0.1,0.1));
    obst_to_publish.push_back(obstacle_def(7.5,7.5,-0.15,0.0,0,obst_id++, 0.1,0.1));
    obst_to_publish.push_back(obstacle_def(7.5,7.5,-0.26,0.0,0,obst_id++, 0.1,0.1));
    //*/

    //set of static obstacles - parked vehicles protruding into lane
    ///*
    obst_to_publish.push_back(obstacle_def(2.1,2.1,0.25,0.0,0,obst_id++, 0.1,0.1));
    obst_to_publish.push_back(obstacle_def(2.5,2.5,0.25,0.0,0,obst_id++, 0.1,0.1));
    obst_to_publish.push_back(obstacle_def(2.9,2.9,0.27,0.0,0,obst_id++, 0.1,0.1));
    obst_to_publish.push_back(obstacle_def(3.2,3.2,0.27,0.0,0,obst_id++, 0.1,0.1));
    obst_to_publish.push_back(obstacle_def(3.5,3.5,0.30,0.0,0,obst_id++, 0.1,0.1));
    obst_to_publish.push_back(obstacle_def(3.8,3.8,0.30,0.0,0,obst_id++, 0.1,0.1));
    obst_to_publish.push_back(obstacle_def(4.1,4.1,0.25,0.0,0,obst_id++, 0.1,0.1));
    //*/


    //Static
    //obst_to_publish.push_back(obstacle_def(5.0,5.0,0.20,0.0,0,obst_id++,0.1,0.2));

    obst_odom_1 = getNodeHandle().advertise<nav_msgs::Odometry>("/obst_odom_1",10);
  }

  void ObstaclePublisher::callbackTimer(const ros::TimerEvent &){
      ros::Time t_now = ros::Time::now();
      autonomos_obstacle_msgs::Obstacles obst_list;
      obst_list.header.stamp = t_now;
      obst_list.header.frame_id = "map";
      obst_list.header.seq = gSeqNum++;
      //stack the obstacles definition into obstacles
      for (size_t i = 0; i < obst_to_publish.size(); i++) {
        double direction =1;
        if(obst_to_publish[i].obst_end<obst_to_publish[i].obst_start)
          direction = -1;
        autonomos_obstacle_msgs::Obstacle new_obstacle;
        new_obstacle.id = i;
        new_obstacle.header.stamp = t_now;
        new_obstacle.header.frame_id = "map";
        new_obstacle.header.seq = i;
        double var_abs_vel = obst_to_publish[i].obst_vel;
        //Logic to stop the dynamic obstacle suddenly
        if(obst_to_publish[i].obst_cur_s > 5.6){
          var_abs_vel=0;
        }
        obst_to_publish[i].obst_cur_s += time_period_loop*obst_to_publish[i].obst_vel*direction;//s = s+vt;
        if(direction>=0)
          obst_to_publish[i].obst_cur_s = (obst_to_publish[i].obst_cur_s<=obst_to_publish[i].obst_end)?obst_to_publish[i].obst_cur_s:obst_to_publish[i].obst_end;
        else
          obst_to_publish[i].obst_cur_s = (obst_to_publish[i].obst_cur_s>=obst_to_publish[i].obst_end)?obst_to_publish[i].obst_cur_s:obst_to_publish[i].obst_end;
        tf::Point pt = m_vehicle_path.getXY(FrenetCoordinate(obst_to_publish[i].obst_cur_s,obst_to_publish[i].obst_d,0,0));
        // obstacle yaw - wrt to map
        double obstacle_map_yaw = obst_to_publish[i].obst_yaw;
        for (size_t p = 0; p < m_vehicle_path.frenet_path.size(); p++) {
          if (m_vehicle_path.frenet_path[p].s>  obst_to_publish[i].obst_cur_s) {
            obstacle_map_yaw = obstacle_map_yaw+m_vehicle_path.frenet_path[p].th;
            break;
          }
        }
        //Obstacle odometry
        obstacle_map_yaw = (direction>=0)?obstacle_map_yaw:M_PI+obstacle_map_yaw;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(obstacle_map_yaw);
        nav_msgs::Odometry odom;
        odom.header.stamp = t_now;
        odom.header.frame_id = "map";
        odom.pose.pose.position.x = pt[0];
        odom.pose.pose.position.y = pt[1];
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = var_abs_vel;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = 0;
        new_obstacle.odom = odom;

        /*Publish odometry of each obstacle for tracking in simulation*/
        obst_odom_1.publish(odom);

        //TwistWithCovariance
        geometry_msgs::TwistWithCovariance abs_vel;
        abs_vel.twist.linear.x = var_abs_vel;
        abs_vel.twist.linear.y = 0.0;
        new_obstacle.abs_velocity = abs_vel;

        //reference_point - stable point on obstacle
        geometry_msgs::Point32 ref_pt;
        ref_pt.x = pt[0];
        ref_pt.y = pt[1];
        ref_pt.z = 0;
        new_obstacle.reference_point = ref_pt;
        //Bounding box min, max wrt to center - given by odom pose
        geometry_msgs::Point32 pt_min;
        pt_min.x = -obst_to_publish[i].obst_length/2;
        pt_min.y = -obst_to_publish[i].obst_width/2;
        pt_min.z = 0;
        geometry_msgs::Point32 pt_max;
        pt_max.x = obst_to_publish[i].obst_length/2;
        pt_max.y = obst_to_publish[i].obst_width/2;
        pt_max.z = 0.1;
        new_obstacle.bounding_box_min = pt_min;
        new_obstacle.bounding_box_max = pt_max;

        new_obstacle.classification = 5;
        new_obstacle.classification_certainty = 0.9;
        new_obstacle.first_observed = t_now;
        new_obstacle.last_observed = t_now;

        obst_list.obstacles.push_back(new_obstacle);
      }//end of for loop

      obstacles_list.publish(obst_list);

  }


}
PLUGINLIB_EXPORT_CLASS(fub_obstacle_publisher::ObstaclePublisher, nodelet::Nodelet)
