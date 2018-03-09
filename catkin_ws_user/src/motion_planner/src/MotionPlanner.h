/*
 */
#ifndef MOTION_PLANNER_H_
#define MOTION_PLANNER_H_
#include <nodelet/nodelet.h>
#include "VehicleState.h"
#include "VehiclePath.h"
#include <ecl/geometry.hpp>
#include <ecl/containers.hpp>
#include <std_msgs/Int16.h>
#include <pluginlib/class_list_macros.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <autonomos_obstacle_msgs/Obstacles.h>
#include <autonomos_obstacle_msgs/Obstacle.h>
#include <fub_trajectory_msgs/Trajectory.h>
#include <fub_trajectory_msgs/TrajectoryPoint.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <fstream>
#include <cmath>
#include "time.h"
#include "math.h"
#include <vector>

namespace fub_motion_planner{
  /*
    Class for defining the target state for creating trajectory
  */
  class target_state{
  	public:
  		double d_eval;
  		double s_tgt;
  		double v_tgt;
  		double a_tgt;
  		double cost =0;
  		bool evaluated = false;
  		nav_msgs::Path path;
      int id;
      double s_reched = 0;

  		target_state(double s_tgt, double d_eval, double v_tgt, double a_tgt, double cost, int idx){
  			this->d_eval = d_eval;
  			this->s_tgt = s_tgt;
  			this->v_tgt = v_tgt;
  			this->a_tgt = a_tgt;
  			this->cost = cost;
        this->id = idx;
  		}
  };

  class MotionPlanner : public nodelet::Nodelet{
    public:
        MotionPlanner();
        ~MotionPlanner();
        virtual void onInit();
    protected:
      VehicleState m_vehicle_state;
      VehicleState m_prev_vehicle_state;
      VehiclePath m_vehicle_path;
      TfListener m_tf_listener;
      //Pushlishers
      ros::Publisher m_mp_traj;
      ros::Publisher mp_traj1;
      ros::Publisher mp_traj2;
      ros::Publisher mp_traj3;
      ros::Publisher mp_traj4;
      ros::Publisher mp_final_traj;
      //Publishers to vizualize obstacle paths
      ros::Publisher obst_path_1;
      //TODO - remove these in the end
      ros::Publisher obst_path_2;
      ros::Publisher obst_path_3;
      //Publisher to indicate the sub path has been completed
      ros::Publisher sub_path_complete_indicate;
      //Subscriber Lane information
      ros::Subscriber m_subscribe_click_point;
      double prev_d_target=0;
      //index - global variable for creating index to different target states in evaluation
      int index =1;
      /*Function to create trajectory*/
      double create_traj_const_acc_xy_spline_3(VehicleState current_state,VehicleState prev_state, ros::Publisher&  traj_pub, \
               target_state &tgt,tf::Point current_pos_map, FrenetCoordinate curr_frenet_coordi);

      // timer to trigger the execution of the motion planner periodically
      ros::Timer m_timer;
      /** callback for the timer that triggers the replanning */
      void callbackTimer(const ros::TimerEvent&);
      /*Function to perform CollisionCheck*/
      double CollisionCheck(VehicleState current_state,std::vector<double> s_pts,std::vector<double> d_pts, std::vector<double> d_coeffs);
      /*Function to perform initial cost calculation for target state */
      void calc_cost(target_state &tgt, double vel_current, double d_tgt,double prev_d_tgt);
      /*create trajectory returns traj in nav_msgs::Path form, it should be converted to fub trajectory message before conversion */
      void convert_path_to_fub_traj(nav_msgs::Path p, double initial_yaw);
      /*Click Point callback to indicate target lane change*/
      void ClickPointCallback(const geometry_msgs::PointStamped & msg);
      /*Function to convert odom frame coordinate to map frame coordinate */
      tf::Point convert_to_map_coordinate(tf::Point odom_coordi);
    private:
      /*Move into a config file all variables starting with k*/
      const double kLookAheadTime = 5.0;
      //These indicate the number of sample points in look ahead time, these samples are used to make splines in x,y
      const int kNumberOfSamples = 11;
      //Distance when the car is near the target - This could be improved to perfect value
      const double kThresholdDist = 0.25;
      //Safety distance for the car - of car(0.20)  + 0.10m extra for safety
      const double kSafetyDist = 0.30;
      // safety width for car - car width = 0.1, safety 0.5 and this should be added to hald width of obstacle
      const double kSafetyWidth = 0.15;
      //Safety distance in terms of time
      const double kSafetyTimeDiff = 1;
      //Safety Time cost multiple
      const double kSafetyTimeDiffCostMul = 3;
      //Min velocity to consider obstacle moving
      const double kMinMovingObstvel = 0.05;
      //sequence number for the trajectory
      unsigned int gPubSeqNum=0;
      //target d value - lane selection - updated in click point callback
      double gTargetd = 0.17;
  };
} // namespace sample_nodelet_ns

#endif /*  */
