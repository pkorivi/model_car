/*
 */
#ifndef MOTION_PLANNER_H_
#define MOTION_PLANNER_H_
#include <nodelet/nodelet.h>
#include "VehicleState.h"
#include "VehiclePath.h"
#include "polyfit.h"
//#include <fub_trajectory_msgs/Trajectory.h>
//#include <fub_trajectory_msgs/TrajectoryPoint.h>
#include <autonomos_obstacle_msgs/Obstacles.h>
#include <autonomos_obstacle_msgs/Obstacle.h>
#include <ecl/geometry.hpp>
#include <ecl/containers.hpp>
#include <geometry_msgs/PointStamped.h>


namespace fub_motion_planner{
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
      ros::Publisher obst_path_2;
      ros::Publisher obst_path_3;
      //Subscriber Lane information
      ros::Subscriber m_subscribe_click_point;
      double prev_d_target=0;
      int index =1;

      void create_traj_spline(VehicleState current_state,VehicleState prev_state, ros::Publisher&  traj_pub, \
              double v_target,double a_target,double d_target,double v_max, double v_min, int polynomial_order);
      void create_traj(VehicleState current_state,VehicleState prev_state, ros::Publisher&  traj_pub, \
              double v_target,double a_target,double d_target,double v_max, double v_min, int polynomial_order);
      void create_traj_const_acc(VehicleState current_state,VehicleState prev_state, ros::Publisher&  traj_pub, \
              double v_target,double a_target,double d_target,double v_max, double v_min, int polynomial_order);
      void create_traj_const_acc_xy_polyeval(VehicleState current_state,VehicleState prev_state, ros::Publisher&  traj_pub, \
              double v_target,double a_target,double d_target,double v_max, double v_min, int polynomial_order);
      double create_traj_const_acc_xy_polyeval_2(VehicleState current_state,VehicleState prev_state, ros::Publisher&  traj_pub, \
              double v_max, double v_min, int polynomial_order, target_state &tgt); //TODO move v_min, v_max, polynomial_order to MotionPlanner.h constants, remove prev state
      double create_traj_const_acc_xy_spline_3(VehicleState current_state,VehicleState prev_state, ros::Publisher&  traj_pub, \
              double v_max, double v_min,int polynomial_order, target_state &tgt);
      /** The callback for the timer that triggers the update.
      */
      void callbackTimer(const ros::TimerEvent&);
      double CollisionCheck(VehicleState current_state,std::vector<double> s_pts,std::vector<double> d_pts, std::vector<double> t_pts, std::vector<double> d_coeffs);
      void calc_cost(target_state &tgt, double vel_current, double d_tgt,double prev_d_tgt);
      void convert_path_to_fub_traj(nav_msgs::Path p, double initial_yaw);
      void ClickPointCallback(const geometry_msgs::PointStamped & msg);
      tf::Point convert_to_map_coordinate(tf::Point odom_coordi);
      // timer triggering our execution // TODO: use WallTimer?
      ros::Timer m_timer;
    private:
      const double kLookAheadTime = 5.0;
      const int kNumberOfSamples = 11; //changing to 11 from 26
      unsigned int gPubSeqNum=0;
      double gTargetd = 0.17;
  };
} // namespace sample_nodelet_ns

#endif /*  */
