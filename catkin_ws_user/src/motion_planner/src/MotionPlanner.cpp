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
#include "Eigen-3.3/Eigen/LU"
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "time.h"
#include "polyfit.h"
#include <autonomos_obstacle_msgs/Obstacles.h>
#include <autonomos_obstacle_msgs/Obstacle.h>
//#include "CreateTraj2.cpp"
#include "CreateTraj3.cpp"
#include "CollisionCheck.cpp"
#include <fstream>
#include <cmath>

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
      ros::Duration timerPeriod = ros::Duration(5.00);
      m_mp_traj = getNodeHandle().advertise<nav_msgs::Path>("/motionplanner/traj", 11);
      mp_traj1 = getNodeHandle().advertise<nav_msgs::Path>("/motionplanner/traj_1", 1);
      mp_traj2 = getNodeHandle().advertise<nav_msgs::Path>("/motionplanner/traj_2", 1);
      mp_traj3 = getNodeHandle().advertise<nav_msgs::Path>("/motionplanner/traj_3", 1);
      mp_traj4 = getNodeHandle().advertise<nav_msgs::Path>("/motionplanner/traj_4", 1);
      m_timer = getNodeHandle().createTimer(timerPeriod, &MotionPlanner::callbackTimer, this);
      //Initialize obstacle publishers
      obst_path_1 = getNodeHandle().advertise<nav_msgs::Path>("/obstacle_path1", 10);
      obst_path_2 = getNodeHandle().advertise<nav_msgs::Path>("/obstacle_path2", 10);
      obst_path_3 = getNodeHandle().advertise<nav_msgs::Path>("/obstacle_path3", 10);
  }


  void MotionPlanner::calc_cost(target_state &tgt, double vel_current, double d_tgt,double prev_d_tgt){
  	double vel_achvd = vel_current + tgt.a_tgt*kLookAheadTime;
  	if(tgt.a_tgt >=0)
  		vel_achvd = (vel_achvd>tgt.v_tgt)?tgt.v_tgt:vel_achvd;
  	else
  		vel_achvd = (vel_achvd<tgt.v_tgt)?tgt.v_tgt:vel_achvd;
  	//TODO Add S term
  	tgt.cost += fabs(vel_achvd- tgt.v_tgt) + fabs(tgt.a_tgt) + \
  							(fabs(d_tgt -tgt.d_eval)*8)/10 + (fabs(prev_d_tgt -tgt.d_eval)*2)/10; //80% weightage to maintaining target value , 20% weightage to change wrt old path
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
        clock_t tStart = clock();
        std::vector<target_state> final_states;
        /* TODO - Add the prediction matrix here after adjusting the weights. Code in test_code.cpp
        */
        //Amax for profiles TODO : Update the Amax based on current velocity
        std::vector<double> d_ranges = {-0.25,-0.17,-0.1,0,0.1,0.17,0.25};
        std::vector<double> acc_prof = {0.2,0.1,0,-0.1,-0.2,-0.4,-0.6, -0.8};
        //TODO min_max Update this values from map
        double v_max = 1.0;
        double v_min = 0; // stand still, no negative speeds
        //target values
        //V_ Target indicated by behavioral layer
        double vel_target = 1.0;
        //TODO a_tgt and d_tgt - part of matrix
        double a_target = acc_prof[0];
        double d_target = 0.0;
        int polynomial_order = 4;
        double vel_current = current_vehicle_state.m_current_speed_front_axle_center;
        double s_target = m_vehicle_path.frenet_path.back().s;

        if(vel_current<=vel_target){
      		//std::cout << "acceleration" << '\n';
      		for(int j=0; j<=2;j++){
      			for(auto d_eval : d_ranges){
              //(D,S,V,A,COST)
      				target_state tgt(s_target,d_eval,vel_target,acc_prof[j], 0,index++);
      				calc_cost(tgt, vel_current, d_target, prev_d_target);
      				std::cout.width(5);
      				final_states.push_back(tgt);
      				//std::cout << tgt.cost<< "   ";
      			}
      			//std::cout <<'\n';
      		}
      	}
      	else {
      		//std::cout << "Deceleration" << '\n';
      		for(int j=3; j<=5;j++){
      			for(auto d_eval : d_ranges){
              //(D,S,V,A,COST)
      				target_state tgt(s_target,d_eval,vel_target,acc_prof[j], 0,index++);
      				calc_cost(tgt, vel_current, d_target, prev_d_target);
      				std::cout.width(5);
      				final_states.push_back(tgt);
      				//std::cout << tgt.cost<< "   ";
      			}
      			//std::cout <<'\n';
      		}
      	}
      	vel_target =0;
        //std::cout<<"Stopping profiles" <<'\n';
      	for(int j=3; j<=7;j++){
      		for(auto d_eval : d_ranges){
            //(D,S,V,A,COST) extra cost for going to zero
      			target_state tgt(s_target,d_eval,vel_target,acc_prof[j], 2,index++);
      			calc_cost(tgt, vel_current, d_target, prev_d_target);
      			std::cout.width(5);
      			final_states.push_back(tgt);
      			//std::cout << tgt.cost<< "   ";
      		}
      		//std::cout <<'\n';
      	}
        //Re initialize for next cycle
        index =1;

        sort( final_states.begin(),final_states.end(), [ ](const target_state& ts1, const target_state& ts2){
      				return ts1.cost < ts2.cost;});
      	//std::cout << "id "<<final_states.front().id<<" cost "<<final_states.front().cost << '\n';
      	while(final_states.front().evaluated != true){
      		//TODO change to insertion sort - this vector is almost sorted
      		//create_traj(final_states.front());
          double cost_val = create_traj_const_acc_xy_spline_3(current_vehicle_state,m_prev_vehicle_state,mp_traj1,v_max,v_min,polynomial_order, final_states.front());
          //double cost_val = create_traj_const_acc_xy_polyeval_2(current_vehicle_state,m_prev_vehicle_state,mp_traj1,v_max,v_min,polynomial_order, final_states.front());
          //std::cout << "cost" <<cost_val <<'\n';
          std::cout <<" ID: "<<final_states.front().id <<" cost :  " << final_states.front().cost<< "  "<< final_states.front().evaluated<< '\n';
      		sort( final_states.begin(),final_states.end(), [ ](const target_state& ts1, const target_state& ts2){
         				return ts1.cost < ts2.cost;});
      		//std::cout << "id "<<final_states.front().id<<" cost "<<final_states.front().cost << '\n';
      	}

        //(D,S,V,A,COST)
        //target_state tgt(m_vehicle_path.frenet_path.back().s,d_target,v_target,a_target,0);
        //calc_cost(tgt, current_vehicle_state.m_current_speed_front_axle_center, d_target, prev_d_target);
        //create_traj_spline(current_vehicle_state,mp_traj1,v_target,a_target,d_target,v_max,v_min,polynomial_order);
        //create_traj_const_acc(current_vehicle_state,m_prev_vehicle_state,mp_traj2,v_target,a_target,d_target,v_max,v_min,polynomial_order);
        //double cost_val = create_traj_const_acc_xy_polyeval_2(current_vehicle_state,m_prev_vehicle_state,mp_traj1,v_target,a_target,d_target,v_max,v_min,polynomial_order, tgt);
        //double cost_val = create_traj_const_acc_xy_polyeval_2(current_vehicle_state,m_prev_vehicle_state,mp_traj1,v_max,v_min,polynomial_order, tgt);
        //final_states.push_back(tgt);
        nav_msgs::Path p1 = final_states.front().path;
        mp_traj2.publish(p1);
        prev_d_target = final_states.front().d_eval;
        std::cout <<" Final Published ID: "<<final_states.front().id <<" cost :  " << final_states.front().cost<< "  "<< final_states.front().evaluated<< '\n';
        std::cout << final_states.front().path.poses[0].pose.position.x << '\n';
        std::cout << final_states.front().path.poses[0].pose.position.y << '\n';
        ROS_INFO("Time taken: %f", (double)(clock() - tStart)/CLOCKS_PER_SEC);
      }
      else{
        ROS_INFO("waiting for route path");
      }
      //Store the current state to previous planning state
      m_prev_vehicle_state = current_vehicle_state;
  }

} // namespace sample_nodelet_ns

PLUGINLIB_EXPORT_CLASS(fub_motion_planner::MotionPlanner, nodelet::Nodelet)
