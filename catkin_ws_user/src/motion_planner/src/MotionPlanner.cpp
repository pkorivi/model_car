/*
 * sample_nodelet_class.cpp

 */
#include "MotionPlanner.h"
#include <pluginlib/class_list_macros.h>
#include "spline.h"

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
      ros::Duration timerPeriod = ros::Duration(8);
      m_mp_traj = getNodeHandle().advertise<nav_msgs::Path>("/motionplanner/traj_1", 10);
      m_timer = getNodeHandle().createTimer(timerPeriod, &MotionPlanner::callbackTimer, this);
  }

  void MotionPlanner::create_traj(VehicleState current_state){
    tk::spline s;
    tk::spline v;
    //Amax for profiles TODO : Update the Amax based on current velocity
    double acc[] = {-0.2,0,0.2};
    double v_current = current_state.m_current_speed_front_axle_center;
    tf::Point cp = current_state.m_vehicle_position;
    double c_yaw = current_state.getVehicleYaw();
    double v_max = 1.1;//1mps - TODO Velocity limit gathered from the map speed limit
    double v_target = v_max; // TODO Optimal target velocity for driving from behavioral layer- currently set to v_max
    double v_min = 0; // stand still, no negative speeds
    double a_current = 0; // TODO update this value from the odometry info
    std::vector<double> spts;
    std::vector<double> tpts;
    std::vector<double> vpts;
    std::vector<double> acc_pts;
    std::vector<double> traj_x;
    std::vector<double> traj_y;
    //get current position in frenet frame
    //Adding a condition to perform traj generation when there is path to be followed
      std::cout<<"New evaluation"<<std::endl;
      //current point in frenet
      FrenetCoordinate frenet_val =  m_vehicle_path.getFenet(cp,c_yaw);
      //Initial time and
      spts.push_back(frenet_val.s);
      vpts.push_back(v_current);
      tpts.push_back(0);
      acc_pts.push_back(a_current);
      double n_vel = v_current;
      double jerk_val = acc[2]; //slope is (acc[2] - 0)/time to reach there assuming our acceleration chnages form 0-0.2 in 1s
      //Time samples of 100ms each, so for 5 seconds we have 50 samples - TODO this as tunable parameter
      double number_of_samples = 50;
      double t_sample = 5/number_of_samples;
      for(int i=1;i<number_of_samples;i++){
        tpts.push_back(i*t_sample);
        //s = ut+0.5at², //v = u+at   .. t_s - t_Sample
        //a_ref = a_current + a_slope*t_s; v_ref = v_cur + a_ref*t_s, p_ref  = p_current + v_ref*t_sample   :: ref is the value the robot should go to next
        // trapezoidal acceleration /'''''''\  .. if a<a_max and v< v_max-0.1 ;
        //phase of increase acceleration if the current acceleration is less than target profile acceleration
        //TODO incorporate v<v_max stuff if needed
        //TODO check which formula to use for calculation of s. Both work good with slight approximation
        double a_ref, v_ref, s_ref;
        //acceleration = a_slope*t_Sample ; a_Slope = Jerk is change in acceleration from zero by time to change
        //current acc is less than target of profile and velocity didnt reach max
        if(acc_pts[i-1]<acc[2] && vpts[i-1] < (v_max-0.1)){  //TODO 0.2mps will be increased in speed if we make our acceleration 0 from 0.2
          a_ref = acc_pts[i-1] + jerk_val*t_sample;
          //if the value increases over limit, limit it to alimit
          if(a_ref > acc[2]){
            a_ref = acc[2];
          }
          acc_pts.push_back(a_ref);
          v_ref = vpts[i-1] + a_ref*t_sample;// increased velocity is the area of rectangle formed by acceleration and time sample
          vpts.push_back(v_ref);
          //TODO as per paper they use v_ref*t_sample - As the sample size is small, even this can be chosen. Compare both values and choose best possible
          //acceleration is a function of time - This can be simply approximated I think
          //spts.push_back(spts[i-1] + vpts[i-1]*t_sample + 0.5*jerk_val*t_sample*t_sample*t_sample);
          spts.push_back(spts[i-1] + v_ref*t_sample);
        }
        //The constant acceleration phase of trapezoid , keep the velocity to a point where it is slightly below threshold, which is increased while making acceleration to zero
        else if(acc_pts[i-1] == acc[2] && vpts[i-1] < (v_max-0.1)){
          //constant acceleration
          acc_pts.push_back(a_ref);
          v_ref = vpts[i-1] + acc[2]*t_sample;
          vpts.push_back(v_ref);
          spts.push_back(spts[i-1] + vpts[i-1]*t_sample + 0.5*acc[2]*t_sample*t_sample);
          //spts.push_back(spts[i-1] + v_ref*t_sample);
        }
        //acceleration to Zero phase - the last -ve jerk ramp
        else if((vpts[i-1] > (v_max-0.2)) && vpts[i-1] < v_max){
          //acceleration should decrease, thus jerk_val is negative
          a_ref = acc_pts[i-1] - jerk_val*t_sample;
          //if the value increases over limit, limit it to alimit
          if(a_ref <= 0){
            a_ref = 0;
          }
          acc_pts.push_back(a_ref);
          v_ref = vpts[i-1] + a_ref*t_sample;// increased velocity is the area of rectangle formed by acceleration and time sample
          vpts.push_back(v_ref);
          //TODO as per paper they use v_ref*t_sample - As the sample size is small, even this can be chosen. Compare both values and choose best possible
          //acceleration is a function of time - This can be simply approximated I think
          //spts.push_back(spts[i-1] + vpts[i-1]*t_sample + 0.5*jerk_val*t_sample*t_sample*t_sample);
          spts.push_back(spts[i-1] + v_ref*t_sample);
        }
        //Max velocity achieved then drop acceleration to zero
        else if(vpts[i-1]>=v_max){
          acc_pts.push_back(0);
          vpts.push_back(v_max);
          //TODO as per paper they use v_ref*t_sample - As the sample size is small, even this can be chosen. Compare both values and choose best possible
          //acceleration is a function of time - This can be simply approximated I think
          spts.push_back(spts[i-1] + v_max*t_sample);
        }
        //If velocity goes negative - dont drive back wards. Backward driving is not included
        else if(vpts[i-1]<= v_min){
          acc_pts.push_back(0);
          vpts.push_back(v_min);
          //TODO as per paper they use v_ref*t_sample - As the sample size is small, even this can be chosen. Compare both values and choose best possible
          //acceleration is a function of time - This can be simply approximated I think
          spts.push_back(spts[i-1]);
        }
      }
      s.set_points(tpts,spts);    // currently it is required that X is already sorted. evaluating s with respect to time
      v.set_points(tpts,vpts);   // spline for velocity

      nav_msgs::Path m_sampled_traj;
      m_sampled_traj.header.stamp = ros::Time::now();
      m_sampled_traj.header.frame_id = "/map";
      //sample every 0.2s
      for(double i=0;i<=25;i++){
        double s_v = s(0.2*i);
        double d_v = 0;
        tf::Point xy = m_vehicle_path.getXY(FrenetCoordinate(s_v,d_v,0)); //TODO check yaw stuff
        //std::cout<<"  (x,y) : "<<xy[0]<<','<<xy[1]<<', (s,d)'<<s_v<<std::endl;
        std::cout<<"acc : "<<acc_pts[i*2]<<" vel : "<<vpts[i*2]<<" posi : "<<spts[i*2]<<std::endl;
        geometry_msgs::PoseStamped examplePose;
        examplePose.pose.position.x = xy[0];
        examplePose.pose.position.y = xy[1];
        examplePose.pose.position.z = 0;//v(0.25*i); //velocity in z direction
        examplePose.pose.orientation.x = 0.0f;
        examplePose.pose.orientation.y = 0.0f;
        examplePose.pose.orientation.z = 0.0f;

        //push PoseStamped into Path
        m_sampled_traj.poses.push_back(examplePose);
      }
      //Publish as path with velocity in z dimension
      m_mp_traj.publish(m_sampled_traj);
  }

  void MotionPlanner::callbackTimer(const ros::TimerEvent &){
      // create a copy of the vehicle state - we do NOT want these values to
      // change while we are working with them
      // TODO: ensure that data does not change during copying
      VehicleState current_vehicle_state = m_vehicle_state;
      //Vehicle Path
      if (m_vehicle_path.route_path_exists == true) {
        create_traj(current_vehicle_state);
      }
      else{
        ROS_INFO("waiting for route path");
      }
  }

} // namespace sample_nodelet_ns

PLUGINLIB_EXPORT_CLASS(fub_motion_planner::MotionPlanner, nodelet::Nodelet)
