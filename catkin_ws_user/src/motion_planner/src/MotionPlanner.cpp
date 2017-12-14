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
    double acc[] = {-0.2,0,0.2};
    double c_vel = current_state.m_current_speed_front_axle_center;
    tf::Point cp = current_state.m_vehicle_position;
    double c_yaw = current_state.getVehicleYaw();
    double v_max = 1.3;//1mps
    double v_min = 0; // stand still, no negative speeds
    std::vector<double> spts;
    std::vector<double> tpts = {0,1,2,3,4,5};
    std::vector<double> vpts;
    std::vector<double> traj_x;
    std::vector<double> traj_y;
    //get current position in frenet frame
    //Adding a condition to perform traj generation when there is path to be followed
      std::cout<<"New evaluation"<<std::endl;
      //current point in frenet
      FrenetCoordinate frenet_val =  m_vehicle_path.getFenet(cp,c_yaw);
      spts.push_back(frenet_val.s);
      vpts.push_back(c_vel);
      //TODO remove this: 6 points in time, thus 5 Durations of 1s. for loop
      for(int i=1;i<tpts.size();i++){
        //s = ut+0.5at², //v = u+at
        double n_vel = vpts[i-1] + acc[2]*1; //TODO - chnage time based on further assumtpions or equations for continious values
        if(n_vel<= v_max && n_vel > 0){
          spts.push_back(spts[i-1] + vpts[i-1]*1 + 0.5*acc[2]*1*1 );
          vpts.push_back(n_vel);
        }
        else if(n_vel> v_max){
          n_vel = v_max;
          //TODO sample spts properly, this will cause the s to be reachable only with more than vmax
          spts.push_back(spts[i-1] + vpts[i-1]*1 + 0.5*acc[2]*1*1 );
          vpts.push_back(n_vel);
        }
        else{
          //same position -TODO improve to see how much distance travelled before stopping or know that with some equation later
          spts.push_back(spts[i-1]);
          vpts.push_back(0);
        }
      }
      s.set_points(tpts,spts);    // currently it is required that X is already sorted. evaluating s with respect to time
      v.set_points(tpts,vpts);   // spline for velocity

      nav_msgs::Path m_sampled_traj;
      m_sampled_traj.header.stamp = ros::Time::now();
      m_sampled_traj.header.frame_id = "/map";
      //0-10, sample every 0.25s
      for(double i=0;i<21;i++){
        double s_v = s(0.25*i);
        double d_v = 0;
        //std::cout<<"s-frame(t,s) : "<<i<<','<<sv;
        tf::Point xy = m_vehicle_path.getXY(FrenetCoordinate(s_v,d_v,0)); //TODO check yaw stuff
        std::cout<<"  (x,y) : "<<xy[0]<<','<<xy[1]<<std::endl;
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
