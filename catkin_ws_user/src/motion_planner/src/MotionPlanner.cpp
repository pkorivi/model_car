/*
 * sample_nodelet_class.cpp

 */
#include "MotionPlanner.h"
#include <pluginlib/class_list_macros.h>
#include "spline.h"
#include "math.h"

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
    double acc[] = {0.2,0,-0.2};
    tf::Point cp = current_state.m_vehicle_position;
    //min_max
    double v_max = 1.1;//1mps - TODO Velocity limit gathered from the map speed limit
    double v_min = 0; // stand still, no negative speeds
    //target values
    double v_target = 0.6; // TODO Optimal target velocity for driving from behavioral layer- currently set to v_max
    double a_target = acc[0];
    //current values
    double v_current = current_state.m_current_speed_front_axle_center;
    v_current = 0.27; //TODO remove it after testing
    double a_current = 0.19; // TODO update this value from the odometry info
    double c_yaw = current_state.getVehicleYaw();
    //TODO Add condition to skip if v_current > v_target and a_target > 0
    std::vector<double> spts;
    std::vector<double> tpts;
    std::vector<double> vpts;
    std::vector<double> acc_pts;
    std::vector<double> traj_x;
    std::vector<double> traj_y;
    double a_ref, v_ref, s_ref;
    enum AccStates { TO_REQ, CONSTANT, TO_ZERO,ZERO};
    AccStates c_acc_phase = TO_REQ;
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
    // if cureent is greater than target then acceleration will decrease and this variable is -ve
    double acc_inc_dec = (a_current>a_target?-1:1);
    //jerk: assuming it takes 1s to reach a_target from zero. Sign of jerk is computed from current acceleration
    //If current is greter than target then slope is negative, jerk is negative
    double jerk_val = fabs(a_target==0?0.2:a_target)*acc_inc_dec;
    //to come to zero acc, it should decrease if it is greater than zero and increase if its less than zero
    //TODO zero stuff here- something can be messy
    double to_zero_acc_inc_dec = (a_target>0?-1:1);
    double to_zero_jerk_val = fabs(a_target==0?0.2:a_target)*to_zero_acc_inc_dec;

    //TODO update jer such that it takes 1s to reach from current acceleration to the target acceleration
    //Time samples of 100ms each, so for 5 seconds we have 50 samples - TODO this as tunable parameter
    double number_of_samples = 50;
    double t_sample = 5/number_of_samples;
    //It is the change in velocity if the acceleration is made to zero from current acceleration with constannt jerk - 0.5*a*t & t is always positive
    double v_change=(0.5*a_current*fabs(a_current/jerk_val));
    for(int i=1;i<number_of_samples;i++){
      //std::cout << "vcur : " << vpts[i-1]<< " acur :"<< acc_pts[i-1] <<"  vtgt "<<v_target<<" a_tgt : "<<a_target << " vch "<<v_change <<'\n';
      //std::cout << "abs v : " << fabs(v_target-vpts[i-1])<< " abs a : " <<fabs(acc_pts[i-1]-acc[2]);
      tpts.push_back(i*t_sample);
      switch (c_acc_phase) {
        case TO_REQ: {
          //std::cout << "  :  to target" << '\n';
          //achieve target acceleration

          a_ref = acc_pts[i-1] + jerk_val*t_sample;
          //Acceleration is increasing - dont let it increase beyond target
          if(acc_inc_dec == 1){
            if(a_ref > a_target){
              a_ref = a_target;
            }
          }
          //Acceleration is decreasing - dont let it decrease beyond target
          else{
            if(a_ref < a_target){
              a_ref = a_target;
            }
          }

          acc_pts.push_back(a_ref);
          v_ref = vpts[i-1] + a_ref*t_sample;// increased velocity is the area of rectangle formed by acceleration and time sample
          // Bound the velocity
          if (v_ref > v_max)
            v_ref = v_max;
          else if (v_ref<v_min)
            v_ref = v_min;
          vpts.push_back(v_ref);
          //TODO as per paper they use v_ref*t_sample - As the sample size is small, even this can be chosen. Compare both values and choose best possible
          //acceleration is a function of time - This can be simply approximated I think
          //spts.push_back(spts[i-1] + vpts[i-1]*t_sample + 0.5*jerk_val*t_sample*t_sample*t_sample);
          spts.push_back(spts[i-1] + v_ref*t_sample);
          //v_change is area of the triangle formed by the increase acceleration and time - 0.5at; jerk = acceleration/time => time = current_acceleration/jerk. time is always +ve
          v_change = (0.5*a_ref*fabs(a_ref/jerk_val));

          //If the change is less than changeable move to decreaase phase
          if((fabs(v_target-v_ref) < fabs(v_change))){
            c_acc_phase = TO_ZERO;
          }
          //if target acc is achieved
          else if(fabs(a_ref-a_target) == 0){
            c_acc_phase = CONSTANT;
          }

          break;
        }
        case CONSTANT: {
          //std::cout << "  :  const acc" << '\n';
          //constant acceleration
          acc_pts.push_back(a_target);
          v_ref = vpts[i-1] + a_target*t_sample;
          // Bound the velocity
          if (v_ref > v_max)
            v_ref = v_max;
          else if (v_ref<v_min)
            v_ref = v_min;
          vpts.push_back(v_ref);

          spts.push_back(spts[i-1] + vpts[i-1]*t_sample + 0.5*a_target*t_sample*t_sample);
          //If the velocity to target is less than vchange, skip to zero acceleration phase
          if((fabs(v_target-v_ref) <= fabs(v_change))){
            c_acc_phase = TO_ZERO;
          }
          break;
        }
        case TO_ZERO: {
          //std::cout << "  :  to zero" << '\n';
          //If not zero then make it to zero. If zero keep going
          if(a_ref != 0){
            a_ref = acc_pts[i-1] + to_zero_jerk_val*t_sample;
            //Acceleration is increasing - dont let it increase beyond zero
            if(to_zero_acc_inc_dec == 1){
              if(a_ref > 0){
                a_ref = 0;
              }
            }
            //Acceleration is decreasing - dont let it decrease beyond zero
            else{
              if(a_ref < 0){
                a_ref = 0;
              }
            }
          }
          acc_pts.push_back(a_ref);
          v_ref = vpts[i-1] + a_ref*t_sample;// increased velocity is the area of rectangle formed by acceleration and time sample
          // Bound the velocity
          if (v_ref > v_max)
            v_ref = v_max;
          else if (v_ref<v_min)
            v_ref = v_min;
          vpts.push_back(v_ref);
          //TODO as per paper they use v_ref*t_sample - As the sample size is small, even this can be chosen. Compare both values and choose best possible
          //acceleration is a function of time - This can be simply approximated I think
          //spts.push_back(spts[i-1] + vpts[i-1]*t_sample + 0.5*jerk_val*t_sample*t_sample*t_sample);
          spts.push_back(spts[i-1] + v_ref*t_sample);
          break;
        }
        default: {
          std::cout << "Oops something is wrong" << '\n';
        }
      }
    }//for loop
    std::cout << "v_change :" << v_change << '\n';
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

  }//end of create trajectory

/*
  void MotionPlanner::create_traj(VehicleState current_state){
    tk::spline s;
    tk::spline v;
    //Amax for profiles TODO : Update the Amax based on current velocity
    double acc[] = {0.2,0,-0.2};
    double v_current = current_state.m_current_speed_front_axle_center;
    //v_current = 0.5; //TODO remove it after testing
    tf::Point cp = current_state.m_vehicle_position;
    double c_yaw = current_state.getVehicleYaw();
    double v_max = 1.1;//1mps - TODO Velocity limit gathered from the map speed limit
    double v_target = 0; // TODO Optimal target velocity for driving from behavioral layer- currently set to v_max
    double v_min = 0; // stand still, no negative speeds
    double a_current = 1.05; // TODO update this value from the odometry info
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
      //TODO update jer such that it takes 1s to reach from current acceleration to the target acceleration
      //Time samples of 100ms each, so for 5 seconds we have 50 samples - TODO this as tunable parameter
      double number_of_samples = 50;
      double t_sample = 5/number_of_samples;
      //It is the change in velocity if the acceleration is made to zero from current acceleration with constannt jerk
      double v_change=0;
      for(int i=1;i<number_of_samples;i++){
        std::cout << "vcur : " << vpts[i-1]<< " acur :"<< acc_pts[i-1] <<"  vtgt "<<v_target<<" a_tgt : "<<acc[2] << " vch "<<v_change <<'\n';
        std::cout << "abs v" << fabs(v_target-vpts[i-1])<< " abs a" <<fabs(acc_pts[i-1]-acc[2])<<'\n';
        tpts.push_back(i*t_sample);
        //s = ut+0.5at², //v = u+at   .. t_s - t_Sample
        //a_ref = a_current + a_slope*t_s; v_ref = v_cur + a_ref*t_s, p_ref  = p_current + v_ref*t_sample   :: ref is the value the robot should go to next
        // trapezoidal acceleration /'''''''\  .. if a<a_max and v< v_target-0.1 ;
        //TODO incorporate v<v_target & check such that acceleration profiles are chosen only in this case and const/ decceleration are chosen for
        //TODO check which formula to use for calculation of s. Both work good with slight approximation

        //acceleration = a_slope*t_Sample ; a_Slope = Jerk is change in acceleration from zero by time to change
        //current acc is less than target of profile and velocity didnt reach max
        //v_change indicates the increase/decrease in velocity while the acceleration changes to zero from curent value
        //TODO check how this works if aceleration decreases from 1mpss to -3mpss then to 0 or from 3mpss to 2mpss
        //Phase 1 of trapezoid where acceleration reaches +,-ve value
        //Till acceleration difference magnitude exists &&
        if( (fabs(acc_pts[i-1]-acc[2]) >0) && (fabs(v_target-vpts[i-1]) > fabs(v_change)) ){
          std::cout << "ramp up" << '\n';
          a_ref = acc_pts[i-1] + jerk_val*t_sample;
          //if the value increases over limit, limit it to alimit either positive or negative
          if(fabs(a_ref) > fabs(acc[2])){
            a_ref = acc[2];
          }
          acc_pts.push_back(a_ref);
          v_ref = vpts[i-1] + a_ref*t_sample;// increased velocity is the area of rectangle formed by acceleration and time sample
          vpts.push_back(v_ref);
          //TODO as per paper they use v_ref*t_sample - As the sample size is small, even this can be chosen. Compare both values and choose best possible
          //acceleration is a function of time - This can be simply approximated I think
          //spts.push_back(spts[i-1] + vpts[i-1]*t_sample + 0.5*jerk_val*t_sample*t_sample*t_sample);
          spts.push_back(spts[i-1] + v_ref*t_sample);
          //v_change is area of the triangle formed by the increase acceleration and time - 0.5at; jerk = acceleration/time => time = acceleration/jerk
          v_change = (0.5*a_ref*acc[2]/jerk_val);  // TODO replace acc[2] with target acceleration everwhere
        }
        //The constant acc/dec phase of trapezoid , keep the velocity to a point where it is slightly below threshold, which is increased while making acceleration to zero
        else if(acc_pts[i-1] == acc[2] && (fabs(v_target-vpts[i-1]) > fabs(v_change)) ){
          std::cout << "const acc" << '\n';
          //constant acceleration
          acc_pts.push_back(acc[2]);
          v_ref = vpts[i-1] + acc[2]*t_sample;
          vpts.push_back(v_ref);
          spts.push_back(spts[i-1] + vpts[i-1]*t_sample + 0.5*acc[2]*t_sample*t_sample);
          //spts.push_back(spts[i-1] + v_ref*t_sample);
        }
        //acceleration to Zero phase - the last -ve jerk ramp
        else if((fabs(v_target-vpts[i-1]) <= fabs(v_change)) && fabs(v_target - vpts[i-1]) >=0 ){
          std::cout << "ramp down" << '\n';
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
        //Target velocity achieved then drop acceleration to zero
        //TODO change this condition, the max may be slightly bigger or smaller than vreq, or add condition above to do the chekc
        else if(vpts[i-1]==v_target){
          std::cout << "v == vtgt" << '\n';
          acc_pts.push_back(0);
          vpts.push_back(v_target);
          //TODO as per paper they use v_ref*t_sample - As the sample size is small, even this can be chosen. Compare both values and choose best possible
          //acceleration is a function of time - This can be simply approximated I think
          spts.push_back(spts[i-1] + v_target*t_sample);
        }
        //If velocity goes negative - dont drive back wards. Backward driving is not included
        else if(vpts[i-1]<= v_min){
          std::cout << "v < vmin" << '\n';
          acc_pts.push_back(0);
          vpts.push_back(v_min);
          //TODO as per paper they use v_ref*t_sample - As the sample size is small, even this can be chosen. Compare both values and choose best possible
          //acceleration is a function of time - This can be simply approximated I think
          spts.push_back(spts[i-1]);
        }
        //If velocity goes beyond max value then limit velocity to vmax
        else if(vpts[i-1]>= v_max){
          std::cout << "v > vmax" << '\n';
          acc_pts.push_back(0);
          vpts.push_back(v_max);
          //TODO as per paper they use v_ref*t_sample - As the sample size is small, even this can be chosen. Compare both values and choose best possible
          //acceleration is a function of time - This can be simply approximated I think
          spts.push_back(spts[i-1]);
        }
      }//for loop
      std::cout << "v_change :" << v_change << '\n';
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
  }*/

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
