
namespace fub_motion_planner{

void MotionPlanner::create_traj(VehicleState current_state){
  //TODO
  /*
  Incorporate checks that +ve acc are chosen only when vtgt > vcur
  Improve execution time - its at 3ms for one traj now
  Reduce variables, Use config files for some variables
  Incorporate distance travelled as a parameter, currently if the min dist to goal is less nothing happens
  Use pointers and avoid variables copies when ever possible
  return a score for each trajectory here after testing the obstacle avoidance etc
  */
  tk::spline s;
  tk::spline v;
  tk::spline a;
  tk::spline d;
  //Amax for profiles TODO : Update the Amax based on current velocity
  double acc[] = {0.2,0,-0.2};
  tf::Point cp = current_state.m_vehicle_position;
  //min_max
  double v_max = 1.1;//1mps - TODO Velocity limit gathered from the map speed limit
  double v_min = 0; // stand still, no negative speeds
  //target values
  double v_target = 1.1; // TODO Optimal target velocity for driving from behavioral layer- currently set to v_max
  double a_target = acc[0];
  double d_target = 0.2;
  //current values
  double v_current = current_state.m_current_speed_front_axle_center;
  v_current = 0.0; //TODO remove it after testing
  double a_current = 0.0; // TODO update this value from the odometry info
  double c_yaw = current_state.getVehicleYaw();
  //TODO Add condition to skip if v_current > v_target and a_target > 0
  std::vector<double> spts;
  std::vector<double> dpts;
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
  dpts.push_back(frenet_val.d);
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
  double number_of_samples = 25;
  double t_sample = (5*1.0)/number_of_samples;
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
  //std::cout << "v_change :" << v_change << '\n';
  s.set_points(tpts,spts);    // currently it is required that X is already sorted. evaluating s with respect to time
  v.set_points(tpts,vpts);   // spline for velocity
  a.set_points(tpts,acc_pts);

  //for change in d -
  //TODO add d to maintain current raidus of curvature

  dpts.push_back(d_target);
  dpts.push_back(d_target);
  dpts.push_back(d_target);
  d.set_points(std::vector<double> {0,4.6,4.8,5},dpts);

  nav_msgs::Path m_sampled_traj;
  m_sampled_traj.header.stamp = ros::Time::now();
  m_sampled_traj.header.frame_id = "/map";
  //sample every 0.2s
  for(double i=0;i<25;i++){
    double t_pt = 0.2*i;//time
    double s_val = s(t_pt);
    double d_val = d(t_pt);
    tf::Point xy = m_vehicle_path.getXY(FrenetCoordinate(s_val,d_val,0)); //TODO check yaw stuff
    //std::cout<<"  (x,y) : "<<xy[0]<<','<<xy[1]<<std::endl;//',    (s,d)'<<s_val<<",",<<d_val
    //TODO printing the calculated values in array - check this indexing
    //std::cout<<"acc : "<<acc_pts[i]<<" vel : "<<vpts[i]<<" posi : "<<spts[i]<<std::endl;
    geometry_msgs::PoseStamped examplePose;
    examplePose.pose.position.x = xy[0];
    examplePose.pose.position.y = xy[1];
    //Currently this velocity is used in trajectory converted to publish velocity at a point
    examplePose.pose.position.z = 0;//v(t_pt); //velocity saved in z direction
    examplePose.pose.orientation.x = 0.0f;//a(t_pt); // save accleration in orientation
    examplePose.pose.orientation.y = 0.0f;
    examplePose.pose.orientation.z = 0.0f;

    //push PoseStamped into Path
    m_sampled_traj.poses.push_back(examplePose);
  }
  //Publish as path with velocity in z dimension
  m_mp_traj.publish(m_sampled_traj);

}//end of create trajectory


}//namespace end
