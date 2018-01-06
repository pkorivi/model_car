
namespace fub_motion_planner{

  // Evaluate a polynomial.
  double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
      result += coeffs[i] * pow(x, i);
    }
    return result;
  }

  // Evaluate a polynomial Derivative.
  double polyeval_derivative(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 1; i < coeffs.size(); i++) {
      result += i*coeffs[i] * pow(x, i-1);
    }
    return result;
  }

  // Evaluate a polynomial Double derivative.
  double polyeval_double_derivative(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 2; i < coeffs.size(); i++) {
      result += i*(i-1)*coeffs[i] * pow(x, i-2);
    }
    return result;
  }

  // Fit a polynomial.
  // Adapted from
  // https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
  Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                          int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
      A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
      for (int i = 0; i < order; i++) {
        A(j, i + 1) = A(j, i) * xvals(j);
      }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
  }

//Traj with polynomials_
//TODO this will return the cost and trajectory - If stored it will take up lot of space
//void MotionPlanner::create_traj(VehicleState current_state){
void MotionPlanner::create_traj(VehicleState current_state,VehicleState prev_state, ros::Publisher&  traj_pub, \
        double v_target,double a_target,double d_target,double v_max, double v_min,int polynomial_order){

  //current values
  double v_current = current_state.m_current_speed_front_axle_center;
  //Condition added to prevent speeding up if current velocity is greater than target and positive acceleration is requested
  if(v_current > v_target && a_target > 0){
    ROS_ERROR("V_Cur is > v_tgt and acceleration is requested");
    return;
  }
  geometry_msgs::PointStamped pt_Stamped_in,pt_stamped_out;
  pt_Stamped_in.header.seq =1;
  pt_Stamped_in.header.stamp = ros::Time::now();
  pt_Stamped_in.header.frame_id= "/odom";
  pt_Stamped_in.point.x = current_state.m_vehicle_position[0];
  pt_Stamped_in.point.y = current_state.m_vehicle_position[1];
  pt_Stamped_in.point.z = 0;
  //tf::StampedTransform transform;
  try{
    m_tf_listener.listener.transformPoint("/map", pt_Stamped_in, pt_stamped_out);
    //listener.lookupTransform("/turtle2", "/turtle1",ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
  }

  //current values
  tf::Point cp ;//= current_state.m_vehicle_position;
  cp[0] =  pt_stamped_out.point.x;
  cp[1] =  pt_stamped_out.point.y;
  //std::cout << "map transformed cp "<< cp[0]<<" , "<<cp[1] << '\n';
  double c_yaw = current_state.getVehicleYaw();

  double time_from_prev_cycle = (current_state.m_last_odom_time_stamp_received - prev_state.m_last_odom_time_stamp_received).toSec();
  //prev velocity value
  double prev_vel = prev_state.m_current_speed_front_axle_center;
  double a_current =0;
  if(time_from_prev_cycle > 0.001)
    a_current = (v_current-prev_vel)/time_from_prev_cycle; // TODO update this value from the odometry info
  //std::cout <<" time :  "<< time_from_prev_cycle<<"  acc_current : " << a_current <<'\n';
  //TODO Add condition to skip if v_current > v_target and a_target > 0

  int number_of_samples = 25;
  Eigen::VectorXd spts(number_of_samples);
  Eigen::VectorXd tpts(number_of_samples);
  Eigen::VectorXd vpts(number_of_samples);
  Eigen::VectorXd acc_pts(number_of_samples);
  //TODO change the number
  Eigen::VectorXd dpts(6);


  double a_ref, v_ref, s_ref;
  enum AccStates { TO_REQ, CONSTANT, TO_ZERO,ZERO};
  AccStates c_acc_phase = TO_REQ;
  //get current position in frenet frame
  //Adding a condition to perform traj generation when there is path to be followed
  //std::cout<<"New evaluation"<<std::endl;
  //current point in frenet
  FrenetCoordinate frenet_val =  m_vehicle_path.getFenet(cp,c_yaw);
  ROS_INFO("map-xy %.3f,%.3f , odom x,y : %.3f,%.3f , cur a: %.3f v: %.3f ",cp[0],cp[1],current_state.m_vehicle_position[0],current_state.m_vehicle_position[1],a_current,v_current);
  ROS_INFO("frenet s,d %.3f %.3f ", frenet_val.s, frenet_val.d);
  //Initial time and
  spts[0] = (frenet_val.s);
  dpts[0] = (frenet_val.d);
  vpts[0] = (v_current);
  tpts[0] = (0);
  acc_pts[0] = (a_current);
  // if cureent is greater than target then acceleration will decrease and this variable is -ve
  double acc_inc_dec = (a_current>a_target?-1:1);
  //jerk: assuming it takes 1s to reach a_target from zero. Sign of jerk is computed from current acceleration
  //If current is greter than target then slope is negative, jerk is negative
  double jerk_val = fabs(a_target==0?0.2:a_target)*acc_inc_dec;
  //to come to zero acc, it should decrease if it is greater than zero and increase if its less than zero
  //TODO zero stuff here- something can be messy
  double to_zero_acc_inc_dec = (a_target>0?-1:1);
  double to_zero_jerk_val = fabs(a_target==0?0.2:a_target)*to_zero_acc_inc_dec;

  //TODO update jerk such that it takes 1s to reach from current acceleration to the target acceleration
  //Time samples of 100ms each, so for 5 seconds we have 50 samples - TODO this as tunable parameter
  double t_sample = (5*1.0)/number_of_samples;
  //It is the change in velocity if the acceleration is made to zero from current acceleration with constannt jerk - 0.5*a*t & t is always positive
  double v_change=(0.5*a_current*fabs(a_current/jerk_val));
  for(int i=1;i<number_of_samples;i++){
    //std::cout << "vcur : " << vpts[i-1]<< " acur :"<< acc_pts[i-1] <<"  vtgt "<<v_target<<" a_tgt : "<<a_target << " vch "<<v_change <<'\n';
    //std::cout << "abs v : " << fabs(v_target-vpts[i-1])<< " abs a : " <<fabs(acc_pts[i-1]-acc[2]);
    tpts[i] =(i*t_sample);
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

        acc_pts[i]=(a_ref);
        v_ref = vpts[i-1] + a_ref*t_sample;// increased velocity is the area of rectangle formed by acceleration and time sample
        // Bound the velocity
        if (v_ref > v_max)
          v_ref = v_max;
        else if (v_ref<v_min)
          v_ref = v_min;

        vpts[i]=(v_ref);
        //TODO as per paper they use v_ref*t_sample - As the sample size is small, even this can be chosen. Compare both values and choose best possible
        //acceleration is a function of time - This can be simply approximated I think
        //spts.push_back(spts[i-1] + vpts[i-1]*t_sample + 0.5*jerk_val*t_sample*t_sample*t_sample);
        spts[i]=(spts[i-1] + v_ref*t_sample);
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
        acc_pts[i]=(a_target);
        v_ref = vpts[i-1] + a_target*t_sample;
        // Bound the velocity
        if (v_ref > v_max)
          v_ref = v_max;
        else if (v_ref<v_min)
          v_ref = v_min;
        vpts[i]=(v_ref);

        spts[i]=(spts[i-1] + vpts[i-1]*t_sample + 0.5*a_target*t_sample*t_sample);
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
        acc_pts[i]=(a_ref);
        v_ref = vpts[i-1] + a_ref*t_sample;// increased velocity is the area of rectangle formed by acceleration and time sample
        // Bound the velocity
        if (v_ref > v_max)
          v_ref = v_max;
        else if (v_ref<v_min)
          v_ref = v_min;
        vpts[i]=(v_ref);
        //TODO as per paper they use v_ref*t_sample - As the sample size is small, even this can be chosen. Compare both values and choose best possible
        //acceleration is a function of time - This can be simply approximated I think
        //spts.push_back(spts[i-1] + vpts[i-1]*t_sample + 0.5*jerk_val*t_sample*t_sample*t_sample);
        spts[i]=(spts[i-1] + v_ref*t_sample);
        break;
      }
      default: {
        std::cout << "Oops something is wrong" << '\n';
      }
    }
  }//for loop
  //std::cout << "v_change :" << v_change << '\n';
  //int polynomial_order=4;
  auto s_coeffs = polyfit(tpts, spts,polynomial_order);
  auto v_coeffs = polyfit(tpts,vpts,polynomial_order);
  auto a_coeffs = polyfit(tpts,acc_pts,polynomial_order);


  //for change in d -
  //TODO add d to maintain current raidus of curvature
  dpts[1]=(0);
  dpts[2]=(0);
  dpts[3]=(d_target);
  dpts[4]=(d_target);
  dpts[5]=(d_target);
  Eigen::VectorXd d_t_pts(6);
  d_t_pts[0] = 0.0;
  d_t_pts[1] = 0.2;
  d_t_pts[2] = 0.4;
  d_t_pts[3] = 4.6;
  d_t_pts[4] = 4.8;
  d_t_pts[5] = 5.0;
  auto d_coeffs = polyfit(d_t_pts,dpts,polynomial_order);

  nav_msgs::Path m_sampled_traj;
  m_sampled_traj.header.stamp = ros::Time::now();
  m_sampled_traj.header.frame_id = "/map";
  //sample every 0.2s
  for(double i=0;i<25;i++){
    double t_pt = 0.2*i;//time
    //double s_val = s(t_pt);
    //double d_val = d(t_pt);
    //Eigen::VectorXd
    double s_val = polyeval( s_coeffs, t_pt);
    double d_val = polyeval( d_coeffs, t_pt);
    double v_val = polyeval( v_coeffs, t_pt);
    double a_val = polyeval( a_coeffs, t_pt);

    tf::Point xy = m_vehicle_path.getXY(FrenetCoordinate(s_val,d_val,0)); //TODO check yaw stuff
    //TODO - remove this
    ROS_INFO("xy %.3f,%.3f , s,d %.3f, %.3f , a: %.3f v: %.3f ",xy[0],xy[1], s_val, d_val, a_val, v_val);
    geometry_msgs::PoseStamped examplePose;
    examplePose.pose.position.x = xy[0];
    examplePose.pose.position.y = xy[1];
    //Currently this velocity is used in trajectory converted to publish velocity at a point
    examplePose.pose.position.z = v_val;//v(t_pt); //velocity saved in z direction
    examplePose.pose.orientation.x = a_val;//0.0f;//a(t_pt); // save accleration in orientation
    examplePose.pose.orientation.y = 0.0f;
    examplePose.pose.orientation.z = 0.0f;

    //push PoseStamped into Path
    m_sampled_traj.poses.push_back(examplePose);
  }
  //Publish as path with velocity in z dimension
  traj_pub.publish(m_sampled_traj);

}//end of create trajectory



//TODO
/*
Incorporate checks that +ve acc are chosen only when vtgt > vcur
Improve execution time - its at 3ms for one traj now
Reduce variables, Use config files for some variables
Incorporate distance travelled as a parameter, currently if the min dist to goal is less nothing happens
Use pointers and avoid variables copies when ever possible
return a score for each trajectory here after testing the obstacle avoidance etc
*/
///*

void MotionPlanner::create_traj_spline(VehicleState current_state, VehicleState prev_state, ros::Publisher&  traj_pub, \
        double v_target,double a_target,double d_target,double v_max, double v_min, int polynomial_order){

  //current values
  double v_current = current_state.m_current_speed_front_axle_center;
  //Condition added to prevent speeding up if current velocity is greater than target and positive acceleration is requested
  if(v_current > v_target && a_target > 0){
    ROS_ERROR("V_Cur is > v_tgt and acceleration is requested");
    return;
  }

  geometry_msgs::PointStamped pt_Stamped_in,pt_stamped_out;
  pt_Stamped_in.header.seq =1;
  pt_Stamped_in.header.stamp = ros::Time::now();
  pt_Stamped_in.header.frame_id= "/odom";
  pt_Stamped_in.point.x = current_state.m_vehicle_position[0];
  pt_Stamped_in.point.y = current_state.m_vehicle_position[1];
  pt_Stamped_in.point.z = 0;
  //tf::StampedTransform transform;
  try{
    m_tf_listener.listener.transformPoint("/map", pt_Stamped_in, pt_stamped_out);
    //listener.lookupTransform("/turtle2", "/turtle1",ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
  }

  //current values
  tf::Point cp ;//= current_state.m_vehicle_position;
  cp[0] =  pt_stamped_out.point.x;
  cp[1] =  pt_stamped_out.point.y;
  //std::cout << "map transformed cp "<< cp[0]<<" , "<<cp[1] << '\n';
  double time_from_prev_cycle = (current_state.m_last_odom_time_stamp_received - prev_state.m_last_odom_time_stamp_received).toSec();
  //prev velocity value
  double prev_vel = prev_state.m_current_speed_front_axle_center;
  double a_current =0;
  if(time_from_prev_cycle > 0.001)
    a_current = (v_current-prev_vel)/time_from_prev_cycle; // TODO update this value from the odometry info

  //TODO - remove this - this is introduced as the current acceleration calculation is not working, so instead of ramp
  // upto target acceleration. Start the process at constannt acceleration or reducing accaleration
  a_current = a_target;

  std::cout <<" time :  "<< time_from_prev_cycle<<"  acc_current : " << a_current <<'\n';

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
  //std::cout<<"New evaluation"<<std::endl;
  //current point in frenet
  FrenetCoordinate frenet_val =  m_vehicle_path.getFenet(cp,c_yaw);
  ROS_INFO("map-xy %.3f,%.3f , odom x,y : %.3f,%.3f , cur a: %.3f v: %.3f ",cp[0],cp[1],current_state.m_vehicle_position[0],current_state.m_vehicle_position[1],a_current,v_current);
  ROS_INFO("frenet s,d %.3f %.3f ", frenet_val.s, frenet_val.d);

  //TODO remove this - just to check if lane chnages are happening
  ROS_INFO("Remove the below lane change logic");
  if(frenet_val.d > 0.17){
    d_target = -0.21;}
  else if(frenet_val.d < -0.17){
    d_target = 0.21;}
  else{

  }
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
  //Create splines
  tk::spline s;
  tk::spline v;
  tk::spline a;
  tk::spline d;
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
    double v_val = v(t_pt);
    double a_val = a(t_pt);
    //std::cout <<i << " Spline eval : s v a " <<s_val<<" , "<<v_val<<" , "<<a_val<<" , "<< '\n';
    tf::Point xy = m_vehicle_path.getXY(FrenetCoordinate(s_val,d_val,0)); //TODO check yaw stuff
    //std::cout<<"  (x,y) : "<<xy[0]<<','<<xy[1]<<std::endl;//',    (s,d)'<<s_val<<",",<<d_val
    //TODO printing the calculated values in array - check this indexing
    //std::cout<<"acc : "<<acc_pts[i]<<" vel : "<<vpts[i]<<" posi : "<<spts[i]<<std::endl;

    ROS_INFO("xy %.3f,%.3f , s,d %.3f, %.3f , a: %.3f v: %.3f ",xy[0],xy[1], s_val, d_val, a_val, v_val);

    geometry_msgs::PoseStamped examplePose;
    examplePose.pose.position.x = xy[0];
    examplePose.pose.position.y = xy[1];
    //Currently this velocity is used in trajectory converted to publish velocity at a point
    examplePose.pose.position.z = v_val;//v(t_pt); //velocity saved in z direction
    examplePose.pose.orientation.x = a_val;//a(t_pt); // save accleration in orientation
    examplePose.pose.orientation.y = 0.0f;
    examplePose.pose.orientation.z = 0.0f;

    //push PoseStamped into Path
    m_sampled_traj.poses.push_back(examplePose);
  }
  //Publish as path with velocity in z dimension
  traj_pub.publish(m_sampled_traj);

}//end of create trajectory
//*/



/*
This is a trajectory creator which uses constant velocity instead of ramp up and ramp down-
a_current calculation is error prone because of the minimum value in velocity changes. As the precision is bad it is not possible to
measure acceleration accurately between 0-0.15 or other value. This is causing troubles in trajectory creation
*/
void MotionPlanner::create_traj_const_acc(VehicleState current_state,VehicleState prev_state, ros::Publisher&  traj_pub, \
        double v_target,double a_target,double d_target,double v_max, double v_min,int polynomial_order){

  //current values
  double v_current = current_state.m_current_speed_front_axle_center;
  //Condition added to prevent speeding up if current velocity is greater than target and positive acceleration is requested
  if(v_current > v_target && a_target > 0){
    ROS_ERROR("V_Cur is > v_tgt and acceleration is requested");
    //TODO return proper value in future, traj cost and other things
    return;
  }
  //Odom frame to map frame conversion for trajectory
  geometry_msgs::PointStamped pt_Stamped_in,pt_stamped_out;
  pt_Stamped_in.header.seq =1;
  pt_Stamped_in.header.stamp = ros::Time::now();
  pt_Stamped_in.header.frame_id= "/odom";
  pt_Stamped_in.point.x = current_state.m_vehicle_position[0];
  pt_Stamped_in.point.y = current_state.m_vehicle_position[1];
  pt_Stamped_in.point.z = 0;
  try{
    m_tf_listener.listener.transformPoint("/map", pt_Stamped_in, pt_stamped_out);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
  }

  //current values in Map frame
  tf::Point cp ;
  cp[0] =  pt_stamped_out.point.x;
  cp[1] =  pt_stamped_out.point.y;
  //std::cout << "map transformed cp "<< cp[0]<<" , "<<cp[1] << '\n';
  double c_yaw = current_state.getVehicleYaw();
  double time_from_prev_cycle = (current_state.m_last_odom_time_stamp_received - prev_state.m_last_odom_time_stamp_received).toSec();
  //TODO - read from confug file
  int number_of_samples = 25;
  Eigen::VectorXd spts(number_of_samples);
  Eigen::VectorXd tpts(number_of_samples);
  //TODO change the number
  Eigen::VectorXd dpts(6);

  double a_ref, v_ref, s_ref;
  //Different Acceleraton States
  enum AccStates { CONSTANT_ACC, ZERO_ACC, BRAKE_DEC};
  AccStates c_acc_phase = CONSTANT_ACC;
  //current point in frenet
  FrenetCoordinate frenet_val =  m_vehicle_path.getFenet(cp,c_yaw);
  //ROS_INFO("map-xy %.3f,%.3f , odom x,y : %.3f,%.3f , cur a: 0 v: %.3f ",cp[0],cp[1],current_state.m_vehicle_position[0],current_state.m_vehicle_position[1],v_current);
  //ROS_INFO("frenet s,d %.3f %.3f ", frenet_val.s, frenet_val.d);
  //Initial Points for polyfit
  spts[0] = (frenet_val.s);
  dpts[0] = (frenet_val.d);
  tpts[0] = (0);
  //TODO - add in config file
  //If the target distance is short the planner should accelerate and deccelerate in 5s, else it will not be able to find a path.
  //This will help create a path with ability to stop if the final destination is arrived
  double brake_dec = 0.3;
  double d_brake =(v_current*v_current)/(2*brake_dec);//v² -u² = 2as, thus to stop with current velocity it is s = -u²/2a; a is -ve this s = u²/2a
  //Time samples of 100ms each, so for 5 seconds we have 50 samples - TODO this as tunable parameter
  double t_sample = (5.0)/number_of_samples;
  double v_previous = v_current;
  for(int i=1;i<number_of_samples;i++){
    //std::cout << "vcur : " << vpts[i-1]<< " acur :"<< acc_pts[i-1] <<"  vtgt "<<v_target<<" a_tgt : "<<a_target << " vch "<<v_change <<'\n';
    //std::cout << "abs v : " << fabs(v_target-vpts[i-1])<< " abs a : " <<fabs(acc_pts[i-1]-acc[2]);
    tpts[i] =(i*t_sample);
    switch (c_acc_phase) {
      case CONSTANT_ACC: {
        //std::cout << "  :  const acc" << '\n';
        v_ref = v_previous + a_target*t_sample;
        // Bound the velocity
        v_ref = v_ref>v_max?v_max:v_ref;
        v_ref = v_ref<v_min?v_min:v_ref;

        //If the v_ref reaches near v_target, make it v_target - this is mainly useful while acceleration and prevent overshoots
        if(fabs(v_target-v_ref) < 0.03)
          v_ref = v_target;

        spts[i] = (v_ref>0)?(spts[i-1] + v_previous*t_sample + 0.5*a_target*t_sample*t_sample):spts[i-1];
        d_brake =(v_ref*v_ref)/(2*brake_dec);
        //0.1 of  extra buffer stopping distance
        // Go to braking if the available road is less and the acceleration requested is greater then or equal to zero.
        // If braking is requested then keep this as a part of cost term
        //TODO - add constants in config file
        if((d_brake > (m_vehicle_path.frenet_path.back().s - spts[i] - 0.1))&&(a_target>=0)){
          c_acc_phase =BRAKE_DEC;
        }
        //TODO If the velocity is in bounds of 0.04 around then skip to zero acceleration phase
        else if((v_ref==v_target)||(a_target == 0)){
          c_acc_phase = ZERO_ACC;
        }
        break;
      }
      case ZERO_ACC: {
        //std::cout << "  : zero" << '\n';
        //Zero acceleration
        v_ref = v_previous;//zero acceleration - constant previous velocity
        // Bound the velocity
        v_ref = v_ref>v_max?v_max:v_ref;
        v_ref = v_ref<v_min?v_min:v_ref;
        spts[i] = (v_ref>0)?(spts[i-1] + v_ref*t_sample):spts[i-1];
        d_brake =(v_ref*v_ref)/(2*brake_dec);
        //0.1 of  extra buffer stopping distance
        // Go to braking if the available road is less and the acceleration requested is greater then or equal to zero.
        // If braking is requested then keep this as a part of cost term
        //TODO - add constants in config file
        if((d_brake > (m_vehicle_path.frenet_path.back().s - spts[i] - 0.1))&&(a_target>=0)){
          c_acc_phase =BRAKE_DEC;
        }
        break;
      }
      case BRAKE_DEC: {
        //std::cout << "  :  decc acc" << '\n';
        //constant acceleration
        v_ref = v_previous - brake_dec*t_sample;
        // Bound the velocity
        v_ref = v_ref>v_max?v_max:v_ref;
        v_ref = v_ref<v_min?v_min:v_ref;
        spts[i] = (v_ref>0)?(spts[i-1] + v_previous*t_sample - 0.5*brake_dec*t_sample*t_sample):spts[i-1];
        break;
      }
      default: {
        std::cout << "Oops something is wrong" << '\n';
      }
    }
    //ROS_INFO("%.3f",spts[i]);//TODO remove
    //Store the velocity calculated in this cycle for next cycle
    v_previous = v_ref;
  }//for loop

  //Fit the points path points to a polynomial of given order
  auto s_coeffs = polyfit(tpts, spts,polynomial_order);
  //for change in d -
  //TODO add d to maintain current raidus of curvature
  dpts[1]=dpts[0];
  dpts[2]=dpts[0];
  dpts[3]=(d_target);
  dpts[4]=(d_target);
  dpts[5]=(d_target);
  //TODO change the time here from 5s to time when velocity becomez zero , default is 5s and if velocity becomes zero before that d should change before that
  // change d as  parameter of s instead of time ?
  // implement logic from the urban planning paper for change in d
  Eigen::VectorXd d_t_pts(6);
  d_t_pts[0] = 0.0;
  d_t_pts[1] = 0.2;
  d_t_pts[2] = 0.4;
  d_t_pts[3] = 4.6;
  d_t_pts[4] = 4.8;
  d_t_pts[5] = 5.0;

  auto d_coeffs = polyfit(d_t_pts,dpts,polynomial_order);

  nav_msgs::Path m_sampled_traj;
  m_sampled_traj.header.stamp = ros::Time::now();
  m_sampled_traj.header.frame_id = "/map";
  double s_val,d_val,v_val, a_val;
  //sample every 0.2s
  for(double i=0;i<25;i++){
    double t_pt = 0.2*i;//time
    s_val = polyeval( s_coeffs, t_pt);
    d_val = polyeval( d_coeffs, t_pt);
    v_val = polyeval_derivative(s_coeffs,t_pt);
    v_val = v_val<v_min?v_min:v_val;
    a_val = polyeval_double_derivative(s_coeffs,t_pt);

    tf::Point xy = m_vehicle_path.getXY(FrenetCoordinate(s_val,d_val,0)); //TODO check yaw stuff
    //TODO - remove this
    //ROS_INFO("xy %.3f,%.3f , s,d %.3f, %.3f , a: %.3f v: %.3f ",xy[0],xy[1], s_val, d_val, a_val, v_val);
    geometry_msgs::PoseStamped examplePose;
    examplePose.pose.position.x = xy[0];
    examplePose.pose.position.y = xy[1];
    //Currently this velocity is used in trajectory converted to publish velocity at a point
    examplePose.pose.position.z = v_val;//v(t_pt); //velocity saved in z direction
    examplePose.pose.orientation.x = a_val;//0.0f;//a(t_pt); // save accleration in orientation //TODO - calculate double derivative for acceleration
    examplePose.pose.orientation.y = 0.0f;
    examplePose.pose.orientation.z = 0.0f;

    //push PoseStamped into Path
    m_sampled_traj.poses.push_back(examplePose);
  }
  //Publish as path with velocity in z dimension
  traj_pub.publish(m_sampled_traj);

}//end of create trajectory




void MotionPlanner::create_traj_const_acc_xy_polyeval(VehicleState current_state,VehicleState prev_state, ros::Publisher&  traj_pub, \
        double v_target,double a_target,double d_target,double v_max, double v_min,int polynomial_order){

  clock_t tStart = clock();
  //current values
  double v_current = current_state.m_current_speed_front_axle_center;
  //Condition added to prevent speeding up if current velocity is greater than target and positive acceleration is requested
  if(v_current > v_target && a_target > 0){
    ROS_ERROR("V_Cur is > v_tgt and acceleration is requested");
    //TODO return proper value in future, traj cost and other things
    return;
  }
  //Odom frame to map frame conversion for trajectory
  geometry_msgs::PointStamped pt_Stamped_in,pt_stamped_out;
  pt_Stamped_in.header.seq =1;
  pt_Stamped_in.header.stamp = ros::Time::now();
  pt_Stamped_in.header.frame_id= "/odom";
  pt_Stamped_in.point.x = current_state.m_vehicle_position[0];
  pt_Stamped_in.point.y = current_state.m_vehicle_position[1];
  pt_Stamped_in.point.z = 0;
  try{
    m_tf_listener.listener.transformPoint("/map", pt_Stamped_in, pt_stamped_out);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
  }

  //current values in Map frame
  tf::Point cp ;
  cp[0] =  pt_stamped_out.point.x;
  cp[1] =  pt_stamped_out.point.y;
  //std::cout << "map transformed cp "<< cp[0]<<" , "<<cp[1] << '\n';
  double c_yaw = current_state.getVehicleYaw();
  double time_from_prev_cycle = (current_state.m_last_odom_time_stamp_received - prev_state.m_last_odom_time_stamp_received).toSec();
  //TODO - read from confug file
  int number_of_samples = 10;
  Eigen::VectorXd spts(number_of_samples);
  Eigen::VectorXd tpts(number_of_samples);
  Eigen::VectorXd xpts(number_of_samples);
  Eigen::VectorXd ypts(number_of_samples);
  //TODO change the number
  Eigen::VectorXd dpts(6);

  double a_ref, v_ref, s_ref;
  //Different Acceleraton States
  enum AccStates { CONSTANT_ACC, ZERO_ACC, BRAKE_DEC};
  AccStates c_acc_phase = CONSTANT_ACC;
  //current point in frenet
  FrenetCoordinate frenet_val =  m_vehicle_path.getFenet(cp,c_yaw);
  //ROS_INFO("map-xy %.3f,%.3f , odom x,y : %.3f,%.3f , cur a: 0 v: %.3f ",cp[0],cp[1],current_state.m_vehicle_position[0],current_state.m_vehicle_position[1],v_current);
  //ROS_INFO("frenet s,d %.3f %.3f ", frenet_val.s, frenet_val.d);
  //Initial Points for polyfit
  spts[0] = (frenet_val.s);
  tpts[0] = (0);
  //Iniial x,y - as per map coordinates
  xpts[0] = ( pt_stamped_out.point.x);
  ypts[0] = ( pt_stamped_out.point.y);
  std::cout << "x,y "<<xpts[0]<<"  "<<ypts[0] << '\n';
  //start of d_stuff
  dpts[0] = (frenet_val.d);
  dpts[1]=dpts[0];
  dpts[2]=dpts[0];
  dpts[3]=(d_target);
  dpts[4]=(d_target);
  dpts[5]=(d_target);
  //TODO change the time here from 5s to time when velocity becomez zero , default is 5s and if velocity becomes zero before that d should change before that
  // change d as  parameter of s instead of time ?
  // implement logic from the urban planning paper for change in d
  Eigen::VectorXd d_t_pts(6);
  d_t_pts[0] = 0.0;
  d_t_pts[1] = 0.2;
  d_t_pts[2] = 0.4;
  d_t_pts[3] = 4.6;
  d_t_pts[4] = 4.8;
  d_t_pts[5] = 5.0;

  ROS_INFO("Initialization: %f\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  tStart = clock();
  auto d_coeffs = polyfit(d_t_pts,dpts,polynomial_order);
  ROS_INFO("polyfit d: %f\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  tStart = clock();
  //end of d_calc

  //TODO - add in config file
  //If the target distance is short the planner should accelerate and deccelerate in 5s, else it will not be able to find a path.
  //This will help create a path with ability to stop if the final destination is arrived
  double brake_dec = 0.3;
  double d_brake =(v_current*v_current)/(2*brake_dec);//v² -u² = 2as, thus to stop with current velocity it is s = -u²/2a; a is -ve this s = u²/2a
  //Time samples of 100ms each, so for 5 seconds we have 50 samples - TODO this as tunable parameter
  double t_sample = (5.0)/number_of_samples;
  double v_previous = v_current;
  for(int i=1;i<number_of_samples;i++){
    //std::cout << "vcur : " << vpts[i-1]<< " acur :"<< acc_pts[i-1] <<"  vtgt "<<v_target<<" a_tgt : "<<a_target << " vch "<<v_change <<'\n';
    //std::cout << "abs v : " << fabs(v_target-vpts[i-1])<< " abs a : " <<fabs(acc_pts[i-1]-acc[2]);
    tpts[i] =(i*t_sample);
    switch (c_acc_phase) {
      case CONSTANT_ACC: {
        //std::cout << "  :  const acc" << '\n';
        v_ref = v_previous + a_target*t_sample;
        // Bound the velocity
        v_ref = v_ref>v_max?v_max:v_ref;
        v_ref = v_ref<v_min?v_min:v_ref;

        //If the v_ref reaches near v_target, make it v_target - this is mainly useful while acceleration and prevent overshoots
        if(fabs(v_target-v_ref) < 0.03)
          v_ref = v_target;

        spts[i] = (v_ref>0)?(spts[i-1] + v_previous*t_sample + 0.5*a_target*t_sample*t_sample):spts[i-1];
        d_brake =(v_ref*v_ref)/(2*brake_dec);
        //0.1 of  extra buffer stopping distance
        // Go to braking if the available road is less and the acceleration requested is greater then or equal to zero.
        // If braking is requested then keep this as a part of cost term
        //TODO - add constants in config file
        if((d_brake > (m_vehicle_path.frenet_path.back().s - spts[i] - 0.1))&&(a_target>=0)){
          c_acc_phase =BRAKE_DEC;
        }
        //TODO If the velocity is in bounds of 0.04 around then skip to zero acceleration phase
        else if((v_ref==v_target)||(a_target == 0)){
          c_acc_phase = ZERO_ACC;
        }
        break;
      }
      case ZERO_ACC: {
        //std::cout << "  : zero" << '\n';
        //Zero acceleration
        v_ref = v_previous;//zero acceleration - constant previous velocity
        // Bound the velocity
        v_ref = v_ref>v_max?v_max:v_ref;
        v_ref = v_ref<v_min?v_min:v_ref;
        spts[i] = (v_ref>0)?(spts[i-1] + v_ref*t_sample):spts[i-1];
        d_brake =(v_ref*v_ref)/(2*brake_dec);
        //0.1 of  extra buffer stopping distance
        // Go to braking if the available road is less and the acceleration requested is greater then or equal to zero.
        // If braking is requested then keep this as a part of cost term
        //TODO - add constants in config file
        if((d_brake > (m_vehicle_path.frenet_path.back().s - spts[i] - 0.1))&&(a_target>=0)){
          c_acc_phase =BRAKE_DEC;
        }
        break;
      }
      case BRAKE_DEC: {
        //std::cout << "  :  decc acc" << '\n';
        //constant acceleration
        v_ref = v_previous - brake_dec*t_sample;
        // Bound the velocity
        v_ref = v_ref>v_max?v_max:v_ref;
        v_ref = v_ref<v_min?v_min:v_ref;
        spts[i] = (v_ref>0)?(spts[i-1] + v_previous*t_sample - 0.5*brake_dec*t_sample*t_sample):spts[i-1];
        break;
      }
      default: {
        std::cout << "Oops something is wrong" << '\n';
      }
    }
    //ROS_INFO("%.3f",spts[i]);//TODO remove
    double d_val1 = polyeval( d_coeffs, tpts[i]);
    tf::Point xy = m_vehicle_path.getXY(FrenetCoordinate(spts[i],d_val1,0)); //TODO check yaw stuff
    xpts[i]= xy[0];
    ypts[i]= xy[1];
    std::cout << "x,y"<<xy[0]<<"  "<<xy[1] << "   time :"<<tpts[i]<<'\n';
    //Store the velocity calculated in this cycle for next cycle
    v_previous = v_ref;
  }//for loop

  ROS_INFO("for loop: %f\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  tStart = clock();

  //Fit the points path points to a polynomial of given order
  auto x_coeffs = polyfit(tpts, xpts,polynomial_order);
  auto y_coeffs = polyfit(tpts, ypts,polynomial_order);
  ROS_INFO("polyfit x,y: %f\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  tStart = clock();

  nav_msgs::Path m_sampled_traj;
  m_sampled_traj.header.stamp = ros::Time::now();
  m_sampled_traj.header.frame_id = "/map";
  double s_val,d_val,v_val, a_val;
  //sample every 0.2s
  for(double i=0;i<25;i++){
    double t_pt = 0.2*i;//time

    double x_der = polyeval_derivative(x_coeffs,t_pt);
    double y_der = polyeval_derivative(y_coeffs,t_pt);
    v_val = sqrt(x_der*x_der + y_der*y_der);
    //a_val = polyeval_double_derivative(s_coeffs,t_pt);

    //TODO - remove this
    //ROS_INFO("xy %.3f,%.3f , s,d %.3f, %.3f , a: %.3f v: %.3f ",xy[0],xy[1], s_val, d_val, a_val, v_val);
    geometry_msgs::PoseStamped examplePose;
    examplePose.pose.position.x = polyeval( x_coeffs, t_pt);
    examplePose.pose.position.y = polyeval( y_coeffs, t_pt);
    //Currently this velocity is used in trajectory converted to publish velocity at a point
    examplePose.pose.position.z = v_val;//v(t_pt); //velocity saved in z direction
    examplePose.pose.orientation.x = 0.0;//a_val;//0.0f;//a(t_pt); // save accleration in orientation //TODO - calculate double derivative for acceleration
    examplePose.pose.orientation.y = 0.0f;
    examplePose.pose.orientation.z = 0.0f;

    //push PoseStamped into Path
    m_sampled_traj.poses.push_back(examplePose);
  }
  ROS_INFO("eval and pub traj: %f\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  tStart = clock();
  //Publish as path with velocity in z dimension
  traj_pub.publish(m_sampled_traj);

}//end of create trajectory



}//namespace end
