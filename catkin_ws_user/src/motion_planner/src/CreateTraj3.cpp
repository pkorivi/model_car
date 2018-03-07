namespace fub_motion_planner{
  /** Evaluate a polynomial.
  @params - polynomial coefficients and the point of evaluation
  **/
  double polyeval_m(std::vector<double> coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
      result += coeffs[i] * pow(x, i);
    }
    return result;
  }

  /* Function to fit a polynomial to the conditions of lateral motion.
    @params - current lateral position, target lateral position, theta is initial angle of car with respect to road
    current longitudinal position,target longitudinal position
    Concept Refernce "Real-Time Trajectory Planning for Autonomous Urban Driving: Framework, Algorithms, and Verifications"
    details provided in concept of Master thesis document also.
  */
  std::vector<double> evaluate_d_coeffs(double d_cur,double d_tgt, double theta, double s_i, double s_f){
    Eigen::MatrixXd A = Eigen::MatrixXd(4,4);
    double si_sq = s_i*s_i;
    double sf_sq = s_f*s_f;
    //Matrix A with s equations
    A << 1, s_i, si_sq , si_sq*s_i,
         0, 1  , 2*s_i , 3*si_sq,
         0, 1  , 2*s_f , 3*sf_sq,
         1, s_f, sf_sq , sf_sq*s_f;

    Eigen::MatrixXd B = Eigen::MatrixXd(4,1);
    B << d_cur,
         theta,
         0,
         d_tgt;
    Eigen::MatrixXd Ai = A.inverse();
    Eigen::MatrixXd C = Ai*B;
    std::vector<double> d_coeffs;
    for (size_t i = 0; i < C.size(); i++) {
      d_coeffs.push_back(C.data()[i]);
    }
    return d_coeffs;
  }

  /** Function to create a trajectory given initial conditions
  ** @params - vehicle current state, vehicle previous state, publisher to publish current path - this is for deug purpose only,
  ** target state, current point on map, current frenet coordinate
  ** Read through the Thesis document for full concept on implementation
  **/
  double MotionPlanner::create_traj_const_acc_xy_spline_3(VehicleState current_state,VehicleState prev_state, ros::Publisher&  traj_pub, \
           target_state &tgt,tf::Point current_pos_map, FrenetCoordinate frenet_val){

    clock_t tStart = clock();
    //Target values
    double v_target= tgt.v_tgt;
    double a_target= tgt.a_tgt;
    double d_target= tgt.d_eval;
    double s_target = tgt.s_tgt;
    double  cost =0;
    //current values
    double v_current = current_state.m_current_speed_front_axle_center;
    tgt.evaluated = true;
    //Condition added to prevent speeding up if current velocity is greater than target and positive acceleration is requested
    if((v_current > v_target && a_target > 0)||(v_current < v_target && a_target < 0)){
      ROS_ERROR("V_Cur is > v_tgt and acceleration is requested or v_cur<v_tgt and decelration requested");
      //TODO return proper value in future, traj cost and other things
      cost =20;
      tgt.cost += cost;
      return cost;
    }
    //current yaw
    double c_yaw = current_state.getVehicleYaw();
    //double time_from_prev_cycle = (current_state.m_last_odom_time_stamp_received - prev_state.m_last_odom_time_stamp_received).toSec();
    //Vectors to store the sample points in s,d frame and x,y frame wrt to time
    std::vector<double> spts;
    std::vector<double> dpts;
    //check if its needed
    std::vector<double> vpts;

    //Variables to use in calculation
    double a_ref, v_ref, s_ref;
    //Different Acceleraton States
    enum AccStates { CONSTANT_ACC, ZERO_ACC, BRAKE_DEC};
    AccStates c_acc_phase = CONSTANT_ACC;

    //current point in frenet
    //ROS_INFO("map-xyz %.3f,%.3f, %.3f , odom x,y : %.3f,%.3f , v: %.3f , yaw = %.3f",current_pos_map[0],current_pos_map[1],current_state.m_vehicle_position[0],current_state.m_vehicle_position[1],v_current,c_yaw);

    //Initial Points for spline/polynomial
    spts.push_back(frenet_val.s);
    vpts.push_back(v_current);


    //TODO - add in config file
    //If the target distance is short the planner should accelerate and deccelerate in 5s, else it will not be able to find a path.
    //This will help create a path with ability to stop if the final destination is arrived
    double brake_dec = 0.3;
    //Distance needed to break the car with brake_dec based on current velocity
    double d_brake =(v_current*v_current)/(2*brake_dec);//v² -u² = 2as, thus to stop with current velocity it is s = -u²/2a; a is -ve this s = u²/2a
    //Sample time divide the look ahead time into multiple copies
    double t_sample = kLookAheadTime/(kNumberOfSamples-1);
    double v_previous = v_current;
    for(int i=1;i<kNumberOfSamples;i++){
      switch (c_acc_phase) {
        case CONSTANT_ACC: {
          v_ref = v_previous + a_target*t_sample;
          // Bound the velocity - if acc vel should be <= target, if decc vel >= tgt
          v_ref = (a_target>=0)?(v_ref>v_target?v_target:v_ref):(v_ref<v_target?v_target:v_ref);
          //store s points and calculate breaking distance
          spts.push_back((v_ref>0)?(spts[i-1] + v_previous*t_sample + 0.5*a_target*t_sample*t_sample):spts[i-1]);
          d_brake =(v_ref*v_ref)/(2*brake_dec);
          //0.1 of  extra buffer stopping distance
          // Go to braking if the available road is less and the acceleration requested is greater then or equal to zero.
          // If braking is requested then keep this as a part of cost term
          //TODO - add constants in config files
          if((d_brake > (s_target - spts[i] - 0.1))&&(v_target>0)){
            c_acc_phase =BRAKE_DEC;
            cost += 2.0; //TODO how much to add, need to clarify
          }
          else if((v_ref==v_target)||(a_target == 0)){
            c_acc_phase = ZERO_ACC;
          }
          break;
        }
        case ZERO_ACC: {
          //Zero acceleration
          v_ref = v_previous;//zero acceleration - constant previous velocity
          // Bound the velocity - if acc vel should be <= target, if decc vel >= tgt
          v_ref = (a_target>=0)?(v_ref>v_target?v_target:v_ref):(v_ref<v_target?v_target:v_ref);
          //store s and d_breaking values
          spts.push_back((v_ref>0)?(spts[i-1] + v_ref*t_sample):spts[i-1]);
          d_brake =(v_ref*v_ref)/(2*brake_dec);
          //0.1 of  extra buffer stopping distance
          // Go to braking if the available road is less and the acceleration requested is greater then or equal to zero.
          // If braking is requested then keep this as a part of cost term
          //TODO - add constants in config file
          if((d_brake > (s_target - spts[i] - 0.1))&&(v_target>0)){
            c_acc_phase =BRAKE_DEC;
            cost += 2.0; //TODO how much to add, need to clarify
          }
          break;
        }
        case BRAKE_DEC: {
          //constant acceleration
          v_ref = v_previous - brake_dec*t_sample;
          // Bound the velocity
          v_ref = v_ref<0?0:v_ref;
          spts.push_back((v_ref>0)?(spts[i-1] + v_previous*t_sample - 0.5*brake_dec*t_sample*t_sample):spts[i-1]);
          break;
        }
        default: {
          std::cout << "Unknown state in trajectory creation - check whats wrong" << '\n';
        }
      }

      //Store the velocity calculated in this cycle for next cycle
      v_previous = v_ref;
      //pushback to store the velocity points
      vpts.push_back(v_ref);

    }//for loop

    //This trajectory is not moving min 0.01 distance should  be travelled
    if ( fabs(spts.back()-spts.front())<=0.02) {
      ROS_INFO("Trajectory traverses zero distance");
      cost =10; //TODO adjust the cost
      nav_msgs::Path m_sampled_traj;
      m_sampled_traj.header.stamp = ros::Time::now();
      m_sampled_traj.header.frame_id = "/map";
      for(double i=0;i<10;i++){
        double t_pt = 0.1*i;//time
        geometry_msgs::PoseStamped examplePose;
        //ROS_INFO("xy %.3f,%.3f , v_xy, v_fit : %.3f  %.3f ", x_val, y_val, v_val, v_fit);
        examplePose.pose.position.x = current_pos_map[0];
        examplePose.pose.position.y = current_pos_map[1];
        //Currently this velocity is used in trajectory converted to publish velocity at a point
        examplePose.pose.position.z = 0;//i*t_sample;//v_fit;//v(t_pt); //velocity saved in z direction
        examplePose.pose.orientation.x = 0.0;//a_val;//0.0f;//a(t_pt); // save accleration in orientation //TODO - calculate double derivative for acceleration
        examplePose.pose.orientation.y = 0.0f;//TODO fix orientation
        examplePose.pose.orientation.z = 0.0f;
        examplePose.pose.orientation.w = 1.0f;
        //push PoseStamped into Path
        m_sampled_traj.poses.push_back(examplePose);
      }
      tgt.path = m_sampled_traj;
      tgt.cost += cost;
      //last point reached in s
      tgt.s_reched = spts.back();

      return cost;
    }

    //calculate coefficients for lateral movement
    std::vector<double> d_coeffs = evaluate_d_coeffs(frenet_val.d,d_target,frenet_val.th, spts.front(), spts.back());
    if(std::isnan(d_coeffs[0])||std::isnan(d_coeffs[1])||std::isnan(d_coeffs[2])||std::isnan(d_coeffs[3])){
      ROS_ERROR("Nan in evaluation of d coeffs - traj planner");
      std::cout << "dcoeffs "<<d_coeffs[0]<<"," <<d_coeffs[1]<<"," <<d_coeffs[2]<<"," <<d_coeffs[3]<<"  car_d "<<frenet_val.d\
      << "  tgt_d "<<d_target<<"  th "<<frenet_val.th<<" smin_max "<<spts.front()<<","<<spts.back() << '\n';
    }

    //ECL Arrays for ecl splines
    ecl::Array<double> ecl_ts(kNumberOfSamples);
    ecl::Array<double> ecl_x(kNumberOfSamples);
    ecl::Array<double> ecl_y(kNumberOfSamples);
    //create xy sample points - making splines in x.y is beneficial over s,d as it will help maintain continuity in steering angle for controller to follow
    for (size_t i = 0; i < kNumberOfSamples; i++) {
      double d_val1 = polyeval_m( d_coeffs, spts[i]);
      dpts.push_back(d_val1);
      tf::Point xy = m_vehicle_path.getXY(FrenetCoordinate(spts[i],d_val1,0,0)); //TODO check yaw stuff
      //ECL spline poilts
      ecl_x[i] = xy[0];
      ecl_y[i] = xy[1];
      ecl_ts[i] = i*t_sample;

    }

    cost += CollisionCheck(current_state,spts,dpts,d_coeffs);
    //ROS_INFO("frenet to xy conversion: %f\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    tStart = clock();

    ecl::CubicSpline mSpline_x;
    ecl::CubicSpline mSpline_y;
    //ecl::CubicSpline mSpline_z; //TODO check if needed
    //TODO - last two points are change in velocity for x, y- follow as per fub controller soon
    mSpline_x = ecl::CubicSpline::ContinuousDerivatives(ecl_ts, ecl_x, v_current*cos(c_yaw), (ecl_x[kNumberOfSamples-1] - ecl_x[kNumberOfSamples-2])/t_sample);
    mSpline_y = ecl::CubicSpline::ContinuousDerivatives(ecl_ts, ecl_y, v_current*sin(c_yaw), (ecl_y[kNumberOfSamples-1] - ecl_y[kNumberOfSamples-2])/t_sample);
    //mSpline_z = ecl::CubicSpline::ContinuousDerivatives(x_set, y_set_z, frontVelocity.getZ(), backVelocity.getZ());
    //ROS_INFO("mp xy %.3f,%.3f conv_map_xy %.3f,%.3f, spline x,y %.3f,%.3f, v_xy %.3f,%.3f ",current_pos_map[0],current_pos_map[1],ecl_x[0],ecl_y[0],mSpline_x(0),mSpline_y(0),v_current*cos(c_yaw),v_current*sin(c_yaw));
    nav_msgs::Path m_sampled_traj;
    m_sampled_traj.header.stamp = ros::Time::now();
    m_sampled_traj.header.frame_id = "/map";
    double s_val,d_val,v_val, a_val;
    //Sample Path for the controller to follow -TODO check what are the ideal number of sample to follow or make these samples dependent on distance
    for(double i=0;i<51;i++){
      double t_pt = 0.1*i;//time
      geometry_msgs::PoseStamped examplePose;
      double x_val = mSpline_x(t_pt);//x(t_pt);
      double y_val = mSpline_y(t_pt);//y(t_pt);
      double dv_x = mSpline_x.derivative(t_pt);
      double dv_y = mSpline_y.derivative(t_pt);
      double v_fit = sqrt(dv_x*dv_x + dv_y*dv_y);
      //ROS_INFO("xy %.3f,%.3f , v_xy, v_fit : %.3f  %.3f ", x_val, y_val, v_val, v_fit);
      examplePose.pose.position.x = x_val;
      examplePose.pose.position.y = y_val;
      //Currently this velocity is used in trajectory converted to publish velocity at a point
      examplePose.pose.position.z = v_fit;//i*t_sample;//v_fit;//v(t_pt); //velocity saved in z direction
      examplePose.pose.orientation.x = 0.0;//a_val;//0.0f;//a(t_pt); // save accleration in orientation //TODO - calculate double derivative for acceleration
      examplePose.pose.orientation.y = 0.0f;
      examplePose.pose.orientation.z = 0.0f;
      examplePose.pose.orientation.w = 1.0f;//TODO fix orientation -  we have dx, dy it should be fairly easy to use these and calcluate orientation

      //push PoseStamped into Path
      m_sampled_traj.poses.push_back(examplePose);
    }
    //Publish as path with velocity in z dimension
    traj_pub.publish(m_sampled_traj);
    //last point reached in s
    tgt.s_reched = spts[kNumberOfSamples-1];

    //Add cost if s_reached is more than target over threshold
    //TODO make this threshold as a constant or in config file
    if(tgt.s_reched > s_target+kThresholdDist)
      cost += fabs(tgt.s_reched - s_target+kThresholdDist);

    //Reduce cost if target is reached //TODO make the low trajectories evaluated when one of the traj reaches end
    if((tgt.v_tgt==0)){
      if((fabs(tgt.s_reched - s_target) <= kThresholdDist)){
        cost -= 1; //Promote these trajectories which reach end with zero velocity
      }
      else
        cost +=1;  //If the stopping profile is chosen before reaching end then increase its weight so that it does not dominate other profiles
    }

    tgt.cost += cost;
    tgt.path = m_sampled_traj;
    //std::cout << "cost in planner "<<tgt.cost << " ret cost "<<cost << '\n';
    return cost;
  }//end of create trajectory


}//end of namespace
