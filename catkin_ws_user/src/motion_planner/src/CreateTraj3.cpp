namespace fub_motion_planner{
  // Evaluate a polynomial.
  double polyeval_m(std::vector<double> coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
      result += coeffs[i] * pow(x, i);
    }
    return result;
  }
  /*
  std::vector<double> evaluate_d_coeffs(double d_cur,double d_tgt, double poly_order){
    //TODO change the number
    clock_t tStart = clock();
    std::vector<double> dpts = {d_cur,d_cur,d_cur,d_tgt,d_tgt,d_tgt};
    //TODO make is dependent on s - implement as per paper
    std::vector<double> d_t_pts = {0.0,0.2,0.4,4.6,4.8,5.0};
    auto d_coeffs =  polyfit(d_t_pts,dpts,poly_order);
    //ROS_INFO("polyfit d: %f\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    return d_coeffs;
  }*/
  /*
  car current d, target d, and angle between the road and the car, s_i initial s, sf final s
  following as per  'REAL-TIME TRAJECTORY PLANNING FOR AUTONOMOUS URBAN DRIVING paper'
  */
  std::vector<double> evaluate_d_coeffs(double d_cur,double d_tgt, double theta, double s_i, double s_f){
    //TODO change the number
    clock_t tStart = clock();
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
    //ROS_INFO("polyfit d: %f\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    return d_coeffs;
  }

  double MotionPlanner::create_traj_const_acc_xy_spline_3(VehicleState current_state,VehicleState prev_state, ros::Publisher&  traj_pub, \
          double v_max, double v_min,int polynomial_order, target_state &tgt){

    double v_target= tgt.v_tgt;
    double a_target= tgt.a_tgt;
    double d_target= tgt.d_eval;
    double s_target = tgt.s_tgt;
    double  cost =0;
     //TODO use this instead of end of the path
    clock_t tStart = clock();
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
    std::vector<double> spts;
    std::vector<double> dpts;
    std::vector<double> tpts;
    std::vector<double> xpts;
    std::vector<double> ypts;
    std::vector<double> vpts;

    double a_ref, v_ref, s_ref;
    //Different Acceleraton States
    enum AccStates { CONSTANT_ACC, ZERO_ACC, BRAKE_DEC};
    AccStates c_acc_phase = CONSTANT_ACC;
    //current point in frenet
    //ROS_INFO("map-xyz %.3f,%.3f, %.3f , odom x,y : %.3f,%.3f , v: %.3f , yaw = %.3f",cp[0],cp[1],cp[2],current_state.m_vehicle_position[0],current_state.m_vehicle_position[1],v_current,c_yaw);
    FrenetCoordinate frenet_val =  m_vehicle_path.getFenet(cp,c_yaw);
    //ROS_INFO("frenet s,d %.3f %.3f ", frenet_val.s, frenet_val.d);
    //Initial Points for polyfit
    spts.push_back(frenet_val.s);
    tpts.push_back(0);
    //Iniial x,y - as per map coordinates - TODO .... as of now updating everything in below for loop of populating x,y
    //dpts.push_back(frenet_val.d);
    //xpts.push_back( pt_stamped_out.point.x);
    //ypts.push_back( pt_stamped_out.point.y);
    vpts.push_back(v_current);
    //std::cout << "current x,y "<<xpts[0]<<"  "<<ypts[0] << '\n';
    //ROS_INFO("Initialization: %f\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    tStart = clock();
    //TODO - add in config file
    //If the target distance is short the planner should accelerate and deccelerate in 5s, else it will not be able to find a path.
    //This will help create a path with ability to stop if the final destination is arrived
    double brake_dec = 0.3;
    double d_brake =(v_current*v_current)/(2*brake_dec);//v² -u² = 2as, thus to stop with current velocity it is s = -u²/2a; a is -ve this s = u²/2a
    //Time samples of 100ms each, so for 5 seconds we have 50 samples - TODO this as tunable parameter
    double t_sample = kLookAheadTime/(kNumberOfSamples-1);
    double v_previous = v_current;
    for(int i=1;i<kNumberOfSamples;i++){
      //std::cout << "vcur : " << vpts[i-1]<< " acur :"<< acc_pts[i-1] <<"  vtgt "<<v_target<<" a_tgt : "<<a_target << " vch "<<v_change <<'\n';
      //std::cout << "abs v : " << fabs(v_target-vpts[i-1])<< " abs a : " <<fabs(acc_pts[i-1]-acc[2]);
      tpts.push_back(i*t_sample);
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

          spts.push_back((v_ref>0)?(spts[i-1] + v_previous*t_sample + 0.5*a_target*t_sample*t_sample):spts[i-1]);
          d_brake =(v_ref*v_ref)/(2*brake_dec);
          //0.1 of  extra buffer stopping distance
          // Go to braking if the available road is less and the acceleration requested is greater then or equal to zero.
          // If braking is requested then keep this as a part of cost term
          //TODO - add constants in config file
          //TODO changed a_tgt to v_tgt - its working fine, just check if its ok - may affect the deceleration protion
          if((d_brake > (s_target - spts[i] - 0.1))&&(v_target>0)){
            c_acc_phase =BRAKE_DEC;
            cost += 2.0; //TODO how much to add, need to clarify
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
          //std::cout << "  :  decc acc" << '\n';
          //constant acceleration
          v_ref = v_previous - brake_dec*t_sample;
          // Bound the velocity
          v_ref = v_ref>v_max?v_max:v_ref;
          v_ref = v_ref<v_min?v_min:v_ref;
          spts.push_back((v_ref>0)?(spts[i-1] + v_previous*t_sample - 0.5*brake_dec*t_sample*t_sample):spts[i-1]);
          break;
        }
        default: {
          std::cout << "Oops something is wrong" << '\n';
        }
      }
      //ROS_INFO("%.3f",spts[i]);//TODO remove
      //Store the velocity calculated in this cycle for next cycle
      v_previous = v_ref;
      vpts.push_back(v_ref);
      //std::cout <<"  state "<< c_acc_phase <<" s "<<spts[i]<<" d_bk "<<d_brake <<" v "<<v_ref<< '\n';
    }//for loop
    //ROS_INFO("for loop: %f\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    tStart = clock();
    //d_calc
    //auto d_coeffs = evaluate_d_coeffs(frenet_val.d,d_target, polynomial_order);
    std::vector<double> d_coeffs = evaluate_d_coeffs(frenet_val.d,d_target,frenet_val.th, spts.front(), spts.back());
    //ROS_INFO("d:  %.3f ,%.3f, %.3f, %.3f   th : %.3f",d_coeffs[0],d_coeffs[1],d_coeffs[2],d_coeffs[3],frenet_val.th);

    //ECL ARrays for ecl splines
    ecl::Array<double> ecl_ts(kNumberOfSamples);
    ecl::Array<double> ecl_x(kNumberOfSamples);
    ecl::Array<double> ecl_y(kNumberOfSamples);
    //create xy sample points
    //TODO - make d as a function of s. At high speeds d can be function of time
    //At low speeds d should be function of s to ensure curvature
    for (size_t i = 0; i < kNumberOfSamples; i++) {
      //double d_val1 = polyeval_m( d_coeffs, tpts[i]);
      double d_val1 = polyeval_m( d_coeffs, spts[i]);
      dpts.push_back(d_val1);
      tf::Point xy = m_vehicle_path.getXY(FrenetCoordinate(spts[i],d_val1,0,0)); //TODO check yaw stuff
      xpts.push_back(xy[0]);
      ypts.push_back(xy[1]);

      //ECL spline poilts
      ecl_x[i] = xy[0];
      ecl_y[i] = xy[1];
      ecl_ts[i] = i*t_sample;
      //std::cout << "x,y  "<<xy[0]<<"  "<<xy[1] << " vel  "<< vpts[i] <<"   time "<<tpts[i]<<'\n';
    }

    cost += CollisionCheck(current_state,spts,dpts,tpts,d_coeffs);
    //ROS_INFO("frenet to xy conversion: %f\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    tStart = clock();

    //Fit the points path points to a polynomial of given order
    //std::cout << "sizes : "<< tpts.size()<<" "<< xpts.size()<<" "<< ypts.size()<<" " << '\n';
    /* TODO - replacing with spline
    auto x_coeffs =  polyfit(tpts, xpts,polynomial_order);
    auto y_coeffs =  polyfit(tpts, ypts,polynomial_order);
    auto v_coeffs =  polyfit(tpts, vpts,polynomial_order);
    */

    /* Replacing with ECL splines
    tk::spline x;
    tk::spline y;
    tk::spline v;
    x.set_points(tpts,xpts);
    y.set_points(tpts,ypts);
    v.set_points(tpts,vpts);
    */
    ecl::CubicSpline mSpline_x;
    ecl::CubicSpline mSpline_y;
    ecl::CubicSpline mSpline_z;
    //TODO - last two points are change in velocity for x, y- follow as per fub controller soon
    mSpline_x = ecl::CubicSpline::ContinuousDerivatives(ecl_ts, ecl_x, v_current*cos(c_yaw), (ecl_x[kNumberOfSamples-1] - ecl_x[kNumberOfSamples-2])/t_sample);
    mSpline_y = ecl::CubicSpline::ContinuousDerivatives(ecl_ts, ecl_y, v_current*sin(c_yaw), (ecl_y[kNumberOfSamples-1] - ecl_y[kNumberOfSamples-2])/t_sample);
    //mSpline_z = ecl::CubicSpline::ContinuousDerivatives(x_set, y_set_z, frontVelocity.getZ(), backVelocity.getZ());

    //ROS_INFO("polyfit x,y: %f\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    tStart = clock();

    nav_msgs::Path m_sampled_traj;
    m_sampled_traj.header.stamp = ros::Time::now();
    m_sampled_traj.header.frame_id = "/map";
    double s_val,d_val,v_val, a_val;
    //sample every 0.2s
    for(double i=0;i<26;i++){
      double t_pt = 0.2*i;//time
      /* - TODO replace with derivative of spline
      double x_der = polyeval_derivative(x_coeffs,t_pt);
      double y_der = polyeval_derivative(y_coeffs,t_pt);
      v_val = sqrt(x_der*x_der + y_der*y_der);
      */
      //a_val = polyeval_double_derivative(s_coeffs,t_pt);

      //TODO - remove this
      geometry_msgs::PoseStamped examplePose;
      double x_val = mSpline_x(t_pt);//x(t_pt);
      double y_val = mSpline_y(t_pt);//y(t_pt);
      double dv_x = mSpline_x.derivative(t_pt);
      double dv_y = mSpline_y.derivative(t_pt);
      double v_fit = sqrt(dv_x*dv_x + dv_y*dv_y); //v(t_pt);
      //ROS_INFO("xy %.3f,%.3f , s,d %.3f, %.3f , bs x,y: %.3f, %.3f vel %.3f",x_val, y_val, spts[i], dpts[i], xpts[i], ypts[i],v_fit);
      //ROS_INFO("xy %.3f,%.3f , v_xy, v_fit : %.3f  %.3f ", x_val, y_val, v_val, v_fit);
      examplePose.pose.position.x = x_val;
      examplePose.pose.position.y = y_val;
      //Currently this velocity is used in trajectory converted to publish velocity at a point
      examplePose.pose.position.z = v_fit;//i*t_sample;//v_fit;//v(t_pt); //velocity saved in z direction
      examplePose.pose.orientation.x = 0.0;//a_val;//0.0f;//a(t_pt); // save accleration in orientation //TODO - calculate double derivative for acceleration
      examplePose.pose.orientation.y = 0.0f;
      examplePose.pose.orientation.z = 0.0f;
      examplePose.pose.orientation.w = 1.0f;

      //push PoseStamped into Path
      m_sampled_traj.poses.push_back(examplePose);
    }
    //ROS_INFO("eval and pub traj: %f\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    tStart = clock();
    //Publish as path with velocity in z dimension
    traj_pub.publish(m_sampled_traj);
    //last point reached in s
    tgt.s_reched = spts[kNumberOfSamples-1];
    tgt.cost += cost;
    tgt.path = m_sampled_traj;
    //std::cout << "cost in planner "<<tgt.cost << " ret cost "<<cost << '\n';
    return cost;
  }//end of create trajectory


}//end of namespace
