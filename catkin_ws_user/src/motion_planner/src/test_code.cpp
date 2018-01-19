/*
Code for creating the approximation matrix
TODO - Adjust the weights to give proper predictions
*/


class target_state{
  public:
    target_state(double, double, double, double);
    //~target_state();
    double d;
    double s;
    double v;
    double cost;
};
//Constructor with initializer
target_state::target_state(double d, double s, double v, double cost){
  this->d = d;
  this->s = s;
  this->v = v;
  this->cost =cost;
}

int main(){

  double current_vel = 0;
  double target_vel = 1.1;
  double current_d = 0.0;
  double prev_tgt_d = 0;
  double target_d = 0.2;
  double target_S = 5;

  std::vector<target_state> cost;
  double cost_val;
  //target lateral shifts
  std::vector<double> d_ranges = {-0.25,-0.17,-0.1,0,0.1,0.17,0.25};
  //-0.1,0,0.1,0.17,0.25};
  //Acceleration to Max Deceleration
  std::vector<double> acc_prof = {0.2,0.1,0,-0.1,-0.2,-0.4,-0.6, -0.8};
  //double acc_prof[] = {0.2,0.1,0,-0.1,-0.2,-0.4,-0.6, -0.8};

  //TODO - add some variable or something
  for(int j=0; j<5;j++){
    for(int i=0; i< d_ranges.size();i++){
      cost_val = (current_vel-target_vel)*acc_prof[j]+ fabs(acc_prof[j]) + \
                  fabs(target_d - d_ranges[i])*2 + fabs(prev_tgt_d - d_ranges[i]);
      target_state tgt(target_d,target_S,target_vel, cost_val);
      std::cout.width(5);
      cost.push_back(tgt);
      std::cout << cost_val<< "   ";
    }
    std::cout <<'\n';
  }
  std::cout<<"Stopping profiles" <<'\n';
  //Profiles for stopping
  target_vel =0;
  for(int j=3; j<8;j++){
    for(int i=0; i< d_ranges.size();i++){
      cost_val = (current_vel-target_vel)*acc_prof[j]+ 2*fabs(acc_prof[j]) + \
                  fabs(target_d - d_ranges[i])*2 + fabs(prev_tgt_d - d_ranges[i]);
      target_state tgt(target_d,target_S,target_vel, cost_val);
      std::cout.width(5);
      cost.push_back(tgt);
      std::cout << cost_val<< "   ";
    }
    std::cout <<'\n';
  }

  //srand(3);
  /*
  for(int j=3; j<8; j++){
    //lateral shift is same as old
    cost_val = (current_vel-target_vel)*acc_prof[j]+ fabs(acc_prof[j]) + fabs(target_d - current_d) + fabs(prev_tgt_d - current_d);
    target_state tgt(current_d,target_S,target_vel, cost_val);
    cost.push_back(tgt);
    std::cout << cost_val <<'\n';
  }*/


 return 0;
}

/*End of code for creating approximation matrix*/




//ROS_INFO("timer, pos x %f",m_vehicle_state.m_vehicle_position[0]);
//TODO Test all the functions implemented in vehicle state and vehicle path
/* //Vehicle state stuff works
ROS_INFO("VS : pose %f %f",current_vehicle_state.m_ego_state_pose.pose.position.x,current_vehicle_state.m_ego_state_pose.pose.position.y);
ROS_INFO("VS : posi %f %f %f", current_vehicle_state.m_vehicle_position[0],current_vehicle_state.m_vehicle_position[1],current_vehicle_state.m_vehicle_position[2]);
ROS_INFO("VS: time %f ",current_vehicle_state.m_last_odom_time_stamp_received.toSec());
ROS_INFO("VS : %f, yaw : %f",current_vehicle_state.m_current_speed_front_axle_center,current_vehicle_state.getVehicleYaw());
*/

tf::Point a =tf::Point{0.35,-0.23,0.0};
tf::Point b =tf::Point{2.43,0.26,0.0};
tf::Point c =tf::Point{3.7,-2.6,0.0};
tf::Point d =tf::Point{5.4,-1.6,0.0};
/*
for (size_t i = 0; i < m_vehicle_path.xy_path.size(); i++) {
  ROS_INFO("x,y : (%f,%f)  s,d,k : (%f,%f,%f)",m_vehicle_path.xy_path[i][0], \
          m_vehicle_path.xy_path[i][1],m_vehicle_path.frenet_path[i].s,\
          m_vehicle_path.frenet_path[i].d,m_vehicle_path.frenet_path[i].k );
}
ROS_INFO("slope %f",m_vehicle_path.slope(a,b));
ROS_INFO("slope %f",m_vehicle_path.slope(b,a));
ROS_INFO("distance %f",m_vehicle_path.distance(a,b));
ROS_INFO("closestWayPoint %d",m_vehicle_path.closestWayPoint(a));
ROS_INFO("closestWayPoint %d",m_vehicle_path.closestWayPoint(b));
ROS_INFO("NextWayPoint %d",m_vehicle_path.NextWayPoint(a,0));
ROS_INFO("NextWayPoint %d",m_vehicle_path.NextWayPoint(b,0));
*/

tf::Point cp;
cp =tf::Point{0.1,0.20,0.0};
int i =1;
FrenetCoordinate p2 = m_vehicle_path.getFenet(cp,0.5);
ROS_INFO("%lu frenet %f,%f,%f,%f ",i,p2.s,p2.d,p2.k, p2.th);
p2 = m_vehicle_path.getFenet(cp,-0.5);
ROS_INFO("%lu frenet %f,%f,%f,%f ",i,p2.s,p2.d,p2.k, p2.th);
cp =tf::Point{4.97,-0.65,0.0};
p2 = m_vehicle_path.getFenet(cp,0.0);
ROS_INFO("%lu frenet %f,%f,%f,%f ",i,p2.s,p2.d,p2.k, p2.th);
p2 = m_vehicle_path.getFenet(cp,2.0);
ROS_INFO("%lu frenet %f,%f,%f,%f ",i,p2.s,p2.d,p2.k, p2.th);

std::vector<FrenetCoordinate> vec_fre;
vec_fre.push_back(FrenetCoordinate(0.1,0,0));
vec_fre.push_back(FrenetCoordinate(4.8,0.56,0));
vec_fre.push_back(FrenetCoordinate(5.7,-0.15,0));
//TODO - debug why wrong values for 8.1, what line fault is causing this error
vec_fre.push_back(FrenetCoordinate(8.1,-0.15,0));
for (size_t i = 0; i < vec_fre.size(); i++) {
  tf::Point p1 = m_vehicle_path.getXY(vec_fre[i]);
  ROS_INFO("%d xy %f,%f,%f ",i,p1[0],p1[1],p1[2]);
  FrenetCoordinate f1 = m_vehicle_path.getFenet(p1,0);
  ROS_INFO("%d frenet %f,%f,%f ",i,f1.s,f1.d,f1.k);
}

/*
std::vector<tf::Point> vec_xy ;
vec_xy.push_back(a);vec_xy.push_back(b);vec_xy.push_back(c);vec_xy.push_back(d);
for (size_t i = 0; i < vec_xy.size(); i++) {
  FrenetCoordinate p2 = m_vehicle_path.getFenet(vec_xy[i],0);
  ROS_INFO("%d frenet %f,%f,%f ",i,p2.s,p2.d,p2.k);
  tf::Point p3 = m_vehicle_path.getXY(p2);
  ROS_INFO("%d xy %f,%f,%f ",i,p3[0],p3[1],p3[2]);
}*/



std::vector<tf::Point> vec_xy ;
std::vector<double> vec_xy_slope ;
 vec_xy.push_back(tf::Point{3.46,-0.0,0.0});vec_xy_slope.push_back(0.01);
 vec_xy.push_back(tf::Point{5.07,-1.31,0.0});vec_xy_slope.push_back(-1.55);
 vec_xy.push_back(tf::Point{4.003,-2.78,0.0});vec_xy_slope.push_back(-2.59);
 vec_xy.push_back(tf::Point{2.28,-3.81,0.0});vec_xy_slope.push_back(-2.61);
 vec_xy.push_back(tf::Point{5.180000,-1.590000, 0.0});vec_xy_slope.push_back(-1.89);
 vec_xy.push_back(tf::Point{4.5600000,-2.420000, 0.0});vec_xy_slope.push_back(-2.59);
 vec_xy.push_back(tf::Point{3.110000,-3.630000, 0.0});vec_xy_slope.push_back(-2.61);
 vec_xy.push_back(tf::Point{1.710000,-4.350000, 0.0});vec_xy_slope.push_back(-2.63);

for (size_t i = 0; i < vec_xy.size(); i++) {
  FrenetCoordinate p2 = m_vehicle_path.getFenet(vec_xy[i],vec_xy_slope[i]);
  ROS_INFO("%lu frenet %f,%f,%f ",i,p2.s,p2.d,p2.k);
  tf::Point p3 = m_vehicle_path.getXY(p2);
  ROS_INFO("%lu xy %f,%f,%f ",i,p3[0],p3[1],p3[2]);
}






//Old motion planner
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
  }



This takes a lot of time. Improve this code for having an excution time of around 1ms compared to current 3ms.
If possible look at improving further. 30ms for 300 trajs. It should be short 0.1ms best

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
      //listener.lookupTransform("/turtle2", "/turtle1",ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
    }

    //current values in Map frame
    tf::Point cp ;//= current_state.m_vehicle_position;
    cp[0] =  pt_stamped_out.point.x;
    cp[1] =  pt_stamped_out.point.y;
    //std::cout << "map transformed cp "<< cp[0]<<" , "<<cp[1] << '\n';
    double c_yaw = current_state.getVehicleYaw();

    double time_from_prev_cycle = (current_state.m_last_odom_time_stamp_received - prev_state.m_last_odom_time_stamp_received).toSec();

    int number_of_samples = 20;
    Eigen::VectorXd spts(number_of_samples);
    Eigen::VectorXd tpts(number_of_samples);
    Eigen::VectorXd vpts(number_of_samples);
    Eigen::VectorXd acc_pts(number_of_samples);
    //TODO change the number
    Eigen::VectorXd dpts(6);

    double a_ref, v_ref, s_ref;
    //Accelerate with target Acceleration
    enum AccStates { CONSTANT_ACC, ZERO_ACC, BRAKE_DEC};
    AccStates c_acc_phase = CONSTANT_ACC;
    //current point in frenet
    FrenetCoordinate frenet_val =  m_vehicle_path.getFenet(cp,c_yaw);
    //ROS_INFO("map-xy %.3f,%.3f , odom x,y : %.3f,%.3f , cur a: 0 v: %.3f ",cp[0],cp[1],current_state.m_vehicle_position[0],current_state.m_vehicle_position[1],v_current);
    //ROS_INFO("frenet s,d %.3f %.3f ", frenet_val.s, frenet_val.d);
    //Initial time and
    spts[0] = (frenet_val.s);
    dpts[0] = (frenet_val.d);
    vpts[0] = (v_current);
    tpts[0] = (0);
    acc_pts[0] = (0);//Assume previous acceleration is zero - bad assumption but accurate values are not possible to measure with current velocity calculation accuracy
    //TODO - add in config file
    //If the target distance is short the planner should accelerate and deccelerate in 5s, else it will not be able to find a path.
    //This will help create a path with ability to stop if the final destination is arrived
    double brake_dec = 0.3;
    double d_brake =(v_current*v_current)/(2*brake_dec);//v² -u² = 2as, thus to stop with current velocity it is s = -u²/2a; a is -ve this s = u²/2a

    //to come to zero acc, it should decrease if it is greater than zero and increase if its less than zero
    //TODO zero stuff here- something can be messy
    double to_zero_acc_inc_dec = (a_target>0?-1:1);
    //Time samples of 100ms each, so for 5 seconds we have 50 samples - TODO this as tunable parameter
    double t_sample = (5.0)/number_of_samples;
    for(int i=1;i<number_of_samples;i++){
      //std::cout << "vcur : " << vpts[i-1]<< " acur :"<< acc_pts[i-1] <<"  vtgt "<<v_target<<" a_tgt : "<<a_target << " vch "<<v_change <<'\n';
      //std::cout << "abs v : " << fabs(v_target-vpts[i-1])<< " abs a : " <<fabs(acc_pts[i-1]-acc[2]);
      tpts[i] =(i*t_sample);
      switch (c_acc_phase) {
        case CONSTANT_ACC: {
          //std::cout << "  :  const acc" << '\n';
          //constant acceleration
          acc_pts[i]=(a_target);
          v_ref = vpts[i-1] + a_target*t_sample;
          // Bound the velocity
          if (v_ref > v_max)
            v_ref = v_max;
          else if (v_ref<v_min)
            v_ref = v_min;

          //If the v_ref reaches near v_target, make it v_target - this is mainly useful while acceleration and prevent overshoots
          if(fabs(v_target-v_ref) < 0.03)
            v_ref = v_target;
          vpts[i]=(v_ref);

          //calc distance only when there is speed
          if(v_ref>0)
            spts[i]=(spts[i-1] + vpts[i-1]*t_sample + 0.5*a_target*t_sample*t_sample);
          else
            spts[i] = spts[i-1];


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
          acc_pts[i]=(0);
          v_ref = vpts[i-1];//zero acceleration - constant velocity
          // Bound the velocity
          if (v_ref > v_max)
            v_ref = v_max;
          else if (v_ref<v_min)
            v_ref = v_min;
          vpts[i]=(v_ref);

          //calc distance only when there is speed
          if(v_ref>0)
            spts[i]=(spts[i-1] + v_ref*t_sample);
          else
            spts[i] = spts[i-1];

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
          acc_pts[i]=(a_target);
          v_ref = vpts[i-1] - brake_dec*t_sample;
          // Bound the velocity
          if (v_ref > v_max)
            v_ref = v_max;
          else if (v_ref<v_min)
            v_ref = v_min;
          vpts[i]=(v_ref);
          if(v_ref>0)
            spts[i]=(spts[i-1] + vpts[i-1]*t_sample - 0.5*brake_dec*t_sample*t_sample);
          else
            spts[i] = spts[i-1];

          break;
        }
        default: {
          std::cout << "Oops something is wrong" << '\n';
        }
      }
    //ROS_INFO("%.3f",spts[i]);//TODO remove
    }//for loop
    //std::cout << "v_change :" << v_change << '\n';
    //int polynomial_order=4;
    auto s_coeffs = polyfit(tpts, spts,polynomial_order);
    //auto v_coeffs = polyfit(tpts,vpts,polynomial_order);
    //auto a_coeffs = polyfit(tpts,acc_pts,polynomial_order);


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
    double s_val,d_val,v_val, a_val, s_prev;
    s_prev = polyeval( s_coeffs, -0.2);
    //sample every 0.2s
    for(double i=0;i<25;i++){
      double t_pt = 0.2*i;//time
      //double s_val = s(t_pt);
      //double d_val = d(t_pt);
      //Eigen::VectorXd
      s_val = polyeval( s_coeffs, t_pt);
      d_val = polyeval( d_coeffs, t_pt);
      v_val = (s_val - s_prev)/0.2;
      s_prev = s_val;
      if(v_val<v_min)
        v_val = v_min;

      //v_val = polyeval( v_coeffs, t_pt);
      //a_val = polyeval( a_coeffs, t_pt);

      tf::Point xy = m_vehicle_path.getXY(FrenetCoordinate(s_val,d_val,0)); //TODO check yaw stuff
      //TODO - remove this
      //ROS_INFO("xy %.3f,%.3f , s,d %.3f, %.3f , a: %.3f v: %.3f ",xy[0],xy[1], s_val, d_val, a_val, v_val);
      geometry_msgs::PoseStamped examplePose;
      examplePose.pose.position.x = xy[0];
      examplePose.pose.position.y = xy[1];
      //Currently this velocity is used in trajectory converted to publish velocity at a point
      examplePose.pose.position.z = v_val;//v(t_pt); //velocity saved in z direction
      examplePose.pose.orientation.x = 0.0;//0.0f;//a(t_pt); // save accleration in orientation
      examplePose.pose.orientation.y = 0.0f;
      examplePose.pose.orientation.z = 0.0f;

      //push PoseStamped into Path
      m_sampled_traj.poses.push_back(examplePose);
    }
    //Publish as path with velocity in z dimension
    traj_pub.publish(m_sampled_traj);

  }//end of create trajectory
