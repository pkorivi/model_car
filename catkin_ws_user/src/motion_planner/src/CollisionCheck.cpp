
//#include "MotionPlanner.h"

namespace fub_motion_planner{

  /** Function for obstacle collision check checking
  **
  ** @param msg
  */

  double MotionPlanner::CollisionCheck(VehicleState current_state,std::vector<double> s_pts,std::vector<double> d_pts, std::vector<double> t_pts, std::vector<double> d_coeffs){
      //Move this to motion planner callback timer
      double cost = 0;
      if(current_state.m_obstacle_msg){
  			//tf::StampedTransform obstMapTf;
        tf::StampedTransform obstMapTfMsg;
        tf::Transform obstMapTf;
        for(const autonomos_obstacle_msgs::Obstacle & obst : current_state.m_obstacle_msg->obstacles ){
          //TODO remove std::cout << " obst vel "<<obst.abs_velocity.twist.linear.x << '\n';
          geometry_msgs::PoseStamped stamped_in, stamped_out;
          stamped_in.header = obst.odom.header;
          stamped_in.pose = obst.odom.pose.pose;
          //std::cout << "stamped_in  " <<stamped_in.pose.position.x<<"  "<<stamped_in.pose.position.y <<'\n';
          if(current_state.m_obstacle_msg->header.frame_id != obst.header.frame_id){
            try{
              m_tf_listener.listener.transformPose(current_state.m_obstacle_msg->header.frame_id,stamped_in,stamped_out);
            }
            catch (tf::TransformException &ex) {
              ROS_ERROR("%s",ex.what());
            }
            //std::cout << "stamped out transform" <<stamped_out.pose.position.x<<"  "<<stamped_out.pose.position.y <<'\n';
          }
          else{
            //No need to transform
            stamped_out = stamped_in;
          }
          //std::cout << "stamped out" <<stamped_out.pose.position.x<<"  "<<stamped_out.pose.position.y <<'\n';
  				tf::Point obstPos;
          obstPos[0] = stamped_out.pose.position.x;
          obstPos[1] = stamped_out.pose.position.y;
  				tf::Quaternion obstQuater;
          tf::quaternionMsgToTF(stamped_out.pose.orientation,obstQuater);
  				double roll, pitch, yaw;
  				tf::Matrix3x3 m(obstQuater);
          //tf::Matrix3x3 m(stamped_out.pose.orientation);
      		m.getRPY(roll, pitch, yaw);
  				// We need velocity of the obstacle to predicte the position of the obstacle
  				geometry_msgs::Vector3 obstVel;
  				obstVel=obst.abs_velocity.twist.linear;
  				if (obstVel.x!=obstVel.x) //check if velocity is nan.
  				{
  					ROS_ERROR("Obstacle %d 's speed is NAN!!!",obst.id);
  					obstVel.x=0;
  				}
          FrenetCoordinate obst_frenet =  m_vehicle_path.getFenet(obstPos,yaw);
          //std::cout << "obst x, y "<<obstPos[0]<<" "<<obstPos[1]<<" s, d "<<obst_frenet.s<<" "<<obst_frenet.d<<" vel "<<obstVel.x<< '\n';
          //TODO replace wih correct values
          //This should be wcar/2 + wobst/2+safe_dist
          double d_min_diff = 0.20;
          //Lets say W_obst =
          //Obstacle at 0, 2 ,4,5 sec
          std::vector<double> obst_s = {obst_frenet.s, obst_frenet.s+ obstVel.x*2,obst_frenet.s+ obstVel.x*4, obst_frenet.s+ obstVel.x*5 };
          std::vector<double> ego_vehicle_s = {s_pts[0],s_pts[10],s_pts[20],s_pts[25]};
          std::vector<double> ego_vehicle_d = {d_pts[0],d_pts[10],d_pts[20],d_pts[25]};
          //TODO - Remove this obstacle path debug
          nav_msgs::Path m_obst_traj;
          m_obst_traj.header.stamp = ros::Time::now();
          m_obst_traj.header.frame_id = "/map";
          std::vector<int> times_check = {0,2,4,5};
          for (size_t i = 0; i < 4; i++) {
            //TODO - remove or make it better
            tf::Point xy = m_vehicle_path.getXY(FrenetCoordinate(obst_s[i],obst_frenet.d,0,0));
            geometry_msgs::PoseStamped examplePose;
            examplePose.pose.position.x = xy[0];
            examplePose.pose.position.y = xy[1];
            //Currently this velocity is used in trajectory converted to publish velocity at a point
            examplePose.pose.position.z = 0;//times_check[i];//obstVel.x;//v(t_pt); //velocity saved in z direction
            examplePose.pose.orientation.x = 0.0;//a_val;//0.0f;//a(t_pt); // save accleration in orientation //TODO - calculate double derivative for acceleration
            examplePose.pose.orientation.y = 0.0f;
            examplePose.pose.orientation.z = 0.0f;
            examplePose.pose.orientation.w = 1.0f;
            //push PoseStamped into Path
            m_obst_traj.poses.push_back(examplePose);

            //std::cout << "obstacle "<<obst_s[i]<<" "<<obst_frenet.d<< " vehicle "<<ego_vehicle_s[i]<<" "<<ego_vehicle_d[i] << '\n';
          }
          obst_path_1.publish(m_obst_traj);
          //end of obstacle traj publishing
          /* TODO - Remove
          std::cout << "debug" << '\n';
          std::cout << "obst s " <<obst_s[0] <<" "<<obst_s[1] <<" "<<obst_s[2] <<" "<<obst_s[3] <<" "<<'\n';
          std::cout << "ego_vehicle_s " <<ego_vehicle_s[0] <<" "<<ego_vehicle_s[1] <<" "<<ego_vehicle_s[2] <<" "<<ego_vehicle_s[3] <<" "<<'\n';
          std::cout << "ego_vehicle_d" <<ego_vehicle_d[0] <<" "<<ego_vehicle_d[1] <<" "<<ego_vehicle_d[2] <<" "<<ego_vehicle_d[3] <<" "<<'\n';
          */
          //If the obstacle is behind ego vehcile dont consider it for collision check
          if(obst_s[0] < ego_vehicle_s[0] && fabs(ego_vehicle_d[0] - obst_frenet.d)<d_min_diff){
            //cost =0; //Obstacle is behind dont check for collision
            //std::cout << "obst behind car" << '\n';
            continue; //go to next obstacle
          }
          //Its not behind, check for collision
          else {
            //check for intersection in s
            const double kSafetyDist = 0.25; //TODO - config file or constant in .h file
            std::vector<double> intersection = {std::max(obst_s.front()-kSafetyDist,ego_vehicle_s.front()-kSafetyDist),
                                                std::min(obst_s.back()+kSafetyDist,ego_vehicle_s.back()+kSafetyDist)};
            //std::cout << "intersection  "<<intersection.front()<<"  "<<intersection.back() << '\n';
            //check for collision in s
            if (intersection.back()<intersection.front()) {
              //cost =  0; // No intersection in s, no potential collision
              continue; //go to next obstacle
            }
            else{ //check for collision in d where s is intersecting
              double d_val1 = polyeval_m( d_coeffs,intersection.front());
              double d_val2 = polyeval_m( d_coeffs,intersection.back());
              //std::cout << "d_ego "<<d_val1<<" "<<d_val2<<" obs_D "<<obst_frenet.d << '\n';
              if((fabs(d_val1 - obst_frenet.d)> d_min_diff)&&   //start of intersection
                  (fabs(d_val2 - obst_frenet.d)> d_min_diff)&&  //end of intersection
                  (fabs((d_val1+d_val2)/2 - obst_frenet.d)> d_min_diff)){ //in middle of intersection also road is free
                    continue; //go to next obstacle
                  }
              else{ //collision in s and d, //TODO may be remove this 0.05 and make zero
                if(obstVel.x>0.05){ // for collision in t for dynamic obstacles
                  //Check for timing - dynamic obstacle - static obstacle not needed consider it collision
                  double pt_duration = kLookAheadTime/(s_pts.size()-1);
                  size_t i=0;
                  double ego_t1 = 0,obst_t1 =0;// assign to minimum time - change based on need
                  for (i = 0; i < s_pts.size(); i++) {
                    if((intersection.front()<=s_pts[i]) && (fabs(d_pts[i] - obst_frenet.d)<= d_min_diff)){
                      i = (i>0?i-1:i); //If the intersection is less than the first element consider 0 time
                      ego_t1 = i*pt_duration; //Time of Intersection for ego vehicle
                      obst_t1 = (s_pts[i] - obst_frenet.s)/obstVel.x ; // s_@ = s_0 + vel*time // Time of intersection for obstacle
                      break;
                    } //found where time starts
                  }

                  double ego_t2 = kLookAheadTime, obst_t2 = kLookAheadTime; // Assign to max
                  //check from back as the value is mostl likely in the end
                  for (i = s_pts.size()-1; i >=0; i--) {
                    if((intersection.back()>=s_pts[i]) && (fabs(d_pts[i] - obst_frenet.d)<= d_min_diff)){
                      ego_t2 = (i+1)*pt_duration; // Margin as higher end of time
                      obst_t2 = (s_pts[i+1] - obst_frenet.s)/obstVel.x ;
                      break;
                    } //found where time starts
                  }

                  std::cout << "ego t "<<ego_t1<<" "<<ego_t2 << '\n';
                  std::cout << "obs t "<<obst_t1<<" "<<obst_t2 <<'\n';
                  if((ego_t1-obst_t1)*(ego_t2-obst_t2) >0 ){
                    //Same sign for time difference - No collision
                    double minimum_time_diff = std::min(fabs(ego_t1-obst_t1),fabs(ego_t2-obst_t2));
                    //If the  minimum time difference is >2s then all good, safe to derivative
                    //If the time is less than 2, add a proportional cost to how close it gets to obstacle
                    cost += (minimum_time_diff>2)?0:(2-minimum_time_diff)*3; //add cost to all obstacles
                  }//else cost of 50 defined initially will be returned
                  else{
                    cost += 50; //Collision with Dynamic Obstacle
                  }
                }//If condition for dynamic obstacle check
                else{
                  cost += 40; //Collision with static obstacle
                }
              }//collision check in time -when both in s,d there is collision
            }//collision check in d
          }//collision check in s
      } //for loop of all obstacles
    }// if obstacles
    else{ //No obstacles - so no cost
      cost = 0;
    }
    return cost;
  }//end of collision check

}//end of namespace

/* taken from //Its not behind, check for collision - add this portion later
for (size_t i = 0; i < 3; i++) {
  //check for s overlap
  //consider two ranges  [s1....e1], [s2....e2], they dont overlap if e1<s2 or e2<s1
  if(!(ego_vehicle_s[i+1] < obst_s[i] || obst_s[i+1]<ego_vehicle_s[i])){
    //s overlaps, check if they collidie by measuring difference between d
    if(fabs(d_pts[i] - obst_frenet.d)< d_min_diff || \
    fabs(d_pts[i+1] - obst_frenet.d)< d_min_diff ){
      //collision -  add more weight if collision is immediate ,
      //TODO  add car velocity at the moment component also in future - check the weight
      cost = 5 + (3-i);
    } //if for collision check in d
  } //if for
}//local loop for checking collision
*/
