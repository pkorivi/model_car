
//#include "MotionPlanner.h"

namespace fub_motion_planner{

  /** Function for obstacle collision check checking
  **
  ** @param msg
  */

  double MotionPlanner::CollisionCheck(VehicleState current_state,std::vector<double> s_pts,std::vector<double> d_pts, std::vector<double> d_coeffs){
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
  				if (std::isnan(obstVel.x)) //check if velocity is nan.
  				{
  					ROS_ERROR("Obstacle %d 's speed is NAN!!!",obst.id);
  					obstVel.x=0;
  				}
          FrenetCoordinate obst_frenet =  m_vehicle_path.getFenet(obstPos,yaw);

          //Obstacle direction -0 default - something ahead is moving laterally - someone driving across or walking across
          int direction = 0;
          //TODO Check if the angle should be wrt to road ro vehicle? May be vehicle then only the intersections thing work else it will be screwed up
          //if the angle betwen vehicle and road is between -pi/3 to pi/3 then its driving along the road direction = +1
          if (fabs(obst_frenet.th)<M_PI/3) {
            direction = 1;
          }else if(fabs(obst_frenet.th)>2*M_PI/3){ // If it is between -2/3pi and 2/3pi then it is driving opposite to the road -1.
            direction = -1;
          }else{ //Jay walking or vehicles driving cross at intersection
            direction =0;
          }
          //Bounding box y coordinate indicates width. thus width to left of center, right of center divided by 2
          double d_min_diff = kSafetyWidth + (fabs(obst.bounding_box_min.y)+fabs(obst.bounding_box_max.y))/2;
          //Increase the lateral safety distance by lane width such that if he is here far end of other lane then onöly drive
          if ((direction == 0) && (obstVel.x > kMinMovingObstvel)) {
            d_min_diff += 0.25;
          }
          //Obstacle location over look ahead time
          std::vector<double> obst_s;
          for (size_t i = 0; i <= kLookAheadTime; i++) {
            double nxt_pt_s = obst_frenet.s+ obstVel.x*i*direction;
            obst_s.push_back(nxt_pt_s>0?nxt_pt_s:0);
          }

          double ego_vehicle_start = polyeval_m(d_coeffs,s_pts.front());
          //TODO - Remove this obstacle path debug
          nav_msgs::Path m_obst_traj;
          m_obst_traj.header.stamp = ros::Time::now();
          m_obst_traj.header.frame_id = "/map";
          std::vector<int> times_check = {0,2,4,5};
          for (size_t i = 0; i <= kLookAheadTime; i++) {
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
          }
          obst_path_1.publish(m_obst_traj);
          //end of obstacle traj publishing
          /* TODO - Remove
          std::cout << "debug" << '\n';
          std::cout << "obst s " <<obst_s[0] <<" "<<obst_s[1] <<" "<<obst_s[2] <<" "<<obst_s[3] <<" "<<'\n';
          */
          //If the obstacle is in same lane and behind ego vehcile dont consider it for collision check
          if(obst_s[0] < s_pts[0] && fabs(ego_vehicle_start - obst_frenet.d)<d_min_diff){
            //std::cout << "obst behind car" << '\n';
            continue; //go to next obstacle
          }
          //Its not behind, check for collision
          else {
            //check for intersection in s -  min, max of obst travel distance to accomodate obstacles in opposite direction
            std::vector<double> intersection = {std::max(*std::min_element(obst_s.begin(),obst_s.end())-kSafetyDist,s_pts.front()-kSafetyDist),
                                                std::min(*std::max_element(obst_s.begin(),obst_s.end())+kSafetyDist,s_pts.back()+kSafetyDist)};
            //check for collision in s
            if (intersection.back()<intersection.front()) {
              //cost =  0; // No intersection in s, no potential collision
              continue; //go to next obstacle
            }
            else{ //check for collision in d where s is intersecting
              std::cout <<"obst Id "<<obst.id <<" x, y "<<obstPos[0]<<" "<<obstPos[1]<<" s, d th"<<obst_frenet.s<<" "<<obst_frenet.d<<" "<<obst_frenet.th <<" vel "<<obstVel.x<<" direc "<<direction <<'\n';
              double d_val1 = polyeval_m( d_coeffs,intersection.front());
              double d_val2 = polyeval_m( d_coeffs,intersection.back());
              //TODO add the dmin and dmax by finding min max of the d, if s positions for min max falls in the intersection
              std::cout << "intersection  "<<intersection.front()<<"  "<<intersection.back() <<"  d_ego "<<d_val1<<" "<<d_val2<<" obs_D "<<obst_frenet.d << '\n';
              if((fabs(d_val1 - obst_frenet.d)> d_min_diff)&&   //start of intersection
                  (fabs(d_val2 - obst_frenet.d)> d_min_diff)&&  //end of intersection
                  (fabs((d_val1+d_val2)/2 - obst_frenet.d)> d_min_diff)){ //in middle of intersection also road is free
                    continue; //go to next obstacle
                  }
              else{ //collision in s and d, //TODO may be remove this kMinMovingObstvel and make zero
                if(obstVel.x>kMinMovingObstvel && direction !=0){ // for collision in t for dynamic obstacles
                  //Check for timing - dynamic obstacle - static obstacle not needed consider it collision
                  double pt_duration = kLookAheadTime/(s_pts.size()-1);
                  size_t i=0;
                  double ego_t1 = 0,obst_t1 =0;// assign to minimum time - change based on need
                  for (i = 0; i < s_pts.size(); i++) {
                    if((intersection.front()<=s_pts[i]) && (fabs(d_pts[i] - obst_frenet.d)<= d_min_diff)){
                      i = (i>0?i-1:i); //If the intersection is less than the first element consider 0 time
                      ego_t1 = i*pt_duration; //Time of Intersection for ego vehicle
                      obst_t1 = fabs(s_pts[i] - obst_frenet.s)/obstVel.x ; // s_@ = s_0 + vel*time // Time of intersection for obstacle
                      break;
                    } //found where time starts
                  }

                  double ego_t2 = kLookAheadTime, obst_t2 = kLookAheadTime; // Assign to max
                  //check from back as the value is mostl likely in the end, size-1 representing last element
                  for (i = s_pts.size()-1; i >=0; i--) {
                    if((intersection.back()>=s_pts[i]) && (fabs(d_pts[i] - obst_frenet.d)<= d_min_diff)){
                      ego_t2 = (i)*pt_duration; // Margin as higher end of time
                      obst_t2 = fabs(s_pts[i] - obst_frenet.s)/obstVel.x ; //TODO safer to keep here i instead of i+1 while behind the obstacle and +1 while infront of obstacle choose what to do
                      break;
                    } //found where time starts
                  }
                  /*
                  //TODO Enable this code for dynamic obstacle checking - to work better
                  double ego_velocity = (s_pts[i]-s_pts[i-1])/pt_duration;
                  ego_velocity = ego_velocity>0.1?ego_velocity:0.1; //No reason just making it 0.1 such that there is some traction
                  double ego_t1_f = 0 ,ego_t1_b=0,ego_t2_f=0,ego_t2_b=0;
                  double obst_t1_f =0,obst_t1_b=0,obst_t2_f=0,obst_t2_b=0;

                  ego_t1_f = ego_t1 - kLengthForward/ego_velocity;
                  ego_t1_b = ego_t1 + kLengthBackward/ego_velocity;
                  ego_t2_f = ego_t2 - kLengthForward/ego_velocity;
                  ego_t2_b = ego_t2 + kLengthBackward/ego_velocity;

                  double obstacle_length_half = (fabs(obst.bounding_box_min.x)+fabs(obst.bounding_box_max.x))/2;
                  obst_t1_f = obst_t1 - obstacle_length_half/obstVel.x;
                  obst_t1_b = obst_t1 + obstacle_length_half/obstVel.x;
                  obst_t2_f = obst_t2 - obstacle_length_half/obstVel.x;
                  obst_t2_b = obst_t2 + obstacle_length_half/obstVel.x;

                  double t1_diff = std::min(ego_t1_f-obst_t1_f,ego_t1_f- obst_t1_b,ego_t1_b-obst_t1_f,ego_t1_b- obst_t1_b);
                  double t2_diff = std::min(ego_t2_f-obst_t2_f,ego_t2_f- obst_t2_b,ego_t2_b-obst_t2_f,ego_t2_b- obst_t2_b);
                  //chnage to below if
                  //if(t1_diff*t2_diff>0)
                  //double minimum_time_diff = std::min(fabs(t1_diff),fabs(t2_diff));
                  //*/
                  std::cout << "ego t "<<ego_t1<<" "<<ego_t2 <<"  obs t "<<obst_t1<<" "<<obst_t2 <<'\n';
                  if((ego_t1-obst_t1)*(ego_t2-obst_t2) >0 ){
                    //Same sign for time difference - No collision
                    double minimum_time_diff = std::min(fabs(ego_t1-obst_t1),fabs(ego_t2-obst_t2));
                    //If the  minimum time difference is >2s then all good, safe to derivative
                    //If the time is less than 2, add a proportional cost to how close it gets to obstacle
                    cost += (minimum_time_diff>kSafetyTimeDiff)?0:(kSafetyTimeDiff-minimum_time_diff)*kSafetyTimeDiffCostMul; //add cost to all obstacles
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
