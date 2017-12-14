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
