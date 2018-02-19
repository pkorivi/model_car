//TODO Create a calss to receive the route planner data and do all conversions for frenet and stuff.
// All fucntions to x,y. path, whats the width so much will be done here.

#include "VehiclePath.h"
namespace fub_motion_planner{
  VehiclePath::VehiclePath(){}
  VehiclePath::~VehiclePath(){}
  /** Setup function - generally called from init of the Nodelet to setup this class.
  **
  ** @param nodehandle
  */
  void VehiclePath::setup(ros::NodeHandle & nh){
      ROS_INFO("Vehicle_Path setup");
      m_subscribe_route_planner  = nh.subscribe("/route_planner/sub_path", 1, &VehiclePath::RoutePlannerCallback, this, ros::TransportHints().tcpNoDelay());
      path_with_speed_profiles = nh.advertise<nav_msgs::Path>("/path_speed_limits", 10);
  }
  /** Route planner callback - invoked when router planner publishes a path
  **
  ** @param msg containg path for robot to follow
  */
  void VehiclePath::RoutePlannerCallback(const nav_msgs::Path & msg){
    ROS_INFO("path_received");
    route_path_exists = true;
    m_path = msg;
    //Trigger transormation to frenet and store xy as points
    transformToXYandFrenet();
    //compute speed limit along the path
    calc_speed_limit();
  }

  /**
  ** Path is represented by number of line segments,  find the line segment to which
  ** frenet coordinate´s s belongs, then find the point on a line perpendicular to that line segment
  ** at lateral distnace d
  ** @param : Frenet Coordinate
  **/
  tf::Point VehiclePath::getXY(FrenetCoordinate frenet_pt){
    //ROS_INFO("fp s %f, d %f ",frenet_pt.s,frenet_pt.d);
    int next_wp = 0;
    //ROS_INFO("s: %f %f %f %f",frenet_path[next_wp].s,frenet_path[next_wp+1].s,frenet_path[next_wp+2].s,frenet_path[next_wp+3].s);
    //find the closest waypoint index behind the given frenet point
    //TODO check if >= is fine, made condition >= from >, to accomodate origin or zero distance
    //std::cout <<"get xy _ frenet s :  "<< frenet_pt.s << '\n';
    while((frenet_pt.s >= frenet_path[next_wp].s) && \
            (next_wp < (frenet_path.size()-1))){
      next_wp++;
    }
    if(next_wp>0){
      //TODO modify point at a distance also
      //next waypoint index
      int prev_wp = (next_wp-1)%frenet_path.size();
      //distance along this segment
      double seg_s = (frenet_pt.s- frenet_path[prev_wp].s);

      //https://math.stackexchange.com/questions/175896/finding-a-point-along-a-line-a-certain-distance-away-from-another-point
      //trying https://stackoverflow.com/questions/133897/how-do-you-find-a-point-at-a-given-perpendicular-distance-from-a-line
      //http://mathworld.wolfram.com/PerpendicularVector.html
      double dx = xy_path[next_wp][0] - xy_path[prev_wp][0];
      double dy = xy_path[next_wp][1] - xy_path[prev_wp][1];
      double dist = sqrt(dx*dx + dy*dy);//magnitude for unit vector in this direction
      // unit vector along the line described by the sampling points
      dx /= dist;
      dy /= dist;
      //Point at a distance x along the line
      double seg_x = xy_path[prev_wp][0] + (seg_s*dx); //normalizing for unit vector of dx.
      double seg_y = xy_path[prev_wp][1] + (seg_s*dy);
      //return a point at a perpendicular distance from a point - frenet distance is positive on right side and negative on left side
      // thus if the unit vector is (a,b) then perpendicular is (-b,a) but as frenet d values have opposite sign. we will negate here.
      return tf::Point{seg_x + (frenet_pt.d*dy), seg_y - (frenet_pt.d*dx),0.0};
    }
    else{
      ROS_INFO("ooops no point found - something wrong (s,d) %f %f ",frenet_pt.s,frenet_pt.d);
      return tf::Point{0,0,0};

    }
  }

  /**Function to calculate frenet coordinate
  **
  ** @param point in x,y coordinate and vehicle yaw
  */
  FrenetCoordinate VehiclePath::getFenet(tf::Point xy_pt, double theta){
    //the line segment closest to the current point
    int next_wp = NextWayPoint(xy_pt, theta);
    if(next_wp==0){
      ROS_INFO("Oops, the vehicle is away/behind start of route, next pt is made 1, x,y: %.3f %.3f, th %.3f",xy_pt[0],xy_pt[1],theta);
      //TODO - This is done if the source is origin then the next waypoint is zero - may be add condition for origin
      next_wp = 1;
    }
    int prev_wp = next_wp -1;
    //Then find the projection of point on the above found line segment.
    // Distance till projected point provides s and distance between projected
    // and actual point is d.
    //http://www.sunshine2k.de/coding/java/PointOnLine/PointOnLine.html#step5
    //https://math.la.asu.edu/~surgent/mat272/dotcross.pdf (refer to dot product and orthogonal projection sections)
    //vector formed by line segment on map lets say n
    double n_x = xy_path[next_wp][0] - xy_path[prev_wp][0];
    double n_y = xy_path[next_wp][1] - xy_path[prev_wp][1];
    //vector formed by the line joining prev_wp and the given point lets say m
    double m_x = xy_pt[0] -  xy_path[prev_wp][0];
    double m_y = xy_pt[1] -  xy_path[prev_wp][1];

    //n.m = dotproduct(n,m)
    //projection of line m onto n is given by p = (n.m/n.n)n
    //multiplication factor of dot product
    double proj_norm = (m_x*n_x+m_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = xy_path[prev_wp][0] + (proj_norm*n_x);
  	double proj_y = xy_path[prev_wp][1] + (proj_norm*n_y);

    tf::Point pt_on_line = tf::Point{proj_x,proj_y,0.0};
    double frenet_d = distance(pt_on_line,xy_pt);

    // Check point is on left or right side of the lane information //cross product
    // https://stackoverflow.com/questions/1560492/how-to-tell-whether-a-point-is-to-the-right-or-left-side-of-a-line
    double direction_val = n_x*m_y - n_y*m_x;
    //if direction_val = 0, on line, < 0 - right, > 0 for left
    // lets say <= 0 for right side of lane for making distinction of left and right lane
    // lets keep frenet_d to positive if point is on rght side and -ve if on left side for easy understanding.
    if(direction_val>0.0)
      frenet_d *= -1; //lets say frenet coordinates are -ve on left side of reference driving line.

    double frenet_s = frenet_path[prev_wp].s + distance(xy_path[prev_wp],pt_on_line);
    //Road to car angle difference
    double car_road_th = frenet_path[next_wp].th -theta;
    if (car_road_th>3.14)
      car_road_th=car_road_th-6.28;
    else if (car_road_th<-3.14)
      car_road_th=car_road_th+6.28;
    //TODO remove debug
    //ROS_INFO("Gt Frnt projxy %f, %f, prev_x,y %f, %f , next_xy %f %f", proj_x,proj_y,xy_path[prev_wp][0],xy_path[prev_wp][1],xy_path[next_wp][0],xy_path[next_wp][1]);
    //ROS_INFO("nwp %d pwp %d , pwp_s %f , ",next_wp,prev_wp,frenet_path[prev_wp].s,distance(xy_path[prev_wp],pt_on_line));
    return FrenetCoordinate(frenet_s,frenet_d,0, car_road_th);
  }


  /**Find the way point which is closest to a given point from list of waypoints on refernce path.
  **
  ** @param point
  */
  int VehiclePath::closestWayPoint(tf::Point pt){
    double closest_len= 100000; //some large number
    int closest_way_pt = 0;
    double dist = 0;
    for (size_t i = 0; i < xy_path.size(); i++) {
      dist =distance(pt, xy_path[i]);
      if(dist < closest_len){
        closest_len = dist;
        closest_way_pt = i;
      }
    }
    return closest_way_pt;
  }

  /**Find the next way point in driving direction
  **
  ** @param point
  */
  int VehiclePath::NextWayPoint(tf::Point pt, double theta){
    int closest_way_pt  = closestWayPoint(pt);
    //angle between the point and the closest_way_pt
    double heading = slope(pt,xy_path[closest_way_pt]);
    double angle = theta - heading;
    //Normalize angle to be in -pi to pi
    if (angle>M_PI)
      angle=angle-2*M_PI;
    else if (angle<-M_PI)
      angle=angle+2*M_PI;
    angle = abs(angle);
    //If the orientation of the point/car position and closest_way_pt is more than
    // 45 degrees, then the point is behind the car. so choose the next point
    if((angle>(M_PI/4)) && (closest_way_pt < (xy_path.size()-1))) //If the closest point is already the end point dont increment it
      closest_way_pt++;
    //TODO Remove debug
    //ROS_INFO("NW closest_wp %d , heading = %f , angle = %f ",closest_way_pt, heading, angle);
    return closest_way_pt;
  }

  /* Function to calculate curvature given 3 points
    curvature = 1/Radius_of_circumcircle_formemd_these points
    https://en.wikipedia.org/wiki/Menger_curvature,
    0 for a straight line and increases as radius decreases.
  */
  double VehiclePath::calc_curvature(tf::Point pts0,tf::Point pts1,tf::Point pts2){
    //for a triancle abc, area_twice = (a.x-c.x)*(b.y-a.y) - (a.x-b.x)*(c.y-a.y)
    double area = ((pts0[0]-pts2[0])*(pts1[1]-pts0[1]) - \
                    (pts0[0]-pts1[0])*(pts2[1]-pts0[1]))/2;

   //formula for curvature = 4*triangleArea/(sideLength1*sideLength2*sideLength3)
   double curvature = 0;
   if(area !=0){
     curvature = 4*area/(distance(pts0,pts1)*distance(pts1,pts2)*distance(pts2,pts0));
   }
   //ROS_INFO("curvature: %f area: %f, dist a: %f, b: %f , c: %f",curvature, area,a,b,c);
   return curvature;
  }

  /**Function to convert path to based frenet frame and base coordinates in form of tf::Points
  ** @param - None
  **/
  void VehiclePath::transformToXYandFrenet(){
    size_t number_of_pts = m_path.poses.size();
    //Clear the path and refill with new path - Important for accepting next segment
    xy_path.clear();
    frenet_path.clear();
    if(number_of_pts>0){
      //extract the points from the pose list
      size_t i;
      for ( i = 0; i < number_of_pts; i++) {
        tf::Point temp_pt;
        tf::pointMsgToTF(m_path.poses[i].pose.position, temp_pt);
        xy_path.push_back(temp_pt);
      }
      double length_ =0, curv=0, th =0;
      //creating the corresponding frenet information
      if(number_of_pts>2){
        double initial_curvature = calc_curvature(xy_path[0],xy_path[1],xy_path[2]);
        th = slope(xy_path[0],xy_path[1]);
        FrenetCoordinate start_(0,0,initial_curvature,th);
        frenet_path.push_back(start_);
        //As the points are closely placed this is causing the radius of curvature to be small. Thus to avoid it sample points alternatively
        for (i = 1; i < number_of_pts-4; i++) {
          length_ = distance(xy_path[i],xy_path[i-1]);
          // TODO - This may need to chnage - some other good method which is robust to errors is good
          // As the points in corners are closeby taking points at a farther distance bby skipping some to makes ure that curvature is not verz high.
          curv = calc_curvature(xy_path[i],xy_path[i+2],xy_path[i+4]);
          th = slope(xy_path[i-1],xy_path[i]);
          FrenetCoordinate fp(frenet_path[i-1].s + length_,0,curv,th);
          frenet_path.push_back(fp);
        }
        length_ = distance(xy_path[i],xy_path[i-1]);
        curv = calc_curvature(xy_path[i],xy_path[i+2],xy_path[i+4]);
        th = slope(xy_path[i-1],xy_path[i]);
        FrenetCoordinate fp = FrenetCoordinate(frenet_path[i-1].s + length_,0,curv,th);
        frenet_path.push_back(fp);
        i++;
        length_ = distance(xy_path[i],xy_path[i-1]);
        curv = calc_curvature(xy_path[i],xy_path[i+2],xy_path[i+4]);
        th = slope(xy_path[i-1],xy_path[i]);
        fp = FrenetCoordinate(frenet_path[i-1].s + length_,0,curv,th);
        frenet_path.push_back(fp);
        i++;

        //last but one point - i is already incremented before loop breaks & keep last calculated curvature
        length_ = distance(xy_path[i],xy_path[i-1]);
        th = slope(xy_path[i-1],xy_path[i]);
        fp = FrenetCoordinate(frenet_path[i-1].s + length_,0,curv,th);
        frenet_path.push_back(fp);
        i++;
        //last point
        length_ = distance(xy_path[i],xy_path[i-1]);
        th = slope(xy_path[i-1],xy_path[i]);
        fp =  FrenetCoordinate(frenet_path[i-1].s + length_,0,curv,th);
        frenet_path.push_back(fp);
      }
      else{ //If there are only two points in received path
        th = slope(xy_path[0],xy_path[1]);
        FrenetCoordinate start_(0,0,0,th);
        frenet_path.push_back(start_);
        length_ = distance(xy_path[1],xy_path[0]);
        FrenetCoordinate end_(length_,0,0,th);
        frenet_path.push_back(end_);
      }
    }
    else{
      ROS_ERROR("Path is empty");
      //TODO Initialize all the mpath, vectors to none
    }
  }//end of transformToXYandFrenet

  /* Function to calculate speed limit along the path
  Speed limit = min(legal speed, allowed speed due to curvature)
  speed_limit = sqrt(|a_lat/curv|)
  here choosing a_lat max = 3(constnat defined in vehicle path); and speed_limit is factored by 10 to align with interests of model car
  thus sl_curv = sqrt(0.3/curv)
  */
  void VehiclePath::calc_speed_limit(){
    speed_limit.clear();
    double s_val = frenet_path.front().s;
    if(frenet_path.size()>3){
      double avg_curv=0, speed_lmt=0;
      //lets say 0-5 (6)points, here 0-1 is limit @ 0, 1-2, limit@1, 2-3 limit @2 - last two columns are left to bring speed to zero
      size_t i=0;
      avg_curv = (frenet_path[0].k + frenet_path[1].k + frenet_path[2].k)/3;
      speed_lmt= (avg_curv>0)?(std::min(kLegalSpeedLimit,sqrt(kMaxLatAccLimit/avg_curv))):kLegalSpeedLimit;
      speed_lmt = (speed_lmt<kMinSpeedLimit)?kMinSpeedLimit:speed_lmt;
      //Assign same speed limit till next way point in increments of 0.1m
      while(s_val<frenet_path[1].s){
        speed_limit.push_back(speed_lmt);
        s_val += 0.1;
      }
      for (i = 1; i < frenet_path.size()-3; i++) {
        avg_curv = (frenet_path[i-1].k + frenet_path[i].k + frenet_path[i+1].k)/3;
        speed_lmt= (avg_curv>0)?(std::min(kLegalSpeedLimit,sqrt(kMaxLatAccLimit/avg_curv))):kLegalSpeedLimit;
        speed_lmt = (speed_lmt<kMinSpeedLimit)?kMinSpeedLimit:speed_lmt;
        //Assign same speed limit till next way point in increments of 0.1m
        while(s_val<frenet_path[i+1].s){
          speed_limit.push_back(speed_lmt);
          s_val += 0.1;
        }
      }//for loop
      //here 3-4(last but oneportion of path)set speed to half of previous
      while(s_val<frenet_path[i+1].s){
        speed_limit.push_back(speed_lmt/2);
        s_val += 0.1;
      }
      i++;
      //here 4-5(last portion of path) set limit to zero such that car stops
      while(s_val<=frenet_path[i+1].s){
        speed_limit.push_back(0);
        s_val += 0.1;
      }
    }
    else{//Small path keep velocity low
      while(s_val<=frenet_path.back().s){
        speed_limit.push_back(kMinSpeedLimit/3);
        s_val += 0.1;
      }
      size_t speed_vect_sz = speed_limit.size();
      speed_limit[speed_vect_sz-1] =0;
      speed_limit[speed_vect_sz-2] =0;
      speed_limit[speed_vect_sz-3] =0; //Make last points zero
    }

    nav_msgs::Path path_speed = m_path;
    for (size_t i = 0; i < path_speed.poses.size(); i++) {
      size_t idx = ((int)(frenet_path[i].s*10));
      idx = idx>speed_limit.size()-1?speed_limit.size()-1:idx;
      path_speed.poses[i].pose.position.z = speed_limit[idx];
    }
    path_with_speed_profiles.publish(path_speed);

  }//Calculate Speed limit

}//end of namespace
