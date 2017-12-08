//TODO Create a calss to receive the route planner data and do all conversions for frenet and stuff.
// All fucntions to x,y. path, whats the width so much will be done here.

#include "VehiclePath.h"
namespace fub_motion_planner{
  VehiclePath::VehiclePath(){}
  VehiclePath::~VehiclePath(){}
  void VehiclePath::setup(ros::NodeHandle & nh){
      m_subscribe_route_planner  = nh.subscribe("/route_planner/sub_path", 1, &VehicleState::RoutePlannerCallback, this, ros::TransportHints().tcpNoDelay());
  }
  //route planner callback
  void VehicleState::RoutePlannerCallback(const nav_msgs::Path & msg){
    ROS_INFO("path_received");
    m_path = msg;
    //Trigger transormation to frenet function
    transformToFrenet();
  }

  /*
  Path is represented by number of line segments,  find the line segment to which
  this s belongs to then find the point on a line perpendicular to that line segment
  at lateral distnace d
  */
  tf::Point VehicleState::getXY(FrenetCoordinate frenet_pt){
    int prev_wp = -1;
    //find the closest waypoint index behind the given frenet point
    while((frenet_pt.s > frenet_path[prev_wp+1].s) && \
            (prev_wp < frenet_path.size()-1)){
      prev_wp++;
    }
    //next waypoint index
    int next_wp = (prev_wp+1)%frenet_path.size();
    //slop of the line segment
    double heading = slope(next_wp,prev_wp);
    //distance along this segment
    double seg_s = (s- frenet[prev_wp].s);
    //x,y on the path
    double seg_x = xy_path[prev_wp][0] + seg_s*cos(heading);
    double seg_y = xy_path[prev_wp][1] + seg_s*sin(heading);
    //angle of line perpendicular to the current segment
    double perp_heading = heading - M_PI/2;
    //Point at a distance perpendicular to a line segment
    return tf::Point{seg_x + d*cos(perp_heading),seg_y+d*sin(perp_heading),0.0};
  }

  FrenetCoordinate VehicleState::getFenet(tf::Point xy_pt, double theta){
    //the line segment closest to the current point
    int next_wp = NextWayPoint(xy_pt, theta);
    int prev_wp = next_wp -1;
    if(next_wp==0){
      ROS_ERROR("Oops something is wrong, the vehicle is away from the map");
    }
    //Then find the projection of point on the above found line segment.
    // Distance till projected point provides s and distance between projected
    // and actual point is d.
    //http://www.sunshine2k.de/coding/java/PointOnLine/PointOnLine.html#step5
    //https://math.la.asu.edu/~surgent/mat272/dotcross.pdf (refer to dot product and orthogonal projection sections)
    //vector formed by line segment on map lets say n
    double n_y = xy_path[next_wp][1] - xy_path[prev_wp][1];
    double n_x = xy_path[next_wp][0] - xy_path[prev_wp][0];
    //vector formed by the line joining prev_wp and the given point lets say m
    double m_x = xy_pt[0] -  xy_path[prev_wp][0];
    double m_y = xy_pt[1] -  xy_path[prev_wp][1];
    //n.m = dotproduct(n,m)
    //projection of line m onto n is given by p = (n.m/n.n)n
    //multiplication facto of dot product
    double proj_norm = (m_x*n_x+m_y*n_y)/(n_x*n_x+n_y*n_y);
    //projected points onto the line segment
    double proj_x = proj_norm*n_x;
  	double proj_y = proj_norm*n_y;
    tf::Point pt_on_line = tf::Point{proj_x,proj_y}
    //double frenet_d = sqrt((proj_x-xy_pt[0])*(proj_x-xy_pt[0]) +(proj_y-xy_pt[1])*(proj_y-xy_pt[1]));

    double frenet_d = distance(pt_on_line,xy_pt)
    //TODO from here
    // Check point is on left or right side of the lane information //cross product
    // https://stackoverflow.com/questions/1560492/how-to-tell-whether-a-point-is-to-the-right-or-left-side-of-a-line
    double direction_val = n_x*m_y - n_y*m_x;
    //if direction_val = 0, on line, < 0 - right, > 0 for left
    // lets say <= 0 for right side of lane for making distinction of left and right lane
    // lets keep frenet_d to positive if point is on rght side and -ve if on left side for easy understanding.
    if(direction_val>0)
      frenet_d *= -1; //lets say frenet coordinates are -ve on left side of reference driving line.

    double frenet_s = frenet_path[prev_wp].s + distance(xy_path[prev_wp],pt_on_line);
    return FrenetCoordinate(frenet_s,frenet_d,0);
  }



  //Find the way point which is closest to a given point
  int VehicleState::closestWayPoint(tf::Point pt){
    double closest_len= 100000; //some large number
    int closest_way_pt = 0;
    double dist = 0;
    for (size_t i = 0; i < xy_path.size(); i++) {
      dist =distnace(pt, xy_path[i]);
      if(dist < closest_len){
        closest_len = dist;
        closest_way_pt = i;
      }
    }
    return closest_way_pt;
  }

  //Find the next way point in driving direction
  int VehicleState::NextWayPoint(tf::Point pt, double theta){
    int closest_way_pt  = closestWayPoint(pt);
    //angle between the point and the closest_way_pt
    double heading = slope(pt,xy_path[i]);
    //If the orientation of the point/car position and closest_way_pt is more than
    // 45 degrees, then the point is behind the car. so choose the next point
    double angle = abs(theta - heading);
    if(angle>(M_PI/4))
      closest_way_pt++;

    return closest_way_pt;
  }

  /*
    curvature = 1/Radius_of_circumcircle_formemd_these points
    https://en.wikipedia.org/wiki/Menger_curvature,
    0 for a line and increases as radius decreases.
  */
  double calc_curvature(std::vector<tf::Point> pts){
    //for a triancle abc, area_twice = (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x)
    double area = ((pts[1][0]-pts[0][0])*(pts[2][1]-pts[0][1]) - \
                    (pts[1][0]-pts[0][0])*(pts[2][1]-pts[0][1]))/2;
   //curvature = 4*triangleArea/(sideLength1*sideLength2*sideLength3)
   double curvature = 0;
   if(area !=0){
     curvature = 4*area/(distance(pts[0],pts[1])*distance(pts[1],pts[2])*distance(pts[2],pts[0]));
   }
   return curvature;
  }

  void transformToXYandFrenet(){
    size_t number_of_pts = m_path.poses.size();
    if(number_of_pts>0){
      //extract the points from the pose list
      size_t i;
      for ( i = 0; i < number_of_pts; i++) {
        tf::Point temp_pt;
        tf::pointMsgToTF(m_path.poses[i].pose.position, temp_pt);
        xy_path.push_back(temp_pt);
      }
      //creating the cooresponding frenet information
      if(number_of_pts>2){
        double initial_curvature = calc_curvature(xy_path[0],xy_path[1],xy_path[2])
        FrenetCoordinate start_(0,0,initial_curvature);
        frenet_path.push_back(start_);
        double length, curv;
        //leave the last two points as curvature cannot be calculated, Initialize it with last known curvature or straight line
        for (i = 1; i < number_of_pts-2; i++) {
          lenth = distance(xy_path[i],xy_path[i-1]);
          curv = calc_curvature(xy_path[i],xy_path[i+1],xy_path[i+2]);
          FrenetCoordinate fp(frenet_path[i-1].s + lenth,0,curv);
          frenet_path.push_back(fp);
        }
        //last but one point - i is already incremented before loop breaks & keep last calculated curvature
        lenth = distance(xy_path[i],xy_path[i-1]);
        FrenetCoordinate fp(frenet_path[i-1].s + lenth,0,curv);
        frenet_path.push_back(fp);
        i++;
        //last point
        lenth = distance(xy_path[i],xy_path[i-1]);
        FrenetCoordinate fp1(frenet_path[i-1].s + lenth,0,curv);
        frenet_path.push_back(fp1);
      }
      else{ //If there are only two points in received path
        FrenetCoordinate start_(0,0,0);
        frenet_path.push_back(start_);
        lenth = distance(xy_path[1],xy_path[0]);
        FrenetCoordinate end_(lenth,0,0);
        frenet_path.push_back(end_);
      }
    }
    else{
      ROS_INFO("Path is empty");
      //TODO Initialize all the mpath, vectors to none
    }
  }//end of transformToXYandFrenet

}//end of namespace
