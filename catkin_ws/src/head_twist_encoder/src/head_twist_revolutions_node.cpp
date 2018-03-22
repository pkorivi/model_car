#include <head_twist_revolutions/head_twist_revolutions.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "head_twist_revolutions_node");
  ros::NodeHandle nh;
  head_twist_revolutions ReadHeading(nh);
  //TODO Increased the value from 100 to 1000, this improved the performance of odometry reading and reduced the latency to within 40ms
  ros::Rate rate(1000);
   while(ros::ok())
  {
    ros::spinOnce();
    ReadHeading.get();
    rate.sleep();
  }
  return 0;
}
