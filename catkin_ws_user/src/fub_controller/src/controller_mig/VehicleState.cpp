#include "VehicleState.h"

#include <tf/tf.h>


namespace fub {
namespace controller {
namespace mig {

void VehicleState::setup(ros::NodeHandle & nh)
{
    // TODO: increase odom queue to at least 32
    mSubscriberOdom         = nh.subscribe("/odom", 1, &VehicleState::odometryCallback, this, ros::TransportHints().tcpNoDelay());
    mSubscriberPlannedPath  = nh.subscribe("planned_path", 1, &VehicleState::plannedPathCallback, this, ros::TransportHints().tcpNoDelay());
}

void VehicleState::plannedPathCallback(const fub_trajectory_msgs::TrajectoryConstPtr & msg)
{
    std::cout << "Received the Path "<<'\n';
    //mPath = *msg;
    /*To Run on Car*/
    mPath.trajectory.clear();
    mPath.header.seq = msg->header.seq;
    mPath.header.stamp =  msg->header.stamp;
    mPath.header.frame_id = msg->header.frame_id;
    mPath.child_frame_id = msg->child_frame_id;
    for (size_t i = 0; i < msg->trajectory.size(); i++) {
      mPath.trajectory.push_back(msg->trajectory[i]);
    }

    //TODO - Remove after debug
    ROS_INFO("When Path received: seq: %d x,y %.3f,%.3f   vel: %.3f  yaw:%.3f odom_time %f",msg->header.seq,mVehiclePosition[0],\
              mVehiclePosition[1], mCurrentSpeedFrontAxleCenter,getVehicleYaw(), mLastOdomTimeStampReceived.toSec());
}
void VehicleState::odometryCallback(const nav_msgs::OdometryConstPtr & msg)
{
    mEgoStatePose = msg->pose;
    mCurrentSpeedFrontAxleCenter =(double) msg->twist.twist.linear.x;
    tf::pointMsgToTF(mEgoStatePose.pose.position, mVehiclePosition);
    mLastOdomTimeStampReceived = msg->header.stamp;
}

double VehicleState::getVehicleYaw() const
{
    return tf::getYaw(mEgoStatePose.pose.orientation);// * radians;
}

}
}
}
