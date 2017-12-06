/*
 * sample_nodelet_class2.h
 *
 *  Created on: 2016/09/18
 *      Author: cryborg21
 */
#ifndef VEHICLE_STATE_NODELET_CLASS2_H_
#define VEHICLE_STATE_NODELET_CLASS2_H_
#include <nodelet/nodelet.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/tf.h>

namespace fub_motion_planner{
  class VehicleState {    //: public nodelet::Nodelet
    public:
        VehicleState();
        ~VehicleState();
        //virtual void onInit();

        void setup(ros::NodeHandle & nh);
        /** Retrieve the vehicle's yaw orientation in the world
         **
         ** @return the vehicle's yaw in the world
         */
        double getVehicleYaw() const;
    protected:
        void odometryCallback(const nav_msgs::OdometryConstPtr & msg);
        void RoutePlannerCallback(const nav_msgs::Path & msg);

    public:
        /// last received pose (from odometry message)
        geometry_msgs::PoseWithCovariance m_ego_state_pose;

        /// last received position (from odometry message)
        tf::Point m_vehicle_position;

        /// timestamp of last received odometry message
        ros::Time m_last_odom_time_stamp_received;

        /// the currently planned path
        nav_msgs::Path m_path;
        double m_current_speed_front_axle_center;


    private:
        ros::Subscriber m_subscribe_odom;
        ros::Subscriber m_subscribe_route_planner;

  };
} // namespace sample_nodelet_ns

#endif /* SAMPLE_NODELET_CLASS_SRC_SAMPLE_NODELET_CLASS2_H_ */
