#!/usr/bin/env python
import time
import roslib
import rospy
import sys
import numpy as np
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped, Pose, Point, Twist, Quaternion,Vector3
import time
from fub_trajectory_msgs.msg import Trajectory
from fub_trajectory_msgs.msg import TrajectoryPoint
import math
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
import tf
from nav_msgs.msg import Odometry

odom_pub = rospy.Publisher("/odom", Odometry, queue_size =10)

def pub_odom():
    vel =0.2
    x = 0.3#4.19#4.5
    y = 0.0#-2.48#0.0
    current_time = rospy.Time.now()
    odom_broadcaster = tf.TransformBroadcaster()
    #since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0,0,0.0)
    #first, we'll publish the transform over tf
    odom_broadcaster.sendTransform((0, 0, 0),odom_quat,current_time,"base_link","odom")
    #next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vel, 0, 0), Vector3(0, 0, 0))
    #publish the message
    odom_pub.publish(odom)


def main(args):
    rospy.init_node('odom_pub', anonymous = False)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub_odom()
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
