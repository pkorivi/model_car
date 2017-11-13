#!/usr/bin/env python
import random
import time
import roslib
import rospy
import sys
import numpy as np
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
import time
from fub_trajectory_msgs.msg import Trajectory
from fub_trajectory_msgs.msg import TrajectoryPoint


def main(args):
    rospy.init_node('dummy_trajectory_publisher', anonymous = False)
    traj_publisher = rospy.Publisher("/model_car/trajectory", Trajectory, queue_size =1)
    rate = rospy.Rate(1)
    seq = 1
    while not rospy.is_shutdown():
        seq += 1
        hello_str = "time %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        new_traj = Trajectory()
        new_traj.header.seq = seq
        new_traj.header.stamp = rospy.Time.now()
        new_traj.header.frame_id = "/map"
        new_traj.child_frame_id = "/odom"

        #trajectory with points 1m distance and 1mps speed
        for i in range(0,8):
            tp = TrajectoryPoint()
            tp.pose.position.x = -4+i
            tp.pose.position.y = 0
            tp.pose.orientation.w = 1
            #TODO check if orientation of pose is needed

            tp.velocity.linear.x = 1
            #TODO - velocity is of type twist, check if other linear and angular velocities are needed
            tp.acceleration.linear.x = 0
            #TODO - check if complete accleration is needed
            tp.time_from_start =rospy.Duration(i)

            #Append this point to the array of traj points
            new_traj.trajectory.append(tp)
            #print "tp: ",tp

        traj_publisher.publish(new_traj)
        rate.sleep()
    #rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
