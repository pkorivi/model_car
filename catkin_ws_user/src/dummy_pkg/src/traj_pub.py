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
import math
from tf.transformations import quaternion_from_euler


def main(args):
    rospy.init_node('dummy_trajectory_publisher', anonymous = False)
    traj_publisher = rospy.Publisher("/model_car/trajectory", Trajectory, queue_size =10)
    rate = rospy.Rate(1)
    seq = 1
    j = 1
    while j<10:
        j += 1
        seq += 1
        hello_str = "time %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        new_traj = Trajectory()
        new_traj.header.seq = seq
        new_traj.header.stamp = rospy.Time.now()
        new_traj.header.frame_id = "/odom" #TODO change to /map frame
        new_traj.child_frame_id = "/odom"
        #some points for road to follow
        points = [(-0.20, 0.00),
        (1.32, 0.01),
        (2.82, 0.00),
        (3.82, -0.01),
        (4.53, -0.26),
        (4.87, -0.63),
        (5.07, -1.09),
        (5.08, -1.59),
        (4.92, -2.06),
        (4.60, -2.42),
        (4.19, -2.68),
        (3.11, -3.33),
        (2.24, -3.84),
        (1.58, -4.21)]
        #trajectory with points 1m distance and 1mps speed
        for i in range(0,len(points)):
            tp = TrajectoryPoint()
            #pose-position
            tp.pose.position.x = points[i][0]
            tp.pose.position.y = points[i][1]
            tp.pose.position.z = -1
            #pose - orientation
            #make orientation towards next point at every point
            #yaw - gives orientation of car, which is slope of the line or tangent of spline
            #atan result is in between -90 to 90 removing directionality, thus using atan2 will be able to find which quadrant the line is in
            if(i < (len(points)-1)):
                #slope = ((points[i+1][1]-points[i][1])/(points[i+1][0] - points[i][0]))
                #print "il::",i,"slope::",slope
                yaw = math.atan2((points[i+1][1]-points[i][1]),(points[i+1][0] - points[i][0]))
            else:
                #slope = ((points[i][1]-points[i-1][1])/(points[i][0] - points[i-1][0]))
                #print "idk::",i,"slope::",slope
                yaw = math.atan2((points[i][1]-points[i-1][1]),(points[i][0] - points[i-1][0]))

            # RPY to convert: 0deg, 0, slope of line as rotation- yaw
            #sxyz - roll, pitch, yaw
            yaw = yaw*180.0/math.pi
            print "yaw ::",yaw
            qat = quaternion_from_euler(0, 0, yaw,axes='sxyz')
            #print "qat ",qat[0]," ",qat[1]," ",qat[2]," ",qat[3]
            tp.pose.orientation.x = qat[0]
            tp.pose.orientation.y = qat[1]
            tp.pose.orientation.z = qat[2]
            tp.pose.orientation.w = qat[3]
            #tp.pose.orientation.w = 1
            #TODO check if orientation of pose is needed
            #TODO check what can be done to the velocity, is it needed
            #tp.velocity.linear.x = 1
            #TODO - velocity is of type twist, check if other linear and angular velocities are needed
            tp.acceleration.linear.x = 0
            #TODO - check if complete accleration is needed
            tp.time_from_start =rospy.Duration(i*2)

            #Append this point to the array of traj points
            new_traj.trajectory.append(tp)
            #print "tp: ",tp

        #print "yaw ::","0"
        #qat = quaternion_from_euler(0, 0, 0,axes='sxyz')
        #print "qat ",qat[0]," ",qat[1]," ",qat[2]," ",qat[3]

        traj_publisher.publish(new_traj)
        rate.sleep()
    #rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
