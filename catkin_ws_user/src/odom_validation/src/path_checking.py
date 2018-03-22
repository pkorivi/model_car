#!/usr/bin/env python
import random
import time
import roslib
import rospy
import sys
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from std_msgs.msg import Float32
import tf

#file2write=open("odom_car_2.txt",'w')

speed = 0
steer = 0

def speed_callback(data):
    global speed
    speed = data.data
    print "speed ",speed

def steer_callback(data):
    global steer
    steer = data.data
    print "steer ",steer

"""
def odom_callback(data):
    global x,y,z
    file2write.write(str(data.header.stamp.secs))
    file2write.write(",")
    file2write.write(str(data.pose.pose.position.x))
    file2write.write(",")
    file2write.write(str(data.pose.pose.position.y))
    file2write.write(",")
    file2write.write(str(data.twist.twist.linear.x))
    file2write.write(",")
    file2write.write(str(speed))
    file2write.write(",")
    file2write.write(str(steer))
    file2write.write("\n")
"""

def odom_callback(data):
    print rospy.Time.now().secs,".",rospy.Time.now().nsecs
    orien = [data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
    _,_,yaw = tf.transformations.euler_from_quaternion(orien)
    print "x,y : ",data.pose.pose.position.x," , ",data.pose.pose.position.y, " yaw ",yaw

def main(args):
    rospy.init_node('odom_plotter', anonymous = False)
    print('Publish odometry from car/rosbag to record to odom_car.txt ')
    image_sub = rospy.Subscriber("/odom",Odometry, odom_callback)
    image_sub = rospy.Subscriber("/manual_control/speed",Int16, speed_callback)
    image_sub = rospy.Subscriber("/manual_control/steering",Int16, steer_callback)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
