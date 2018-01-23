#!/usr/bin/env python
import random
import time
import roslib
import rospy
import sys
import numpy as np
from matplotlib import animation
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped

file2write=open("seg_9.txt",'w')
seq_no =1

def point_callback(data):
    global seq_no
    file2write.write(str.format("1.1.{}",seq_no))
    file2write.write(" ")
    file2write.write(str.format("{0:.2f}",data.point.x))
    file2write.write(" ")
    file2write.write(str.format("{0:.2f}",data.point.y))
    file2write.write("\n")
    seq_no +=1


def main(args):
    rospy.init_node('rndf_subs', anonymous = False)
    image_sub = rospy.Subscriber("/clicked_point",PointStamped, point_callback)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
