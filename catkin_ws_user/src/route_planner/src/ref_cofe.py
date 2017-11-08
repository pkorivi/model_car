#!/usr/bin/env python
'''
Import Python Packages, ROS messages used.
'''
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
import time
import numpy as np

# Array that keeps information aboutgoals and completed goals
goals = []
completed_goals = []

# Clear markers from the map
# More info about Markers and MarkerArrays here: http://docs.ros.org/kinetic/api/visualization_msgs/html/msg/MarkerArray.html
def clear_map():
    #create a new Marker Array
    newMarkerArray = MarkerArray()
    #Append it with a marker
    newMarkerArray.markers.append(Marker())
    #Assign Frame ID as map
    newMarkerArray.markers[0].header.frame_id = "map"
    #Assign action as 3 indicating to delete the array
    newMarkerArray.markers[0].action = 3
    #Publish the Marker Array
    goal_pub.publish(newMarkerArray)

# Update the marker list with goals completed and finished and publish to RViz
# for vizulaization.
def update_markers(goals, complete_goals):
    newMarkerArray = MarkerArray()
    i = 0
    #Loop twice, for goals and completed goals
    for j in [1,2]:
        if j == 1:
            #Assign goals. marker array will be updated with goals
            tmp_arr = goals
            color = [0.0, 1.0 ,0.0]
        else:
            #Assign competed goals
            tmp_arr = complete_goals
            color = [0.0, 0.0 ,1.0]
        #Update the marker array with further information required to publish to
        #RViz along with x,y assigned in above lines.
        for goal in tmp_arr:
            newMarkerArray.markers.append(Marker())
            newMarkerArray.markers[i].header.frame_id = "map"
            newMarkerArray.markers[i].id = i;
            newMarkerArray.markers[i].type = 2
            newMarkerArray.markers[i].scale.x = 0.3
            newMarkerArray.markers[i].scale.z = 0.1
            newMarkerArray.markers[i].scale.y = 0.3
            newMarkerArray.markers[i].color.a = 1.0
            newMarkerArray.markers[i].color.r = color[0]
            newMarkerArray.markers[i].color.g = color[1]
            newMarkerArray.markers[i].color.b = color[2]
            newMarkerArray.markers[i].pose.position.x = goal[0]
            newMarkerArray.markers[i].pose.position.y = goal[1]
            i = i+1
    #Publish the Message to RViz
    goal_pub.publish(newMarkerArray)

#This function will update the list of goals that should be published to tb_path_publisher
def update_pose_list(goals):
    i = 0
    #Create an object to new pose array
    newPoseArray = PoseArray()
    #assign frame of these pose objects as map
    newPoseArray.header.frame_id = "map"
    #Boundary Checks
    if(len(goals)!=0):
        #Update all the goals to the Pose array by appending new element to the
        #pose array
        for goal in goals:
            newPoseArray.poses.append(Pose())
            newPoseArray.poses[i].position.x = goal[0]
            newPoseArray.poses[i].position.y = goal[1]
            i = i+1
        #Publish the new list to the tb_path_publisher to move the robot.
        poseArray_pub.publish(newPoseArray)

# This Callback function will be called everytime a goal is reached.
# tb_path_publisher will publish the index of point in goal list
def completed_cb(data):
    global goals, completed_goals
    #Index of goal acheived
    element_num = data.data
    #check for bounds
    if(element_num < len(goals)):
        #update the array by adding the new completed goal.
        completed_goals.append(goals[element_num])
        #Remove the point from goals list.
        goals.pop(element_num)
        #Clear the map
        clear_map()
        #Update Markers on RViz
        update_markers(goals, completed_goals)
        #Update the list of goals to be published to tb_path_publisher
        update_pose_list(goals)
    else:
        # If this is printed than there is a problem in tb_path_publisher;
        # it is publishing element index out of range..
        print("OUT OF RANGE")

# This Callback function will called everytime a point is placed on RViz
def click_cb(data):
    global goals
    #Gather the x,y from the point published from RViz
    x = data.point.x
    y = data.point.y
    z = 0
    #Update the list of goals by appending the Currently received point.
    goals.append([x,y])
    #Clear the map
    clear_map()
    #Update the markes to display on RViz
    update_markers(goals, completed_goals)
    #Update the list of goals that should be published to tb_path_publisher
    update_pose_list(goals)

def start():
    global goal_pub, poseArray_pub
    #Initialize the ROS Node for this functionality
    rospy.init_node('tb_rviz_interaction')
    #Initialize publisher to publish Marker array to display on RViz
    goal_pub = rospy.Publisher("/goal_markers", MarkerArray, queue_size = 1)
    #Initialize publisher to publish PoseArray with list of goals to tb_path_publisher
    poseArray_pub = rospy.Publisher("/list_of_goals", PoseArray, queue_size = 1)
    #subscribe to receive the points clicked in RViz
    rospy.Subscriber("/clicked_point", PointStamped, click_cb)
    #Subscribe to message published from tb_path_publisher about the goal accomplished
    rospy.Subscriber("/goal_completed", Int16, completed_cb)
    #Sleep for a while to let all nodes Initialize
    time.sleep(.300)
    #Clear the map
    clear_map()
    #This keeps the  active till it is killed
    rospy.spin()

if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass
