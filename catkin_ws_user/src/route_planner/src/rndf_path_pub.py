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
import networkx as nx
import matplotlib.pyplot as plt
from nav_msgs.msg import Path


# Array that keeps information aboutgoals and completed goals
goals = []
completed_goals = []
path_points = []
completed_path_points = []

#Different weights for different metrics.
#This can be modified for path such that to optimize distance or time
straight_weight = 1
turn_weight = 1 # This makes the car to choose longer path over turns
left_turn_weight = 5
right_turn_weight = 2

###
#Classes to support create RNDF Graph
###
class c_waypoint(object):
    def __init__(self,name,coordi,parent, idn):
        self.name = name
        self.coordi = coordi
        self.parent = parent
        self.idn = idn
        #If unique id for every waypoint is needed ?
        #self.uid = id(self)
    def __str__(self):
        return '{}'.format(self.name)

class c_exit(object):
    def __init__(self, entry, exit, parent):
        self.entry = entry
        self.exit = exit
        self.parent = parent


class c_lane(object):
    def __init__(self,name,parent,idn):
        self.name = name
        self.parent = parent
        self.idn = idn
        self.waypoints = []
        self.exits = []
    def add_waypoint(self,waypoint):
        self.waypoints.append(waypoint)
    def add_exit(self,exit):
        self.exits.append(exit)
    def __str__(self):
        return '{}'.format(self.name)

class c_segment(object):
    def __init__(self,name,parent,idn):
        self.name = name
        self.lanes = []
        self.parent = parent
        self.idn = idn
    def add_lane(self,lane):
        self.lanes.append(lane)
    def __str__(self):
        return '{}'.format(self.name)

class c_rndf(object):
    def __init__(self,name):
        self.name = name
        self.segments= []
    def add_segment(self,segment):
        self.segments.append(segment)
    def __str__(self):
        return '{}'.format(self.name)

#########
# Creating Graph
########
def create_graph():
    G = nx.DiGraph()
    connections = [] # Connections between segments
    stops = [] # later remove from list and update here again
    connect_previous = False #make it true when a waypoint is encountered
    #pullData = open('rndf_1_way_loop.txt',"r").read()
    #pullData = open('/home/korivi/seat/model_car_3/catkin_ws/src/path_pt_publisher/src/rndf_2_way_road.txt',"r").read()
    pullData = open('/home/korivi/seat/model_car_3/catkin_ws/src/path_pt_publisher/src/rndf_simulator_map_2way.txt',"r").read()
    dataArray = pullData.split('\n')
    node_counter = 0
    index = 0

    rndf = c_rndf("RNDF_object")
    for eachLine in dataArray:
        index += 1
        if len(eachLine)>1:
            it = eachLine.split(' ')
            #if any of these key words are encountered make connect previous false # it[0] == 'segment' or  #  it[0] == 'lane' or
            if  (it[0] == '/*' or it[0] == 'RNDF_name' or it[0] == 'num_segments' or it[0] == 'num_zones' \
                or it[0] == 'format_version' or it[0] == 'creation_date' or  it[0] == 'num_lanes' \
                or it[0] == 'segment_name' or it[0] == 'num_waypoints' or it[0] == 'lane_width' \
                or it[0] == 'left_boundary' or it[0] == 'right_boundary'  or it[0] == 'stop' or it[0] == 'end_lane' or it[0] == 'end_segment'\
                or it[0] == 'end_file' or it[0] == '</html>' ) :
                #Donot connect to previous way point
                connect_previous = False

            elif it[0] == 'segment':
                #create a segment and set parent as the rndf object
                s = c_segment(it[1],rndf, idn = int(it[1]))
                rndf.add_segment(s)
                connect_previous = False

            elif it[0] == 'lane':
                #New lane's parent is the latest of the segment in process
                l = c_lane(it[1], rndf.segments[-1],idn = int(it[1].split('.')[-1])) #
                rndf.segments[-1].add_lane(l)
                connect_previous = False

            elif it[0] == 'exit':
                connect_previous = False
                connections.append(eachLine)

            #else consider it as waypoint
            else :
                try:
                    #New waypoint's parent is the latest line of the latest segment in consideration
                    #rndf.segments[-1].lanes[-1].add_waypoint(name=it[0],coordi=[float(it[1]),float(it[2])],parent = rndf.segments[-1].lanes[-1])
                    p = c_waypoint(name = it[0],coordi = [float(it[1]),float(it[2])],parent = rndf.segments[-1].lanes[-1], idn = int(it[0].split('.')[-1]))
                    #add waypoint to the lane
                    rndf.segments[-1].lanes[-1].add_waypoint(p)
                    #add waypoint to the graph
                    G.add_node(p, name=it[0])
                except Exception as ex:
                    #print 'exception'
                    print index
                    print it[0]
                    print ex

                if(connect_previous == True):
                    wt = np.linalg.norm(np.array(p_old.coordi)-np.array(p.coordi), 2, 0)
                    G.add_edge(p_old,p, weight = wt)
                connect_previous = True # Enable such that the next way point will be connected to previous
                p_old = p
                node_counter += 1 #increment node counter


    #Improve this code to add exits easily without so many iterations
    #Add exits to the graph
    #"""
    if len(connections) >0:
        for txt in connections:
            if len(txt)>1:
                it = txt.split(' ')
                if it[0] == 'exit':
                    for x in G.nodes():
                        if it[1] == x.name:
                            initial = x #entry node
                            break
                    for x in G.nodes():
                        if it[2] == x.name:
                            final = x #exit node
                            break
                    #add edge in graph
                    G.add_edge(initial,final, weight = turn_weight)
                    #weight = left_turn_weight if it[3]=='l' else right_turn_weight (#TODO use this weight to differentiate between left and right turns)
                    # add the exit to the lane of the entry point with the lane as parent to exit
                    initial.parent.add_exit(c_exit(entry = initial,exit = final, parent = initial.parent))
    #Return the Graph
    return G, rndf



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
    i = 0 #index for the marker array
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



#This function will update the list of points that should be published to the control
def update_pose_list(goals):
    #Create an object to new pose array
    newPoseArray = PoseArray()
    #assign frame of these pose objects as map
    newPoseArray.header.frame_id = "map"
    #Boundary Checks
    if(len(goals)!=0):
        #Update all the goals to the Pose array by appending new element to the
        i = 0 #index for the pose array
        #pose array
        for goal in goals:
            newPoseArray.poses.append(Pose())
            newPoseArray.poses[i].position.x = goal[0]
            newPoseArray.poses[i].position.y = goal[1]
            i = i+1
        #Publish the new list to the listener to move the robot.
        poseArray_pub.publish(newPoseArray)


# This Callback function will be called everytime a goal is reached.
# control unit or path planner will publish the index of point in goal list
def completed_cb(data):
    element_num = data.data
    #check for bounds
    if(element_num < len(goals)):
        #debug::#TODO remove below line
        print('removed::',goals[element_num],'  ', element_num)
        #update the array by adding the new completed goal.
        completed_goals.append(goals[element_num])
        #Remove the point from goals list.
        goals.pop(element_num)
        #Clear the map
        clear_map()
        #Update Markers on RViz
        update_markers(goals, completed_goals)
        #Update the list of goals to be published to control unit
        update_pose_list(goals)
    elif(len(goals)==0):
        print("Mission accomplished")
    else:
        # If this is printed than there is a problem in control/planner;
        # it is publishing element index out of range..
        print("OUT OF RANGE", element_num)


# This Callback function will called everytime a point is placed on RViz
def click_cb(data):
    pass

def update_path_points(path):
    newPoseArray = PoseArray()
    i = 0 #index for the pose array
    #assign frame of these pose objects as map
    newPoseArray.header.frame_id = "map"
    #Boundary Checks
    if(len(path)!=0):
        for pt in path:
            #print pt.name,pt.coordi
            #Update all the goals to the Pose array by appending new element to the
            newPoseArray.poses.append(Pose())
            newPoseArray.poses[i].position.x = pt.coordi[0]
            newPoseArray.poses[i].position.y = pt.coordi[1]
            #Full Path always available
            path_points.append(pt.coordi)
            #Goals yet to be covered
            goals.append(pt.coordi)
            #Publish the new list to the tb_path_publisher to move the robot.
            i = i+1
        #Publish the list to the driving node
        poseArray_pub.publish(newPoseArray)

def start():
    global goal_pub, poseArray_pub
    #Initialize the ROS Node for this functionality
    rospy.init_node('RNDF_Goal_publisher')
    #Creates and returns an RNDF graph
    #Save the Grah & rndf to a pickled file for easy reteive next time
    graph, rndf = create_graph()
    s = rndf.segments[0].lanes[0].waypoints[3]
    d = rndf.segments[0].lanes[0].waypoints[4]
    path =  nx.shortest_path(graph,source=s,target=d, weight= 'weight')
    print s.name, d.name
    #for ob in path:
    #    print ob.name,ob.coordi

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
    #add the points to the list to publish to other nodes
    update_path_points(path)
    #This keeps the  active till it is killed
    rospy.spin()

if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass
