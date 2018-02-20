#!/usr/bin/env python
'''
Import Python Packages, ROS messages used.
'''
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
import time
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import tf


#Different weights for straight edge and turning edge.
#This can be modified for path such that to optimize distance or time
straight_weight = 1
turn_weight = 1 # This makes the car to choose longer path over turns
left_turn_weight = 5
right_turn_weight = 2

#TODO
"""
Chnage the path to something where you can convey some information
header - time, frame_id
list of
point - (x,y)
significance - each point as stop, intersection(T,Y,X,+, any other), turn(l,r,s,round about) etc
road type -  one way, two way
road width -
road available to left
"""
##Publisher
path_pub = rospy.Publisher("/route_planner/sub_path",Path,queue_size=100)

##gloabl
seg_sub_paths = None
current_sub_path = 0
current_position_odom = None
current_goal_position  = None
graph = None
listener = tf.TransformListener()

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
    connections = []
    stops = [] # later remove from list and update here again
    connect_previous = False #make it true when a waypoint is encountered

    ### Choose the RNDF file for creating graph
    #pullData = open('rndf_1_way_loop.txt',"r").read()
    #pullData = open('rndf_2_way_road.txt',"r").read()
    #pullData = open('rndf_simulator_map_2way.txt',"r").read()
    #pullData = open('/home/korivi/Desktop/frei_traj/code/RNDF_Creation/rndf_files/rndf_1_way_simulator.txt',"r").read()
    #pullData = open('/home/korivi/model_car/catkin_ws_user/src/route_planner/src/rndf_1_way_simulator.txt',"r").read()

    #Used in the testing currently
    pullData = open('/home/korivi/model_car/catkin_ws_user/src/route_planner/src/rndf_map_1_wap_lab.txt',"r").read()
    #Map with 0,0 at left bottom - coodinates as per visual GPS
    #pullData = open('/home/korivi/model_car/catkin_ws_user/src/route_planner/src/rndf_map_1_wap_lab_vgps_map.txt',"r").read()

    #pullData = open('/home/korivi/model_car/catkin_ws_user/src/route_planner/src/sample_map_origin_map_1.txt',"r").read()
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
                    #G.add_edge(node_counter-1,node_counter) #connect previous and current way point
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
    return G, rndf



#### Find the nearest point in the Graph from any point
def closest_node(pt):
    closest_dist =100000.0
    closest_node = None
    for x in graph.nodes():
        dist = (x.coordi[0]-pt[0])**2 + (x.coordi[1]-pt[1])**2
        if(dist<closest_dist):
            closest_dist = dist
            closest_node = x
    return closest_node



#### Divide the Path into sub paths
def get_sub_paths(path,src_coordi,dst_coordi):
    subpaths = []
    sub_path =[]
    prev_parent = path[0].parent.parent.name
    #TODO removed src and destination from list, as this is corrupting if the src and first point are same etc. frenet conversion fails.
    #We need to only provide refernce path to follow, best would be to add a predecessor and success at start and end. Issues at corners there can be two predecessor and successors
    #sub_path.append(src_coordi)
    for node in path:
        #print node.name
        if(node.parent.parent.name == prev_parent):
            sub_path.append(node.coordi)
        else:
            subpaths.append(sub_path)
            #Use the previous sub paths last point as start for this path
            sub_path = [subpaths[-1][-1]]
            sub_path.append(node.coordi)
            prev_parent = node.parent.parent.name
    #sub_path.append(dst_coordi)
    subpaths.append(sub_path)
    #print "Segment wise Sub Paths",subpaths, len(subpaths)
    return subpaths

def publish_sub_paths(sub_path_id):
    rospy.loginfo("published new sub path: %d of %d", current_sub_path, len(seg_sub_paths)-1)
    if(sub_path_id < len(seg_sub_paths)):
        #posestamped list
        pose_list = list()
        #path to send the Rviz
        my_path = Path()
        my_path.header.stamp = rospy.Time.now()
        my_path.header.frame_id = '/map'

        for pt in seg_sub_paths[sub_path_id]:
            pose = PoseStamped()
            pose.pose.position.x =pt[0]
            pose.pose.position.y =pt[1]
            pose_list.append(pose)
            my_path.poses.append(pose)

        path_pub.publish(my_path)
    else:
        rospy.logerr("Requested subpath to publish is not in range")




def completed_sub_path(data):
    global current_sub_path
    current_sub_path += 1
    if(current_sub_path < len(seg_sub_paths)):
        publish_sub_paths(current_sub_path)
    else:
        rospy.loginfo("Destination Reached - provide new goal")

#save current car position
def odom_callback(data):
    global current_position_odom
    current_position_odom = data

#Callback when a new goal is received from the user
def goal_callback(data):
    global current_goal_position,seg_sub_paths,current_sub_path
    #Reset current sub path to zero
    current_sub_path=0
    current_goal_position = data
    #Initial values
    src_coordi = [0,0]
    dst_coordi = [current_goal_position.pose.position.x,current_goal_position.pose.position.y]

    #Convert odometry to map frame for source
    try:
        src_pose = PoseStamped()
        src_pose.header = current_position_odom.header
        src_pose.pose = current_position_odom.pose.pose
        pose_map = listener.transformPose("/map",src_pose)
        src_coordi[0] = pose_map.pose.position.x
        src_coordi[1] = pose_map.pose.position.y
    except Exception as e:
        print e

    #find the closest way points
    closest_way_pt_src  = closest_node(src_coordi)
    closest_way_pt_dst = closest_node(dst_coordi)
    path =  nx.shortest_path(graph,source=closest_way_pt_src,target=closest_way_pt_dst, weight= 'weight')
    if(len(path)<2):
        rospy.logerr("Choose a goal which is minimum of 0.5m ")
    #Update global sub paths for the robot to follow
    seg_sub_paths = get_sub_paths(path,src_coordi,dst_coordi)
    publish_sub_paths(current_sub_path)



def start():
    global seg_sub_paths, path_pub, graph
    #Initialize the ROS Node for this functionality
    rospy.init_node('RNDF_High_level_Route_Planner')
    rospy.Subscriber("/completed_sub_path", Int16, completed_sub_path)
    rospy.Subscriber("/odom",Odometry, odom_callback)
    rospy.Subscriber("/move_base_simple/goal",PoseStamped,goal_callback)
    #path_pub = rospy.Publisher("/route_planner_sub_path",Path,queue_size=100) #make gloabl
    #Creates and returns an RNDF graph
    #Save the Grah & rndf to a pickled file for easy retreive next time
    graph, rndf = create_graph()
    rospy.loginfo("Current odom position is considered as source")
    rospy.loginfo("Provide Goal using Rviz- 2D NavGoal")
    """ Debug purpose - printing path
    print "src ",src_coordi, " dst ", dst_coordi
    print closest_way_pt_src.name, closest_way_pt_dst.name
    prev = None
    for node in path:
        print node.name,node.coordi
        if prev == None:
            prev = node
        #print G.get_edge_data(prev,node,default=0)
        prev = node
    """
    #Sleep for a while to let all nodes Initialize
    time.sleep(.300)
    #This keeps the  active till it is killed
    rospy.spin()

if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass
