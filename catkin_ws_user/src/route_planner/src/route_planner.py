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


#Different weights for straight edge and turning edge.
#This can be modified for path such that to optimize distance or time
straight_weight = 1
turn_weight = 1 # This makes the car to choose longer path over turns
left_turn_weight = 5
right_turn_weight = 2


##Publisher
path_pub = rospy.Publisher("/route_planner_sub_path",Path,queue_size=100)

##gloabl
seg_sub_paths = None
sub_path_number = 0


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
    pullData = open('/home/korivi/Desktop/frei_traj/code/RNDF_Creation/rndf_files/rndf_1_way_simulator.txt',"r").read()
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
def closest_node(G, pt):
    closest_dist =100000.0
    closest_node = None
    for x in G.nodes():
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
    sub_path.append(src_coordi)
    for node in path:
        #print node.name
        if(node.parent.parent.name == prev_parent):
            sub_path.append(node.coordi)
        else:
            subpaths.append(sub_path)
            sub_path = []
            sub_path.append(node.coordi)
            prev_parent = node.parent.parent.name
    sub_path.append(dst_coordi)
    subpaths.append(sub_path)
    #print "Segment wise Sub Paths",subpaths, len(subpaths)
    return subpaths

def completed_sub_path(data):
    print data
    global sub_path_number
    sub_path_number += 1
    if(sub_path_number < len(seg_sub_paths)):
        #posestamped list
        pose_list = list()
        #path to send the Rviz
        my_path = Path()
        my_path.header.stamp = rospy.Time.now()
        my_path.header.frame_id = '/odom'

        for pt in seg_sub_paths[sub_path_number]:
            pose = PoseStamped()
            pose.pose.position.x =pt[0]
            pose.pose.position.y =pt[1]
            pose_list.append(pose)
            my_path.poses.append(pose)

        path_pub.publish(my_path)
    else:
        print"Destination Reached"

    print "published new sub path: ", sub_path_number, len(seg_sub_paths)

def start():
    global seg_sub_paths, path_pub
    #Initialize the ROS Node for this functionality
    rospy.init_node('RNDF_High_level_Route_Planner')
    rospy.Subscriber("/completed_sub_path", Int16, completed_sub_path)
    #path_pub = rospy.Publisher("/route_planner_sub_path",Path,queue_size=100) #make gloabl
    #Creates and returns an RNDF graph
    #Save the Grah & rndf to a pickled file for easy retreive next time
    graph, rndf = create_graph()
    ###Find the shortest path from source to Destination
    src_coordi = [1,0]
    dst_coordi = [-1,0]
    closest_way_pt_src  = closest_node(graph,src_coordi)
    closest_way_pt_dst = closest_node(graph,dst_coordi)
    # The indexes start from 0 and the names start from 1, check indexes and refer to map for points
    #s = rndf.segments[0].lanes[0].waypoints[1] #If to provide the src and destination from way points directly
    path =  nx.shortest_path(graph,source=closest_way_pt_src,target=closest_way_pt_dst, weight= 'weight')
    seg_sub_paths = get_sub_paths(path,src_coordi,dst_coordi)

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
    # TODO: from here - what to do once I have sub paths

    #Sleep for a while to let all nodes Initialize
    time.sleep(.300)
    #posestamped list
    pose_list = list()
    #path to send the Rviz
    my_path = Path()
    my_path.header.stamp = rospy.Time.now()
    my_path.header.frame_id = '/odom'
    for pt in seg_sub_paths[0]:
        pose = PoseStamped()
        pose.pose.position.x =pt[0]
        pose.pose.position.y =pt[1]
        pose_list.append(pose)
        my_path.poses.append(pose)

    path_pub.publish(my_path)
    #This keeps the  active till it is killed
    rospy.spin()

if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass
