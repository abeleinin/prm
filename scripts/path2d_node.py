#!/usr/bin/env python3
from distutils.command import clean
import random
from tkinter import W

import rospy
import math
import numpy as np

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import matplotlib.pyplot as plt

import shapely.geometry as shp_geo
from shapely.ops import nearest_points

# from graph import Graph
from dijkstar import Graph, find_path

np.set_printoptions(precision=1)


obs_stack = []
def env_callback(mrkr_arr):
    # consider a rectangular robot with dimension w=0.2 and l=0.4
    rbt_w = 0.2
    rbt_l = 0.4
    smpls_num = 1000


    ### inflation to obstacles 
    infl = math.sqrt( (rbt_w/2)*(rbt_w/2) + (rbt_l/2)*(rbt_l/2)) 
    rospy.loginfo(" Rviz Environment callback function: extracting obstacles ...  ")
    
    
    # define vector of obstacles, each obstacle is a polygon
    print("Extracted Obstacles are: ")
    for marker in mrkr_arr.markers:

        if marker.type == 1:
            print("CUBE")
            rect_cntr = [marker.pose.position.x, marker.pose.position.y] 
            rect_orn = marker.pose.orientation
            rect_wl = [marker.scale.x, marker.scale.y]
            rect_points = []
            if rect_orn.w > 0.8:
                # print("norm: ", rect_wl)
                rect_points.append( shp_geo.Point(rect_cntr[0] - rect_wl[0]/2-infl, rect_cntr[1] - rect_wl[1]/2-infl) ) 
                rect_points.append( shp_geo.Point(rect_cntr[0] + rect_wl[0]/2+infl, rect_cntr[1] - rect_wl[1]/2-infl) ) 
                rect_points.append( shp_geo.Point(rect_cntr[0] + rect_wl[0]/2+infl, rect_cntr[1] + rect_wl[1]/2+infl) ) 
                rect_points.append( shp_geo.Point(rect_cntr[0] - rect_wl[0]/2-infl, rect_cntr[1] + rect_wl[1]/2+infl) ) 
            elif rect_orn.w < 1:
                # print("90deg: ", rect_wl)
                rect_points.append( shp_geo.Point(rect_cntr[0] - rect_wl[1]/2-infl, rect_cntr[1] - rect_wl[0]/2-infl) ) 
                rect_points.append( shp_geo.Point(rect_cntr[0] + rect_wl[1]/2+infl, rect_cntr[1] - rect_wl[0]/2-infl) ) 
                rect_points.append( shp_geo.Point(rect_cntr[0] + rect_wl[1]/2+infl, rect_cntr[1] + rect_wl[0]/2+infl) ) 
                rect_points.append( shp_geo.Point(rect_cntr[0] - rect_wl[1]/2-infl, rect_cntr[1] + rect_wl[0]/2+infl) ) 

            obs_stack.append( shp_geo.Polygon([[pt.x, pt.y] for pt in rect_points]) )

        elif marker.type == 3:
            print("CYLINDER")
            sph_cntr = [marker.pose.position.x, marker.pose.position.y] 
            sph_orn = marker.pose.orientation
            sph_dmtr = marker.scale.x
            obs_stack.append( shp_geo.Point(sph_cntr[0], sph_cntr[1]).buffer(sph_dmtr/2 + infl) ) 
        
        elif marker.type == 4:
            print("LINE_STRIP: boundary")
            brdr_pts = marker.points   
            brdr_scale = marker.scale         
            exter = []
            inter = []
            for brdr_pt in brdr_pts:     
                exter.append( ( brdr_pt.x, brdr_pt.y) ) 

            inter.append( ( brdr_pts[0].x + brdr_scale.x + infl, brdr_pts[0].y + brdr_scale.x + infl) ) 
            inter.append( ( brdr_pts[1].x - brdr_scale.x - infl, brdr_pts[1].y + brdr_scale.x + infl) ) 
            inter.append( ( brdr_pts[2].x - brdr_scale.x - infl, brdr_pts[2].y - brdr_scale.x - infl) ) 
            inter.append( ( brdr_pts[3].x + brdr_scale.x + infl, brdr_pts[3].y - brdr_scale.x - infl) ) 
            brdr_plyg = shp_geo.Polygon(exter, [inter])
            obs_stack.append(brdr_plyg)


        elif marker.type == 8:
            start = [marker.points[0].x, marker.points[0].y, 0]# marker.points[0].z]
            goal = [marker.points[1].x, marker.points[1].y, 0]#marker.points[1].z] 
    sub.unregister()

    # call PRMpathPlanner algorithm which takes as argument:
    # - obstacles 
    # - number of sampled points 
    # - start and goal points 
    # and it return a 2D path 
    # path_2d = PRMPathPlanner(smpls_num, obs_stack, start, goal)
g = Graph()
def PRMpathPlanner(smpls_num, k, start, goal):
    random_points = getRandomPoints(smpls_num)
    clean_points = checkCollision(random_points)
    clean_points.insert(0, start)
    clean_points.insert(1, goal)
    g._undirected = True
    lines = findNeighbor(clean_points, k)
    path = find_path(g, 0, 1).nodes
    points = [clean_points[p] for p in path]
    for i in range(len(path)-1):
        line = shp_geo.LineString([clean_points[path[i]], clean_points[path[i + 1]]])
        x = [line.coords[0][0], line.coords[1][0]]
        y = [line.coords[0][1], line.coords[1][1]]
        plt.plot(x, y, color='r', linewidth=4)
    return points

def getRandomPoints(smpls_num):
    coordList = []
    for num in range(smpls_num):
        rand_x = random.uniform(-5, 5)
        rand_y = random.uniform(-5, 5)
        coordList.append(shp_geo.Point(rand_x, rand_y))
    return coordList

def checkCollision(lst_points):
    global obs_stack
    collisionFree = []
    collision = False
    fig, axs = plt.subplots()
    for obs in obs_stack:
        x,y = obs.exterior.xy
        axs.fill(x, y, alpha=0.5, fc='y', ec='none')
        if obs.length < 20:
            axs.fill(x, y, alpha=1, fc='y', ec='none')
    for point in lst_points:
        for obs in obs_stack:
            if collision:
                break
            bound = obs.bounds
            min_x = bound[0]
            min_y = bound[1]
            max_x = bound[2]
            max_y = bound[3]
            if obs.length < 10:
                center_x = (min_x + max_x) / 2
                center_y = (min_y + max_y) / 2
                radius = ((-1 * min_x) + max_x) / 2
                point_location = ((point.x - center_x) ** 2) + ((point.y - center_y) ** 2)
                if point_location < (radius ** 2):
                    collision = True
            if obs.length > 10 and obs.length < 20:
                if point.x > min_x and point.x < max_x and point.y > min_y and point.y < max_y:
                    collision = True
        if not collision:
            collisionFree.append(point)
            plt.scatter(point.x, point.y)
        collision = False
    return collisionFree

def findNeighbor(lst_points, k):
    i = 0
    collisionFreeLines = []
    for i in range(len(lst_points)):
        if(i % 100 == 0):
            rospy.loginfo((i / len(lst_points)) * 100)
        closest = []
        removed = lst_points.copy()
        temp = removed[i]
        removed.remove(temp)
        for x in range(k):
            near = nearest_points(temp, shp_geo.MultiPoint(removed))
            closest.append(near[1])
            removed.remove(near[1])
        for point in closest:
            line = shp_geo.LineString([temp, point])
            if(collissionFreeLine(temp, point)):
                inverseLine = shp_geo.LineString([point, temp])
                if inverseLine in collisionFreeLines:
                    continue
                else:
                    collisionFreeLines.append(line)
                    g.add_edge(lst_points.index(temp), lst_points.index(point), line.length)
    return collisionFreeLines

def collissionFreeLine(start, end):
    collision = False
    line = shp_geo.LineString([start, end])
    for obs in obs_stack:
        if collision:
            break
        bound = obs.bounds
        min_x = bound[0]
        min_y = bound[1]
        max_x = bound[2]
        max_y = bound[3]
        if obs.length > 10 and obs.length < 20:
            min_min = shp_geo.Point(min_x, min_y)
            max_min = shp_geo.Point(max_x, min_y)
            min_max = shp_geo.Point(min_x, max_y)
            max_max = shp_geo.Point(max_x, max_y)
            # if(doIntersect(start, end, min_min, max_min) or
               # doIntersect(start, end, min_min, min_max) or
               # doIntersect(start, end, max_min, max_max) or
               # doIntersect(start, end, min_max, max_max)):
            if line.intersects(obs):
                collision = True
    if not collision:
        x = [line.coords[0][0], line.coords[1][0]]
        y = [line.coords[0][1], line.coords[1][1]]
        plt.plot(x, y)
        return True
    collision = False
    return False 

# Code from Geeks for Geeks 
def orientation(p, q, r):
    val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y))
    if(val > 0):
        return 1
    elif(val < 0):
        return 2
    else:
        return 0
    
def onSegment(p, q, r):
    if ( (q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and
           (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))):
        return True
    return False

def doIntersect(p1, q1, p2, q2):
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)
    
    if ((o1 != o2) and (o3 != o4)):
        return True
    if ((o1 == 0) and onSegment(p1, p2, q1)):
        return True
    if ((o2 == 0) and onSegment(p1, q2, q1)):
        return True
    if ((o3 == 0) and onSegment(p2, p1, q2)):
        return True
    if ((o4 == 0) and onSegment(p2, q1, q2)):
        return True
    return False
    
def find_prm_path():
    rospy.init_node('path2d_node')
    rbt_w = rospy.get_param('~rbt_w', 0.2)
    rbt_l = rospy.get_param('~rbt_l', 0.4)    
    # consider a rectangular robot with dimension: width=0.2m, and length=0.4m
    # rospy.loginfo("robot dimension: width, length=  %d, %d", rbt_w , rbt_l)
    # rospy.loginfo("#samples, #neighbours= %d, %d",   smpls_num , nybrs_num)
    rospy.Subscriber("visualization_marker_array", MarkerArray, env_callback)
    rospy.spin()

def pathPublisher(path):
    pub = rospy.Publisher('/prm_path', PoseArray, queue_size=10)
    posePath = PoseArray()
    poseArray = []
    while not rospy.is_shutdown():
        for point in path:
            pose = Pose()
            pose.position.x = point.x
            pose.position.y = point.y
            pose.position.z = 0
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 0
            poseArray.append(pose)
        # posePath.header = Header()
        posePath.poses = poseArray
        pub.publish(posePath) 
        poseArray.clear()

if __name__ == '__main__':
    rospy.init_node('path2d_node')
    sub = rospy.Subscriber("visualization_marker_array", MarkerArray, env_callback)
    start = shp_geo.Point(-4, -4)
    goal = shp_geo.Point(-4, -3.5)
    path = PRMpathPlanner(100, 5, start, goal)
    for p in path:
        print(p)
    pathPublisher(path)
    for i in obs_stack:
        if i.type == 1:
            print("1")
        else:
            print(i.length)
    plt.show()
    # find_prm_path()
