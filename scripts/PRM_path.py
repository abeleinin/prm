#!/usr/bin/env python3
import rospy
import math
import random

from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import MarkerArray
import matplotlib.pyplot as plt

import shapely.geometry as shp_geo
from shapely.ops import nearest_points

from dijkstar import Graph, find_path

class PRM_path:
    def __init__(self, sample_num: int, k: int, start: shp_geo.Point, goal: shp_geo.Point):
        self.obs_stack = []
        self.objects_sub = rospy.Subscriber("visualization_marker_array", MarkerArray, self.envCallback)
        self.graph = Graph()
        self.graph._undirected = True
        self.pathToGoal = self.shortestPath(sample_num, k, start, goal)
        self.pathPublisher(self.pathToGoal)
        
    def envCallback(self, mrkr_arr: MarkerArray) -> None:
        """
        Generate the obstical stack from the environment.
        """
        # consider a rectangular robot with dimension w=0.2 and l=0.4
        rbt_w = 0.2
        rbt_l = 0.4

        ### inflation to obstacles 
        infl = math.sqrt( (rbt_w/2)*(rbt_w/2) + (rbt_l/2)*(rbt_l/2)) 
        rospy.loginfo("Rviz Environment callback function: extracting obstacles ...  ")
    
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

                self.obs_stack.append( shp_geo.Polygon([[pt.x, pt.y] for pt in rect_points]) )

            elif marker.type == 3:
                print("CYLINDER")
                sph_cntr = [marker.pose.position.x, marker.pose.position.y] 
                sph_orn = marker.pose.orientation
                sph_dmtr = marker.scale.x
                self.obs_stack.append( shp_geo.Point(sph_cntr[0], sph_cntr[1]).buffer(sph_dmtr/2 + infl) ) 
        
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
                # self.obs_stack.append(brdr_plyg)

            elif marker.type == 8:
                start = [marker.points[0].x, marker.points[0].y, ]
                goal = [marker.points[1].x, marker.points[1].y, 0]
        self.objects_sub.unregister()

    def shortestPath(self, sample_num: int, k: int, start: shp_geo.Point, goal: shp_geo.Point) -> list:
        """
        Generate the shortest path from the `start` point to the `goal`
        using `sample_num` sampled random points in the environment with
        `k` connections to the closest neighbors.

        Returns
            Shortest Path in the form of a list of `shapely.geometry.Points`
        """
        randomPoints = self.generatePoints(sample_num)
        collisionFreeCoords = self.checkCollision(randomPoints)
        collisionFreeCoords.insert(0, start)
        collisionFreeCoords.insert(1, goal)
        self.findNearestNeighbor(collisionFreeCoords, k)
        shortestPath = find_path(self.graph, 0, 1).nodes
        shortestPathPoints = [collisionFreeCoords[p] for p in shortestPath]
        for i in range(len(shortestPath)-1):
            line = shp_geo.LineString([collisionFreeCoords[shortestPath[i]], collisionFreeCoords[shortestPath[i + 1]]])
            x = [line.coords[0][0], line.coords[1][0]]
            y = [line.coords[0][1], line.coords[1][1]]
            plt.plot(x, y, color='r', linewidth=4)
        return shortestPathPoints

    def generatePoints(self, sample_num: int) -> list:
        """
        Generates a sample_nums amout of random `shapely.geometry.Point(x, y)` 

        Returns
            List of random `shapely.geometry.Points`
        """
        randomPoints = []
        i = 0
        while(i < sample_num):
            randX = random.uniform(-5, 5)
            randY = random.uniform(-5, 5)
            randomPoints.append(shp_geo.Point(randX, randY))
            i += 1
        return randomPoints 

    def checkCollision(self, points: list) -> list:
        """
        Removes points that collide with obsticles in the environment.

        Returns
            List of `shapely.geometry.Points`
        """
        collisionFreePoints = []
        collision = False
        fig, axs = plt.subplots()
        for point in points:
            for obs in self.obs_stack:
                x,y = obs.exterior.xy
                axs.fill(x, y, alpha=1, fc='y', ec='none')
                if obs.contains(point):
                    collision = True
                    break
            if not collision:
                collisionFreePoints.append(point)
                plt.scatter(point.x, point.y)
            collision = False
        return collisionFreePoints

    def collissionFreeLine(self, start: shp_geo.Point, end: shp_geo.Point) -> bool:
        """
        Identifies lines that collide with objects in the environment.
        
        Returns
            Boolean values that represent if the line intersects with
            and object in the environment.
        """
        collision = False
        line = shp_geo.LineString([start, end])
        for obs in self.obs_stack:
            if collision:
                break
            if line.intersects(obs):
                collision = True
                break
        if not collision:
            x = [line.coords[0][0], line.coords[1][0]]
            y = [line.coords[0][1], line.coords[1][1]]
            plt.plot(x, y)
        return not collision 

    def findNearestNeighbor(self, points: list, k: int) -> list:
        """
        Finds the nearest neighbor for every points and add that connect
        to `self.graph` to create a graph of all the sampled points in the 
        environment.
        
        Returns
            List of `shapely.geometry.LineString`
        """
        i = 0
        collisionFreeLines = []
        for i in range(len(points)):
            if(i % 100 == 0):
                rospy.loginfo((i / len(points)) * 100)
            closest = []
            removed = points.copy()
            temp = removed[i]
            removed.remove(temp)
            for x in range(k):
                near = nearest_points(temp, shp_geo.MultiPoint(removed))
                closest.append(near[1])
                removed.remove(near[1])
            for point in closest:
                line = shp_geo.LineString([temp, point])
                if(self.collissionFreeLine(temp, point)):
                    inverseLine = shp_geo.LineString([point, temp])
                    if inverseLine in collisionFreeLines:
                        continue
                    else:
                        collisionFreeLines.append(line)
                        self.graph.add_edge(points.index(temp), points.index(point), line.length)
        return collisionFreeLines

    def pathPublisher(self, shortestPath: list) -> None:
        """
        Publishes the shortest path from the sampled points from the starting
        position to the goal position.
        """
        rospy.loginfo("Path published")
        pub = rospy.Publisher('/prm_path', PoseArray, queue_size=10)
        posePath = PoseArray()
        poseArray = []
        while not rospy.is_shutdown():
            for point in shortestPath:
                pose = Pose()
                pose.position.x = point.x
                pose.position.y = point.y
                poseArray.append(pose)
            posePath.poses = poseArray
            pub.publish(posePath) 
            poseArray.clear()
    
    def printShortestPath(self):
        """
        Prints the points that represent the shortest path from start to goal.
        """
        for point in self.path:
            print(point)

    def showGraph():
        """
        Shows a matplotlib graph of the sampled points, obsticles, connections
        to other points, and the shortest path from start to goal.
        """
        plt.show()

if __name__ == '__main__':
    rospy.init_node('path2d_node')
    start = shp_geo.Point(-4, -4)
    goal = shp_geo.Point(-4, 4)
    sample_num = 500
    k = 10
    PRM_path(sample_num, k, start, goal)
