#!/usr/bin/env python3

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import rospy
import math

width = 10

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray,  queue_size=10)

rospy.init_node('register')

markerArray = MarkerArray()

## Creating Arena with obstacles
boundary = Marker()
boundary.header.frame_id = "map" # This is to pair the frames with Rviz
boundary.type = Marker.LINE_STRIP
boundary.action = Marker.ADD

boundary.scale.x = 0.2

boundary.color.a = 1.0
boundary.color.r = 1.0
boundary.color.g = 1.0
boundary.color.b = 0.0

boundary.ns = "Obstacles"
boundary.id = 0
#boundary.points = []

boundary.points.append(Point(-width/2,-width/2,0))
boundary.points.append(Point(width/2,-width/2,0))
boundary.points.append(Point(width/2,width/2,0))
boundary.points.append(Point(-width/2,width/2,0))
boundary.points.append(Point(-width/2,-width/2,0))

markerArray.markers.append(boundary)
###################################################
## Creating Obstacles

for i in range(4):
    obst = Marker()
    obst.type = Marker.CUBE
    obst.header.frame_id = "map"
    obst.action = Marker.ADD
    obst.ns = "Obstacles"
    obst.color = boundary.color
    obst.id = i+1

    obst.scale.x = obst.scale.z = 0.5
    obst.scale.y = 8
    obst.pose.position.z = 0.25
    obst.pose.orientation.x = obst.pose.orientation.y = obst.pose.orientation.z = 0
    obst.pose.orientation.w = 1

    if i == 0:
        obst.pose.position.x = -3
        obst.pose.position.y = -1
    if i == 1:
        obst.pose.position.x = -1.5
        obst.pose.position.y = 2
        obst.scale.y = 5
        obst.pose.orientation.w = obst.pose.orientation.z = 0.7071
    if i == 2:
        obst.pose.position.x = 3
        obst.pose.position.y = 1
    if i == 3:
        obst.pose.position.x = 1.5
        obst.pose.position.y = -2
        obst.scale.y = 5
        obst.pose.orientation.w = obst.pose.orientation.z = 0.7071

    markerArray.markers.append(obst)

for i in range(4,7):
    obst = Marker()
    obst.type = Marker.CYLINDER
    obst.header.frame_id = "map"
    obst.action = Marker.ADD
    obst.ns = "Obstacles"
    obst.color = boundary.color
    obst.id = i+1

    obst.scale.x = obst.scale.y = obst.scale.z = 2
    obst.pose.position.x = 0
    obst.pose.position.z = 1

    if i == 4:
        obst.pose.position.y = 4
    if i == 5:
        obst.pose.position.y = 0
    if i == 6:
        obst.pose.position.y = -4

    markerArray.markers.append(obst)

##################################################
## Start point and end points
    goalPoint = Marker()
    goalPoint.type = Marker.POINTS
    goalPoint.header.frame_id = "map"
    goalPoint.action = Marker.ADD
    goalPoint.ns = "Obstacles"
    goalPoint.pose.orientation.w = 1.0
    goalPoint.scale.x = 0.25
    goalPoint.scale.y = 0.25
    goalPoint.color.g = 1.0
    goalPoint.color.a = 1.0
    goalPoint.id = 8

    goalPoint.points.append(Point(-4,-4,0))
    goalPoint.points.append(Point(4,4,0))
    markerArray.markers.append(goalPoint)

#################################################

while not rospy.is_shutdown():
    publisher.publish(markerArray)



    rospy.sleep(0.01)
