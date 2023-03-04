#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32

driver = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
path_points = PoseArray()
index = 1

def callback(path):
    global path_points, index, driver
    path_points = path
    goto = PoseStamped()
    goto.pose = path_points.poses[index]
    print(goto)
    driver.publish(goto)
    rospy.loginfo("Going to point: " + str(index))
    index += 1
    path_sub.unregister() 

def distance_callback(dist):
    global index
    goto = PoseStamped()
    goto.pose = path_points.poses[index]

    if dist.data < Float32(0.2).data:
        rospy.loginfo("Going to point: " + str(index - 1))
        driver.publish(goto)
        index += 1

if __name__ == '__main__':
    rospy.init_node('driver')
    path_sub = rospy.Subscriber("/prm_path", PoseArray, callback)
    distance_sub = rospy.Subscriber("/diff_drive_go_to_goal/distance_to_goal", Float32, distance_callback)
    rospy.spin()