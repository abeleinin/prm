#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32

class PRM_Drive:
    def __init__(self):
        rospy.init_node('PRM_drive')
        self.driver = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
        self.path_sub = rospy.Subscriber("/prm_path", PoseArray, self.path_callback)
        self.distance_sub = rospy.Subscriber("/diff_drive_go_to_goal/distance_to_goal", Float32, self.distance_callback)
        self.path = PoseArray()
        self.pathLength = 0
        self.dist_to_goal = Float32()
        self.current_goal = PoseStamped()
        self.index = 1

    def path_callback(self, path):
        self.path = path
        self.pathLength = len(self.path.poses)
        self.current_goal.pose = self.path.poses[self.index]
        self.driver.publish(self.current_goal)
        rospy.loginfo("Going to point: " + str(self.index))
        self.index += 1
        self.path_sub.unregister()
    
    def distance_callback(self, dist):
        if self.index < self.pathLength and self.index > 0:
            if dist.data < Float32(0.2).data:
                self.current_goal.pose = self.path.poses[self.index]
                self.driver.publish(self.current_goal)
                rospy.loginfo("Going to point: " + str(self.index))
                self.index += 1

    def get_path(self):
        return self.path

if __name__ == '__main__':
    PRM_Drive()
    rospy.spin()