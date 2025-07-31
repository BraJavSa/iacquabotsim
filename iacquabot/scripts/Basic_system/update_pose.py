#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

class UpdateNode:
    def __init__(self):
        rospy.init_node("update_pose_node")
        self.pose_pub = rospy.Publisher("/boat/pose_d", PoseStamped, queue_size=10)
        self.pose_sub = rospy.Subscriber("/boat/posed", PoseStamped, self.update_desired_pose)
        self.desired_pose = PoseStamped()
        self.desired_pose.pose.position.x = 0
        self.desired_pose.pose.position.y = 0
        self.desired_pose.pose.position.z = 0
        self.rate = rospy.Rate(10)  # Frecuencia de publicación de 100 Hz
    
    def update_desired_pose(self, msg):
        # Actualizar la pose deseada si se recibe un nuevo mensaje
        self.desired_pose = msg

    def publish_initial_pose(self):
        # Publicar la pose inicial cada 100 Hz
        while not rospy.is_shutdown():
            self.pose_pub.publish(self.desired_pose)
            self.rate.sleep()

def main():
    manual_mode = UpdateNode()
    manual_mode.publish_initial_pose()

if __name__ == "__main__":
    main()
