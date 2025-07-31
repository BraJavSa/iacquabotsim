#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class UpdateNode:
    def __init__(self):
        rospy.init_node("update_twist_node")
        self.vel_pub = rospy.Publisher("boat/cmd_vel", Twist, queue_size=10)
        self.vel_sub = rospy.Subscriber("/boat/cmd", Twist, self.update_velocity)
        self.vel = Twist()

    def update_velocity(self, msg):
        self.vel = msg

    def process(self):
        self.vel_pub.publish(self.vel)

def main():
    manual_mode = UpdateNode()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        manual_mode.process()
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
