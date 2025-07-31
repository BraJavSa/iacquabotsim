#!/usr/bin/env python3
import rospy
import time 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class UpdateNode:
    def __init__(self):
        rospy.init_node("update_twist_node")
        self.vel_pub= rospy.Publisher("boat/cmd_vel", Twist, queue_size=10)     
        self.vel_sub = rospy.Subscriber("command_boat/cmd", Twist, self.update_velocity)
        self.vel = Twist()
 
 
    def update_velocity(self, vel):
    	self.vel = vel

    def send_initial_velocity(self):
        
        self.vel.linear.x=0
        self.vel.linear.y=0
        self.vel.linear.z=0
        self.vel.angular.x=0
        self.vel.angular.y=0
        self.vel.angular.z=0      
        self.vel_pub.publish(self.vel)

    def process(self):   
        self.vel_pub.publish(self.vel)

def main():
    manual_mode = UpdateNode()
    manual_mode.send_initial_velocity()
    rate = rospy.Rate(200)

    while not rospy.is_shutdown():
        manual_mode.process()
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except IOError as e:
        print(e)