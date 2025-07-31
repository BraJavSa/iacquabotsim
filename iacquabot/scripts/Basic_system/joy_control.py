#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyListener:
    def __init__(self):
        rospy.init_node('joy_listener', anonymous=True)
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.pub = rospy.Publisher('command_boat/cmd', Twist, queue_size=10)
        self.twist_msg = Twist()
        self.linear_vel_x = 0.0
        self.linear_vel_y = 0.0
        self.angular_vel = 0.0
    
    def joy_callback(self, msg):
        linear_vel_x = msg.axes[1]*2
        self.angular_vel= msg.axes[3]*0.7
        self.twist_msg.linear.x = linear_vel_x
        self.twist_msg.angular.z = self.angular_vel
        self.pub.publish(self.twist_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    joy_listener = JoyListener()
    joy_listener.run()
