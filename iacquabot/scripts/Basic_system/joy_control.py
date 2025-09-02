#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3

pub = None

def joy_cb(msg):
    throttle = msg.axes[1]   # adelante/atrás
    steer    = -msg.axes[2]   # izquierda/derecha
    left  = throttle + steer
    right = throttle - steer
    cmd = Vector3(x=left, y=right, z=0.0)
    pub.publish(cmd)

if __name__ == "__main__":
    rospy.init_node("joy_to_thruster_simple")
    pub = rospy.Publisher("/boat/cmd_thruster", Vector3, queue_size=10)
    rospy.Subscriber("/joy", Joy, joy_cb)
    rospy.spin()
