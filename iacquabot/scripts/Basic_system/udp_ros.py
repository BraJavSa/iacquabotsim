#!/usr/bin/env python3
import rospy, socket
from geometry_msgs.msg import Vector3   # más simple que Pose

UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", UDP_PORT))

rospy.init_node("apriltag_cmd_listener")
pub = rospy.Publisher("/apriltag_cmd", Vector3, queue_size=10)

while not rospy.is_shutdown():
    data, _ = sock.recvfrom(1024)
    parts = data.decode().split(",")
    if len(parts) != 2:
        continue

    dx, dy = parts
    dx, dy = float(dx), float(dy)

    msg = Vector3()
    msg.x = dx/100
    msg.y = dy/100    
    msg.z = 0.0   # no usado

    pub.publish(msg)
