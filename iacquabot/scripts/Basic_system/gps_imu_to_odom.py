#!/usr/bin/env python3
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class OdomToTF:
    def __init__(self):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber("/iacquabot/sensors/position/p3d_wamv", Odometry, self.cb_odom)

    def cb_odom(self, msg):
        tfm = TransformStamped()
        tfm.header.stamp = rospy.Time.now()
        tfm.header.frame_id = msg.header.frame_id       
        tfm.child_frame_id = msg.child_frame_id         
        tfm.transform.translation.x = msg.pose.pose.position.x
        tfm.transform.translation.y = msg.pose.pose.position.y
        tfm.transform.translation.z = msg.pose.pose.position.z
        tfm.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tfm)

if __name__ == "__main__":
    rospy.init_node("odom_to_tf")
    OdomToTF()
    rospy.spin()
