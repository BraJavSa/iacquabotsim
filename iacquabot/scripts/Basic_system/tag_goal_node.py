#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
import math
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Pose2D

class TagGoalNode:
    def __init__(self):
        rospy.init_node("tag_goal_node")
        self.pub_goal = rospy.Publisher("/docking_goal", Pose2D, queue_size=10)

        # Suscribirse a detecciones de AprilTag
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.cb)

        # Buffer y listener TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ID del AprilTag de docking
        self.tag_id = rospy.get_param("~dock_tag_id", 1)

        rospy.loginfo("Nodo tag_goal_node listo, esperando detecciones...")

    def cb(self, msg):
        if not msg.detections:
            return

        for det in msg.detections:
            if det.id[0] == self.tag_id:
                # Pose del tag en frame de la cámara
                tag_pose = PoseStamped()
                tag_pose.header = det.pose.header      # ✅ header correcto
                tag_pose.pose = det.pose.pose.pose     # PoseWithCovariance → Pose

                try:
                    # Transformar al marco "map"
                    transform = self.tf_buffer.lookup_transform(
                        "map",                           # frame destino
                        tag_pose.header.frame_id,        # frame origen (cámara)
                        rospy.Time(0),
                        rospy.Duration(0.1))
                    tag_map = tf2_geometry_msgs.do_transform_pose(tag_pose, transform)

                    x = tag_map.pose.position.x
                    y = tag_map.pose.position.y

                    # Orientación del tag → yaw
                    q = tag_map.pose.orientation
                    siny = 2*(q.w*q.z + q.x*q.y)
                    cosy = 1-2*(q.y*q.y + q.z*q.z)
                    yaw = math.atan2(siny, cosy)

                    # Punto deseado: 1 m delante del tag
                    x_goal = x - 1.0*math.cos(yaw)
                    y_goal = y - 1.0*math.sin(yaw)
                    yaw_goal = yaw  # de frente al muelle

                    goal = Pose2D()
                    goal.x = x_goal
                    goal.y = y_goal
                    goal.theta = yaw_goal
                    self.pub_goal.publish(goal)

                    rospy.loginfo_throttle(2.0,
                        f"Docking goal publicado: ({goal.x:.2f}, {goal.y:.2f}, {goal.theta:.2f})")

                except Exception as e:
                    rospy.logwarn("No se pudo transformar el tag: %s" % str(e))

if __name__ == "__main__":
    TagGoalNode()
    rospy.spin()
