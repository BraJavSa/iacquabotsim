import numpy as np
import time
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from math import sin, cos

class PositionController:
    def __init__(self, fr=50, tf=60):
        self.fr = fr
        self.ts = 1 / fr
        self.tf = tf
        self.t = np.arange(0, tf + self.ts, self.ts)
        
        self.Xd = 80 * np.sin(0.015 * self.t)
        self.Yd = 40 * np.sin(0.015 * self.t)
        
        self.Xdp = np.gradient(self.Xd, self.ts)
        self.Ydp = np.gradient(self.Yd, self.ts)
        
        rospy.init_node('position_controller', anonymous=True)
        self.odom_sub = rospy.Subscriber('/boat/odom', Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('/iacquabot/cmd_vel', Twist, queue_size=10)
        self.traj_pub = rospy.Publisher('/draw_trayectory', Pose, queue_size=10)  # <-- NUEVO
        
        self.Xr = 0
        self.Yr = 0
        self.psir = 0
    
    def odom_callback(self, msg):
        self.Xr = msg.pose.pose.position.x
        self.Yr = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        self.psir = self.quaternion_to_euler(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    
    def quaternion_to_euler(self, x, y, z, w):
        return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    
    def controller(self, Xd, Yd, Xdp, Ydp, Xr, Yr, psir):
        xre = Xd - Xr
        yre = Yd - Yr
        vx = Xdp + xre
        vy = Ydp + yre
        vpsi = -(vx / 0.2) * sin(psir) + (vy / 0.2) * cos(psir)
        Uref = vx * cos(psir) + vy * sin(psir)
        Wref = vpsi
        return Uref, Wref
    
    def send_velocity_commands(self, Uref, Wref):
        cmd_msg = Twist()
        cmd_msg.linear.x = Uref  
        cmd_msg.angular.z = Wref  
        self.cmd_pub.publish(cmd_msg)
    
    def send_desired_position(self, Xd, Yd):
        pose_msg = Pose()
        pose_msg.position.x = Xd
        pose_msg.position.y = Yd
        pose_msg.position.z = 0.0
        pose_msg.orientation.w = 1.0
        self.traj_pub.publish(pose_msg)
    
    def run(self):
        k = 1
        try:
            while not rospy.is_shutdown() and k < len(self.t):
                Xd, Yd = self.Xd[k], self.Yd[k]
                Uref, Wref = self.controller(Xd, Yd, self.Xdp[k], self.Ydp[k], self.Xr, self.Yr, self.psir)
                self.send_velocity_commands(Uref, Wref)
                self.send_desired_position(Xd, Yd)  # <-- NUEVO
                time.sleep(self.ts)
                k += 1
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    controller = PositionController()
    controller.run()
