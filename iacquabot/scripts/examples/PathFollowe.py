import numpy as np
import time
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from math import sin, cos

class PathFollower:
    def __init__(self, fr=50, tf=60):
        self.fr = fr
        self.ts = 1 / fr
        self.tf = tf
        self.t = np.arange(0, tf + self.ts, self.ts)
        
        self.Xd = 80 * np.sin(0.04 * self.t)  # Trajectory path (X)
        self.Yd = 40 * np.sin(0.02 * self.t)  # Trajectory path (Y)
        
        self.Xdp = 1.5 * np.ones(len(self.t))  
        self.Ydp = 0.5 * np.ones(len(self.t))  
        
        rospy.init_node('path_follower', anonymous=True)
        self.odom_sub = rospy.Subscriber('/boat/odom', Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('/iacquabot/cmd_vel', Twist, queue_size=10)
        self.desired_pos_pub = rospy.Publisher('/desired_position', Point, queue_size=10)
        
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
        pos_msg = Point()
        pos_msg.x = Xd
        pos_msg.y = Yd
        pos_msg.z = 0  
        self.desired_pos_pub.publish(pos_msg)  
    
    def run(self):
        k = 0
        try:
            while not rospy.is_shutdown() and k < len(self.t):
                distancias = np.sqrt((self.Xd - self.Xr)**2 + (self.Yd - self.Yr)**2)
                indice_min = np.argmin(distancias)
                
                Xd_closest = self.Xd[indice_min]
                Yd_closest = self.Yd[indice_min]
                
                Xdp = self.Xdp[k]
                Ydp = self.Ydp[k]
                
                Uref, Wref = self.controller(Xd_closest, Yd_closest, Xdp, Ydp, self.Xr, self.Yr, self.psir)                  
                self.send_velocity_commands(Uref, Wref)
                self.send_desired_position(Xd_closest, Yd_closest)  
                
                time.sleep(self.ts)
                k += 1
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    path_follower = PathFollower()
    path_follower.run()
