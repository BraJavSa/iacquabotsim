#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point  # Cambio de PoseStamped a Point

class TrajectoryAnimation:
    def __init__(self, fr=50, tf=360):
        self.fr = fr
        self.ts = 1 / fr
        self.tf = tf
        self.t = np.arange(0, tf + self.ts, self.ts)
        
        self.Xd = 80 * np.sin(0.04 * self.t)
        self.Yd = 40 * np.sin(0.02 * self.t)

        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-90, 90)
        self.ax.set_ylim(-50, 50)
        self.ax.plot(self.Xd, self.Yd, 'b-', lw=2)

        self.boat_point, = self.ax.plot([], [], 'ro', markersize=10)
        self.desired_point, = self.ax.plot([], [], 'go', markersize=10)

        rospy.init_node('trajectory_animation')
        rospy.Subscriber('/boat/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/desired_position', Point, self.desired_position_callback)  # Cambio de PoseStamped a Point

        self.rate = rospy.Rate(self.fr)

        self.boat_x = 0.0
        self.boat_y = 0.0
        self.desired_x = 0.0
        self.desired_y = 0.0

    def odom_callback(self, msg):
        self.boat_x = msg.pose.pose.position.x
        self.boat_y = msg.pose.pose.position.y

    def desired_position_callback(self, msg):
        self.desired_x = msg.x  # Ajuste para Point
        self.desired_y = msg.y  # Ajuste para Point

    def init_func(self):
        self.boat_point.set_data([], [])
        self.desired_point.set_data([], [])
        return self.boat_point, self.desired_point

    def update(self, frame):
        
        if self.desired_x is not None and self.desired_y is not None:
            self.desired_point.set_data(self.desired_x, self.desired_y)
        
        if self.boat_x is not None and self.boat_y is not None:
            self.boat_point.set_data(self.boat_x, self.boat_y)

        return self.boat_point, self.desired_point

    def animate(self):
        anim = FuncAnimation(self.fig, self.update, frames=len(self.t), init_func=self.init_func, blit=True, interval=self.ts)
        while not rospy.is_shutdown():
            anim.event_source.start()
            plt.pause(self.ts)  
            self.rate.sleep()  
        plt.show()

animation = TrajectoryAnimation()
animation.animate()
