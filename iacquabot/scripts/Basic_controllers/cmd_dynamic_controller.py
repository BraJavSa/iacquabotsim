#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import scipy.io as sio
import os
from std_msgs.msg import Float32MultiArray

def progressive_clip(u, a, b):
    if abs(u) <= a:
        return u
    else:
        return np.sign(u) * (a + (b - a) * np.tanh((abs(u) - a) / (b - a)))

class VelocityController:
    def __init__(self):
        rospy.init_node('velocity_listener')
        
        self.ts = 0.02  # Sampling interval
        self.k_d = 1.0  # Controller gain
        
        self.desired_velocity = {'u': 0.0, 'v': 0.0, 'r': 0.0}
        self.prev_desired_velocity = {'u': 0.0, 'v': 0.0, 'r': 0.0}
        self.acceleration_desired = {'u': 0.0, 'v': 0.0, 'r': 0.0}
        
        self.actual_velocity = {'u': 0.0, 'v': 0.0, 'r': 0.0}
        
        rospy.Subscriber("/cmd_vel", Twist, self.callback_vel_deseada)
        rospy.Subscriber("/boat/odom", Odometry, self.callback_vel_actual)
        self.signals_pub = rospy.Publisher('/wamv/signals', Float32MultiArray, queue_size=10)
        
        # Load the model parameters (M, D, C)
        self.load_model_parameters()

    def load_model_parameters(self):
        """Loads the model parameters (M, D, C) from the provided file."""
        catkin_ws_path = os.path.expanduser('~/catkin_ws/src/usv_sim/src/USV_Identification/Model_follow')
        delta_data = sio.loadmat(os.path.join(catkin_ws_path, 'Amodel/delta_valores_A_model.mat'))
        delta = delta_data['delta'].flatten()

        self.M = np.array([[delta[0], 0, 0], 
                           [0, delta[1], delta[2]], 
                           [0, delta[2], delta[3]]])
        self.D = np.array([[delta[7], 0, 0], 
                           [0, delta[8], delta[9]], 
                           [0, delta[9], delta[10]]])

    def callback_vel_deseada(self, msg):
        """Receives the desired velocity and calculates the desired acceleration."""
        self.desired_velocity['u'] = msg.linear.x
        self.desired_velocity['v'] = msg.linear.y
        self.desired_velocity['r'] = msg.angular.z
        
        self.acceleration_desired['u'] = (self.desired_velocity['u'] - self.prev_desired_velocity['u']) / self.ts
        self.acceleration_desired['v'] = (self.desired_velocity['v'] - self.prev_desired_velocity['v']) / self.ts
        self.acceleration_desired['r'] = (self.desired_velocity['r'] - self.prev_desired_velocity['r']) / self.ts
        
        self.prev_desired_velocity['u'] = self.desired_velocity['u']
        self.prev_desired_velocity['v'] = self.desired_velocity['v']
        self.prev_desired_velocity['r'] = self.desired_velocity['r']
        
        self.gamma_function()
    
    def callback_vel_actual(self, msg):
        """Receives the actual velocity of the USV."""
        self.actual_velocity['u'] = msg.twist.twist.linear.x
        self.actual_velocity['v'] = msg.twist.twist.linear.y
        self.actual_velocity['r'] = msg.twist.twist.angular.z


    def calcularCmd(self, torque):
        # Parámetros para el caso de torque > 0.01
        A_pos = 0.01
        K_pos = 59.82
        B_pos = 5.0
        v_pos = 0.38
        C_pos = 0.56
        M_pos = 0.28

        # Parámetros para el caso de torque < 0.01
        A_neg = -199.13
        K_neg = -0.09
        B_neg = 8.84
        v_neg = 5.34
        C_neg = 0.99
        M_neg = -0.57

        # Invertir las ecuaciones para calcular cmd a partir de T
        if torque > A_pos:
            # Caso positivo
            cmd = M_pos + (np.log((K_pos - A_pos) / (torque - A_pos)) ** v_pos - 1) / (-B_pos)
        elif torque < A_neg:
            # Caso negativo
            cmd = M_neg + (np.log((K_neg - A_neg) / (torque - A_neg)) ** v_neg - 1) / (-B_neg)
        else:
            # Caso neutro (cuando el torque está cerca de 0)
            cmd = 0

        return cmd

    
    def gamma_function(self):
        """Calculates gamma as desired acceleration plus velocity error multiplied by k_d."""
        gamma = {
            'u': self.acceleration_desired['u'] + (self.desired_velocity['u'] - self.actual_velocity['u']) * self.k_d,
            'v': self.acceleration_desired['v'] + (self.desired_velocity['v'] - self.actual_velocity['v']) * self.k_d,
            'r': self.acceleration_desired['r'] + (self.desired_velocity['r'] - self.actual_velocity['r']) * self.k_d
        }
        self.torque_calculation(gamma)

    def torque_calculation(self, gamma):
        """Calculates the torque using the formula: T = M*gamma + C(v)*v + D*v"""
        v = np.array([self.actual_velocity['u'], self.actual_velocity['v'], self.actual_velocity['r']])
        
        # Calculate C(v) matrix as a function of the velocities (u, v, r)
        C = np.array([[0, -self.M[0][1] * self.actual_velocity['r'], -self.M[0][2] * self.actual_velocity['v']],
                      [self.M[0][1] * self.actual_velocity['r'], 0, self.M[1][2] * self.actual_velocity['u']],
                      [self.M[0][2] * self.actual_velocity['v'], -self.M[1][2] * self.actual_velocity['u'], 0]])
        
        # Calculate torque T
        gamma_array = np.array([gamma['u'], gamma['v'], gamma['r']])
        torque = np.dot(self.M, gamma_array) + np.dot(C, v) + np.dot(self.D, v)
        T_u = torque[0]  # Assuming T_u corresponds to the first component of torque
        T_r = torque[2]  # Assuming T_r corresponds to the third component of torque
        T_left, T_right = self.calculate_motor_torques(T_u, T_r)
        cmd_left = self.calcularCmd(T_left)
        cmd_right = self.calcularCmd(T_right)
        msg = Float32MultiArray()
        msg.data = [cmd_left, cmd_right]
        self.signals_pub.publish(msg)

        
        
    def calculate_motor_torques(self, T_u, T_r, d=1.4):
        # Calculate the left and right motor torques
        T_left = (T_u / 4) - (T_r / (4 * d))
        T_right = (T_u / 4) + (T_r / (4 * d))
        return T_left, T_right
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = VelocityController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
