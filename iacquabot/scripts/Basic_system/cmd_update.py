#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
import time

class ThrustController:
    def __init__(self):
        # Inicialización de nodo de ROS
        rospy.init_node('cmd_update', anonymous=True)

        # Configuración de tópicos de publicación para cada thruster
        self.left_front_thruster_pub = rospy.Publisher('/wamv/thrusters/left_front_thrust_cmd', Float32, queue_size=10)
        self.left_rear_thruster_pub = rospy.Publisher('/wamv/thrusters/left_rear_front_thrust_cmd', Float32, queue_size=10)
        self.right_front_thruster_pub = rospy.Publisher('/wamv/thrusters/right_front_thrust_cmd', Float32, queue_size=10)
        self.right_rear_thruster_pub = rospy.Publisher('/wamv/thrusters/right_rear_thrust_cmd', Float32, queue_size=10)

        # Valores de thrust inicializados en cero
        self.left_thrust = 0.0
        self.right_thrust = 0.0

        # Suscripción al tópico /boat/cmd_thruster
        rospy.Subscriber('/boat/cmd_thruster', Vector3, self.cmd_callback)

    def cmd_callback(self, msg):
        """Callback que recibe el vector de thrust y actualiza los valores de thrust para los thrusters."""
        self.left_thrust = msg.x  # Asignar al thruster izquierdo
        self.right_thrust = msg.y  # Asignar al thruster derecho

    def update_thrusters(self):
        """Publica los valores de thrust actualizados en los tópicos correspondientes."""
        rate = rospy.Rate(300)  # 200 Hz
        while not rospy.is_shutdown():
            # Publicar valores de thrust a cada thruster
            self.left_front_thruster_pub.publish(self.left_thrust)
            self.left_rear_thruster_pub.publish(self.left_thrust)
            self.right_front_thruster_pub.publish(self.right_thrust)
            self.right_rear_thruster_pub.publish(self.right_thrust)

            rate.sleep()  # Esperar para mantener la frecuencia de 200 Hz

if __name__ == "__main__":
    thrust_controller = ThrustController()
    thrust_controller.update_thrusters()
