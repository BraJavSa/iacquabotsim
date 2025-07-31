#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry

class OdometerNode:
    def __init__(self):
        # Inicializa el nodo
        rospy.init_node('world2body_node', anonymous=True)

        # Suscripción al tópico de odometría
        self.subscriber = rospy.Subscriber('/wamv/sensors/position/p3d_wamv', Odometry, self.odometry_callback)

        # Publicador para el nuevo tópico de odometría
        self.odom_pub = rospy.Publisher("boat/odom", Odometry, queue_size=10)

        # Tiempo de publicación a 100 Hz
        self.rate = rospy.Rate(100)  # 100 Hz

    def odometry_callback(self, msg):
        # Toma los datos de la odometría
        odom_data = msg

        # Extrae la posición
        position = odom_data.pose.pose.position
        orientation = odom_data.pose.pose.orientation

        # Extrae las velocidades
        linear_velocity = odom_data.twist.twist.linear
        angular_velocity = odom_data.twist.twist.angular

        # Convertir orientación de cuaterniones a ángulo de Euler (yaw)
        _, _, psi = self.quaternion_to_euler(orientation)

        # Cálculo de la matriz de rotación
        J = np.array([[np.cos(psi), -np.sin(psi), 0],
                      [np.sin(psi),  np.cos(psi), 0],
                      [0, 0, 1]])

        # Crear el vector de velocidades en el marco inercial
        dot_eta = np.array([linear_velocity.x, linear_velocity.y, angular_velocity.z])

        # Calcular las velocidades en el marco de cuerpo
        nu = np.linalg.inv(J).dot(dot_eta)

        # Crear nuevo mensaje de odometría
        new_odom = Odometry()
        new_odom.header.stamp = rospy.Time.now()
        new_odom.header.frame_id = "odom"
        new_odom.child_frame_id = "base_link"

        # Asigna la nueva posición y orientación
        new_odom.pose.pose.position = position
        new_odom.pose.pose.orientation = orientation

        # Asigna las velocidades en el marco de cuerpo
        new_odom.twist.twist.linear.x = nu[0]  # Velocidad en X (cuerpo)
        new_odom.twist.twist.linear.y = nu[1]  # Velocidad en Y (cuerpo)
        new_odom.twist.twist.angular.z = nu[2]  # Velocidad angular (cuerpo)

        # Publica la nueva odometría
        self.odom_pub.publish(new_odom)

    def quaternion_to_euler(self, orientation):
        """
        Convierte cuaterniones a ángulos de Euler (yaw).
        """
        # Extraer los componentes del cuaternión
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        
        # Calcular yaw (psi)
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return (0.0, 0.0, yaw)  # Retorna solo el yaw

    def run(self):
        # Ciclo de ejecución del nodo
        while not rospy.is_shutdown():
            self.rate.sleep()  # Mantiene la tasa de 100 Hz

if __name__ == '__main__':
    try:
        odometer_node = OdometerNode()
        odometer_node.run()
    except rospy.ROSInterruptException:
        pass
