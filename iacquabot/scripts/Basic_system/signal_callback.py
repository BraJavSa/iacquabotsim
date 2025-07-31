#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, Float32MultiArray

class ThrusterControl:
    def __init__(self):
        # Inicialización del nodo
        rospy.init_node('thruster_control', anonymous=True)
        self.left_front_thruster_pub = rospy.Publisher('/wamv/thrusters/left_front_thrust_cmd', Float32, queue_size=10)
        self.left_rear_thruster_pub = rospy.Publisher('/wamv/thrusters/left_rear_thrust_cmd', Float32, queue_size=10)
        self.right_front_thruster_pub = rospy.Publisher('/wamv/thrusters/right_front_thrust_cmd', Float32, queue_size=10)
        self.right_rear_thruster_pub = rospy.Publisher('/wamv/thrusters/right_rear_thrust_cmd', Float32, queue_size=10)
        self.publish_rate = rospy.Rate(100)  
        self.left_signal = 0
        self.right_signal = 0

        rospy.Subscriber('/wamv/signals', Float32MultiArray, self.signal_callback)

    def signal_callback(self, msg):
        """Callback que actualiza las señales recibidas desde /wamv/signals"""
        if len(msg.data) == 2:
            self.left_signal = msg.data[0]
            self.right_signal = msg.data[1]

    def run(self):
        while not rospy.is_shutdown():
            # Publicar en los 4 propulsores en configuración diferencial
            self.left_front_thruster_pub.publish(self.left_signal)
            self.left_rear_thruster_pub.publish(self.left_signal)
            self.right_front_thruster_pub.publish(self.right_signal)
            self.right_rear_thruster_pub.publish(self.right_signal)

            # Mantener la frecuencia de publicación (100 Hz)
            self.publish_rate.sleep()

if __name__ == '__main__':
    try:
        thruster_controller = ThrusterControl()
        thruster_controller.run()
    except rospy.ROSInterruptException:
        pass
