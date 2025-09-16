#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3
import math

pub = None

def smooth_curve(val, alpha=2.0, deadzone=0.05):
    """
    Suaviza un valor usando una curva exponencial + zona muerta.
    val: valor en [-1, 1]
    alpha: exponente para suavizado
    deadzone: rango sin respuesta (ej: 0.05 = 5%)
    """
    if abs(val) < deadzone:
        return 0.0
    return math.copysign(abs(val) ** alpha, val)

def apriltag_cb(msg):
    # x = desplazamiento en X (%), y = desplazamiento en Y (%)
    x_percent = msg.x
    y_percent = msg.y

    # Suavizar con curva + zona muerta en X
    x_smooth = smooth_curve(x_percent, alpha=2.0, deadzone=0.1)
    y_smooth = smooth_curve(y_percent, alpha=1.0, deadzone=0.0)

    # Convertir a control
    throttle = -y_smooth    # adelante/atrás
    steer    = -x_smooth    # izquierda/derecha

    left  = throttle + steer
    right = throttle - steer

    cmd = Vector3(x=left, y=right, z=0.0)
    pub.publish(cmd)

if __name__ == "__main__":
    rospy.init_node("apriltag_to_thruster")
    pub = rospy.Publisher("/boat/cmd_thruster", Vector3, queue_size=10)
    rospy.Subscriber("/apriltag_cmd", Vector3, apriltag_cb)
    rospy.spin()
