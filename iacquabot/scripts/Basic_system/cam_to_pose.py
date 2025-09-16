#!/usr/bin/env python3
import rospy
import socket
from geometry_msgs.msg import Twist

# Parámetros
u_max = 1.2
w_max = 0.55
d_ref = 100.0   # cm

Kp_dist = 2.0
Kp_lat = 1.5

# UDP
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

def controller(tx, tz):
    # --- Control lateral ---
    err_lat = (tx / tz) if tz > 0 else 0.0
    w = Kp_lat * err_lat
    w = max(min(w, w_max), -w_max)

    # --- Control distancia ---
    err_dist = (tz - d_ref) / 100.0   # en metros
    u = Kp_dist * err_dist
    u = max(min(u, u_max), -u_max)

    return u, w

def main():
    rospy.init_node("station_keeping_controller")
    vel_pub = rospy.Publisher("iacquabot/cmd_vel", Twist, queue_size=10)

    last_msg_time = rospy.Time.now()
    timeout_sec = 2.0
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        try:
            data, addr = sock.recvfrom(1024)
            msg = data.decode().strip()

            # Ejemplo: "ID:1 T:-2.4,-2.4,133.9 R:225.3,25.9,201.5"
            parts = msg.split()
            tx, ty, tz = [float(x) for x in parts[1][2:].split(",")]

            u, w = controller(tx, tz)

            cmd = Twist()
            cmd.linear.x = u
            cmd.angular.z = w
            vel_pub.publish(cmd)

            rospy.loginfo("tx=%.1f, tz=%.1f -> u=%.2f m/s, w=%.2f rad/s",
                          tx, tz, u, w)

            last_msg_time = rospy.Time.now()

        except BlockingIOError:
            pass
        except Exception as e:
            rospy.logwarn("Error parsing mensaje: %s", str(e))

        # watchdog
        if (rospy.Time.now() - last_msg_time).to_sec() > timeout_sec:
            cmd = Twist()  # velocidades cero
            vel_pub.publish(cmd)

        rate.sleep()

if __name__ == "__main__":
    main()
