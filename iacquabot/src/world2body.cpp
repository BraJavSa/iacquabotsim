#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

class OdometerNode {
public:
    OdometerNode() {
        ros::NodeHandle nh;

        odom_sub_ = nh.subscribe("/iacquabot/sensors/position/p3d_wamv", 10, &OdometerNode::odometryCallback, this);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("boat/odom", 10);
    }

    void spin() {
        ros::Rate rate(100);  // 100 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::Subscriber odom_sub_;
    ros::Publisher odom_pub_;

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        nav_msgs::Odometry new_odom;

        // Extraer posición y orientación
        geometry_msgs::Point position = msg->pose.pose.position;
        geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;

        // Extraer velocidades lineales y angulares
        geometry_msgs::Vector3 linear_velocity = msg->twist.twist.linear;
        geometry_msgs::Vector3 angular_velocity = msg->twist.twist.angular;

        // Convertir cuaternión a yaw
        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // Construir matriz de rotación (solo en 2D, yaw)
        double c = std::cos(yaw);
        double s = std::sin(yaw);
        double inv_det = 1.0 / (c * c + s * s);  // Por seguridad (aunque en teoría siempre es 1)

        // Transformar velocidades del marco inercial al marco cuerpo
        double vx_inertial = linear_velocity.x;
        double vy_inertial = linear_velocity.y;
        double wz_inertial = angular_velocity.z;

        double vx_body =  inv_det * ( c * vx_inertial + s * vy_inertial );
        double vy_body =  inv_det * (-s * vx_inertial + c * vy_inertial );
        double wz_body = wz_inertial;

        // Preparar nuevo mensaje de odometría
        new_odom.header.stamp = ros::Time::now();
        new_odom.header.frame_id = "odom";
        new_odom.child_frame_id = "base_link";

        new_odom.pose.pose.position = position;
        new_odom.pose.pose.orientation = orientation;

        new_odom.twist.twist.linear.x = vx_body;
        new_odom.twist.twist.linear.y = vy_body;
        new_odom.twist.twist.angular.z = wz_body;

        // Publicar mensaje
        odom_pub_.publish(new_odom);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "world2body_node");
    OdometerNode node;
    node.spin();
    return 0;
}
