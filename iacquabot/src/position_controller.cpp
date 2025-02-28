#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class PositionController {
public:
    PositionController() {
        ros::NodeHandle nh;
        pos_sub = nh.subscribe("/iacquabot/desired_position", 10, &PositionController::callbackPosDeseada, this);
        odom_sub = nh.subscribe("/boat/odom", 10, &PositionController::callbackPosActual, this);
        vel_pub = nh.advertise<geometry_msgs::Twist>("/iacquabot/cmd_vel", 10);

        k_p = 1.0;
        k_theta = 1.0;
        u_max = 3.0;
        w_max = 3.0;
        actual_position.setZero();
        desired_position.setZero();
        actual_theta = 0.0;
    }

    void callbackPosDeseada(const geometry_msgs::Pose::ConstPtr& msg) {
        desired_position << msg->position.x, msg->position.y;
    }

    void callbackPosActual(const nav_msgs::Odometry::ConstPtr& msg) {
        actual_position << msg->pose.pose.position.x, msg->pose.pose.position.y;
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, actual_theta);
    }

    void computeControl() {
        Eigen::Vector2d error = desired_position - actual_position;
        double error_norm = error.norm();
    
        if (error_norm < 0.15) {
            publishVelocity(0.0, 0.0);
            return;
        }
        double theta_d = atan2(error.y(), error.x());
        double e_theta = atan2(sin(theta_d - actual_theta), cos(theta_d - actual_theta));
        
        double u = k_p * error.norm();
        double w = k_theta * e_theta;
        
        u = std::max(-u_max, std::min(u_max, u));
        w = std::max(-w_max, std::min(w_max, w));
        
        publishVelocity(u, w);
    }

    void publishVelocity(double u, double w) {
        geometry_msgs::Twist msg;
        msg.linear.x = u;
        msg.angular.z = w;
        vel_pub.publish(msg);
    }

    void run() {
        ros::Rate rate(50);
        while (ros::ok()) {
            ros::spinOnce();
            computeControl();
            rate.sleep();
        }
    }

private:
    ros::Subscriber pos_sub, odom_sub;
    ros::Publisher vel_pub;
    Eigen::Vector2d desired_position, actual_position;
    double actual_theta;
    double k_p, k_theta;
    double u_max, w_max;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "position_controller");
    PositionController controller;
    controller.run();
    return 0;
}
