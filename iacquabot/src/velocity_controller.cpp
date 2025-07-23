#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <geometry_msgs/Vector3.h>

class VelocityController {
public:
    VelocityController() {
        ros::NodeHandle nh;
        vel_sub = nh.subscribe("iacquabot/cmd_vel", 10, &VelocityController::callbackVelDeseada, this);
        odom_sub = nh.subscribe("/boat/odom", 10, &VelocityController::callbackVelActual, this);
        signals_pub = nh.advertise<geometry_msgs::Vector3>("/boat/cmd_thruster", 10);
        
        ts = 0.02L; // Periodo de muestreo 50Hz
        desired_velocity.setZero();
        prev_desired_velocity.setZero();
        acceleration_desired.setZero();
        actual_velocity.setZero();
        
        delta_1  = 122.05714699159923L;
        delta_2  = 72.36702652856437L;
        delta_3  = 1.94662214587859L;
        delta_4  = 141.18381766520358L;
        delta_5  = 56.95914707490276L;
        delta_6  = 35.64555354844870L;
        delta_7  = 21.31359352645391L;
        delta_8  = 69.41907609721954L;
        delta_9  = 30.90596046114065L;
        delta_10 = -1.00889659427417L;
        delta_11 = 214.40956126982840L;
        
        M << delta_1, 0, 0,
             0, delta_2, delta_3,
             0, delta_3, delta_4;
        
        D << delta_8, 0, 0,
             0, delta_9, delta_10,
             0, delta_10, delta_11;
        
        k_d << 2.0L, 0.0L, 10.0L;

        A_pos = 0.000001L;
        K_pos = 40.0209L;
        B_pos = 2.6249L;
        v_pos = 0.1615L;
        C_pos = 0.9432L;
        M_pos = 0.00001L;
        maxForceFwd = 36.3827L;  // Fuerza máxima hacia adelante

        A_neg = -31.4990L;
        K_neg = -0.00001L;
        B_neg = 3.6986L;
        v_neg = 0.3264L;
        C_neg = 0.9713L;
        M_neg = -1.0000L;
        maxForceRev = -28.4393L;  // Fuerza máxima hacia atrás
        }

    
    void callbackVelDeseada(const geometry_msgs::Twist::ConstPtr& msg) {
        desired_velocity << msg->linear.x, msg->linear.y, msg->angular.z;
    }
    
    void callbackVelActual(const nav_msgs::Odometry::ConstPtr& msg) {
        actual_velocity << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.angular.z;
    }
    
    void controlLoop() {
        acceleration_desired = (desired_velocity - prev_desired_velocity) / ts;
        prev_desired_velocity = desired_velocity;
        gammaFunction();
    }
    
    void gammaFunction() {
        Eigen::Matrix<long double, 3, 1> gamma = acceleration_desired + k_d.cwiseProduct(desired_velocity - actual_velocity);
        torqueCalculation(gamma);
    }
    
    void torqueCalculation(const Eigen::Matrix<long double, 3, 1>& gamma) {
        Eigen::Matrix<long double, 3, 1> v = desired_velocity;
        Eigen::Matrix<long double, 3, 3> C;
        C << 0, -delta_5 * v(2), -delta_6 * v(1) - delta_3 * v(2),
             delta_5 * v(2), 0, delta_7 * v(0),
             delta_6 * v(1) + delta_3 * v(2), -delta_7 * v(0), 0;
        
        Eigen::Matrix<long double, 3, 1> torque = M * gamma + C * v + D * v;
        double T_left, T_right;
        calculateMotorTorques(torque(0), torque(2), T_left, T_right);
        double cmd_left, cmd_right;
        calcularCmd(T_left, cmd_left);
        calcularCmd(T_right, cmd_right);
        publishSignals(cmd_left, cmd_right);
    }

    void calcularCmd(double T, double& cmd, double tolerancia = 1e-4, int max_iter = 20) {
        double low = 0.01;
        double high = 1.0;
    
        if (T < 0) {
            low = -1.0;
            high = -0.01;
        } else if (T == 0) {
            cmd = 0.0;
            return;
        }
    
        for (int i = 0; i < max_iter; ++i) {
            double mid = (low + high) / 2.0;
            double T_calculada = 0.0;
            calcularFuerzaPropulsor(mid, T_calculada);
    
            if (std::abs(T_calculada - T) < tolerancia) {
                cmd = mid;
                return;
            } else if (T_calculada < T) {
                low = mid;
            } else {
                high = mid;
            }
        }
    
        cmd = (low + high) / 2.0;  
    }
    
    void calcularFuerzaPropulsor(double cmd, double &T) {
        T = 0.0;
    
        if (cmd > 0.01) {
            T = A_pos + (K_pos - A_pos) / std::pow(C_pos + std::exp(-B_pos * (cmd - M_pos)), 1.0 / v_pos);
        } else if (cmd < -0.01) {
            T = A_neg + (K_neg - A_neg) / std::pow(C_neg + std::exp(-B_neg * (cmd - M_neg)), 1.0 / v_neg);
        } else {
            T = 0;
        }
    
        if (T > maxForceFwd) {
            T = maxForceFwd;
        } else if (T < maxForceRev) {
            T = maxForceRev;
        }
    }

    void calculateMotorTorques(double T_u, double T_r, double& T_left, double& T_right, double d = 1.4) {
        T_left = ((T_u - (T_r / d)) / 2)/2;
        T_right = ((T_u + (T_r / d)) / 2)/2;
    }
    
    void publishSignals(double cmd_left, double cmd_right) {
        geometry_msgs::Vector3 msg;
        msg.x = cmd_left;
        msg.y = cmd_right;
        msg.z = 0.0;  
        signals_pub.publish(msg);
    }
    
    void run() {
        ros::Rate rate(50); // 50 Hz
        while (ros::ok()) {
            ros::spinOnce();
            controlLoop();
            rate.sleep();
        }
    }
    
private:
    ros::Subscriber vel_sub, odom_sub;
    ros::Publisher signals_pub;
    long double ts;
    Eigen::Matrix<long double, 3, 1> k_d;
    Eigen::Matrix<long double, 3, 1> desired_velocity, prev_desired_velocity, acceleration_desired, actual_velocity;
    Eigen::Matrix<long double, 3, 3> M, D;
    long double delta_1, delta_2, delta_3, delta_4, delta_5, delta_6, delta_7, delta_8, delta_9, delta_10, delta_11;
    long double A_pos, K_pos, B_pos, v_pos, C_pos, M_pos, maxForceFwd;
    long double A_neg, K_neg, B_neg, v_neg, C_neg, M_neg, maxForceRev;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_listener");
    VelocityController controller;
    controller.run();
    return 0;
}
