#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>

class ThrustController
{
public:
    ThrustController()
    {
        // Inicialización de nodos de ROS
        ros::NodeHandle nh;

        // Configuración de tópicos de publicación para cada thruster
        left_front_thruster_pub = nh.advertise<std_msgs::Float32>("/iacquabot/thrusters/left_front_thrust_cmd", 10);
        left_rear_thruster_pub = nh.advertise<std_msgs::Float32>("/iacquabot/thrusters/left_rear_thrust_cmd", 10);
        right_front_thruster_pub = nh.advertise<std_msgs::Float32>("/iacquabot/thrusters/right_front_thrust_cmd", 10);
        right_rear_thruster_pub = nh.advertise<std_msgs::Float32>("/iacquabot/thrusters/right_rear_thrust_cmd", 10);
        left_thrust = 0.0;
        right_thrust = 0.0;
        cmd_thruster_sub = nh.subscribe("/boat/cmd_thruster", 10, &ThrustController::cmdCallback, this);
    }

    // Callback que recibe el vector de thrust y actualiza los valores de thrust para los thrusters
    void cmdCallback(const geometry_msgs::Vector3::ConstPtr& msg)
    {
        left_thrust = msg->x;  // Asignar al thruster izquierdo
        right_thrust = msg->y;  // Asignar al thruster derecho
    }

    // Publica los valores de thrust actualizados en los tópicos correspondientes
    void updateThrusters()
    {
        ros::Rate rate(300);  // 300 Hz
        while (ros::ok())
        {
            // Crear los mensajes de thrust
            std_msgs::Float32 left_msg;
            left_msg.data = left_thrust;

            std_msgs::Float32 right_msg;
            right_msg.data = right_thrust;

            // Publicar valores de thrust a cada thruster
            left_front_thruster_pub.publish(left_msg);
            left_rear_thruster_pub.publish(left_msg);
            right_front_thruster_pub.publish(right_msg);
            right_rear_thruster_pub.publish(right_msg);

            ros::spinOnce();  // Llamar a los callbacks pendientes
            rate.sleep();     // Esperar para mantener la frecuencia de 300 Hz
        }
    }

private:
    // Publicadores
    ros::Publisher left_front_thruster_pub;
    ros::Publisher left_rear_thruster_pub;
    ros::Publisher right_front_thruster_pub;
    ros::Publisher right_rear_thruster_pub;

    // Suscriptor
    ros::Subscriber cmd_thruster_sub;

    // Variables de thrust
    float left_thrust;
    float right_thrust;
};

int main(int argc, char** argv)
{
    // Inicializar ROS
    ros::init(argc, argv, "cmd_update");

    // Crear objeto ThrustController y ejecutar
    ThrustController thrust_controller;
    thrust_controller.updateThrusters();

    return 0;
}
