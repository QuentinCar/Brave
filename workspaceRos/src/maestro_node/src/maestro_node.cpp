#include <cstdlib>
#include <iostream>
#include <ros/ros.h>
#include "TimeoutSerial.h"
#include "ROSMaestroController.h"
#include "PololuController.h"
#include "std_msgs/Float64.h"

using namespace std;
using namespace boost;

std::string name_node = "maestro_node";
double targetRudd, targetMSail, targetFSail;

void callbackCommand(const std_msgs::Float64::ConstPtr& msg)
{
    servo = msg.servo
    pwm = msg.pwm

    pololu.maestroSetAngle(servo, pwm)
}

int main(int argc, char** argv) {

    ROS_INFO("Main");

    ros::init(argc, argv, name_node);
    ros::NodeHandle n;

    if (argc != 2) {
        ROS_ERROR("need serial port name as argument");
        return -1;
    };

    //Name serial port
    std::string serial_port(argv[1]);
    
    ros::NodeHandle nh;

    TimeoutSerial serial(serial_port, 9600);
    serial.setTimeout(posix_time::seconds(1));

    ROS_INFO("Start maestro_node");
    /*
    ROSMaestroController controller(name_node, nh, &serial);
    controller.loadConfiguration();
	*/

    PololuController pololu(&serial);

    ros::Subscriber sub = n.subscribe("command", 1, callbackCommand);

	//pololu.maestroSetAngle(0, 1250);
	//pololu.maestroSetAngle(1, 1460);
	//pololu.maestroSetAngle(3, 1710);


    ros::spin();

    ROS_INFO("Close maestro_node");

    serial.close();

    return 0;
}

