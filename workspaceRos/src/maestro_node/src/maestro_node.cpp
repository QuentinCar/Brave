#include <cstdlib>
#include <iostream>
#include <ros/ros.h>
#include "TimeoutSerial.h"
#include "ROSMaestroController.h"
#include "PololuController.h"

#include <maestro_node/Command.h>

using namespace std;
using namespace boost;

std::string name_node = "maestro_node";
double targetRudd=1250;
double targetMSail=1460; 
double targetFSail=1710;



void callbackCommand(const maestro_node::Command::ConstPtr& msg)
{

    targetRudd = double(msg->pwm_rudder);//Rudder
    targetMSail = double(msg->pwm_main_sail);//mainsail
    targetFSail = double(msg->pwm_fore_sail);//foresail

}

int main(int argc, char** argv) {

    ROS_INFO("Start maestro_node");

    ros::init(argc, argv, name_node);
    ros::NodeHandle n;

    /*************Serial**************/

    if (argc != 2) {
        ROS_ERROR("need serial port name as argument");
        return -1;
    };

    std::string serial_port(argv[1]);
    TimeoutSerial serial(serial_port, 9600);
    serial.setTimeout(posix_time::seconds(1));
    PololuController pololu(&serial);

    /***************Subscribe*******************/

    ros::Subscriber sub = n.subscribe("/Command", 1, callbackCommand);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {


    	pololu.maestroSetAngle(0, targetRudd);
    	pololu.maestroSetAngle(1, targetMSail);
    	pololu.maestroSetAngle(3, targetFSail);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Close maestro_node");

    serial.close();

    return 0;
}

