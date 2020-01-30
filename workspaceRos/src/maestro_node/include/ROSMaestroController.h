/* 
 * File:   ROSMaestroController.h
 * Author: raffaello
 *
 * Created on 25 November 2013, 12:52
 */

#ifndef ROSMAESTROCONTROLLER_H
#define	ROSMAESTROCONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "PololuController.h"

class ROSMaestroController : public PololuController {
public:
    ROSMaestroController(std::string name_node, const ros::NodeHandle& nh, TimeoutSerial* serial);
    virtual ~ROSMaestroController();

    void loadConfiguration();
    void addServo(std::string name, int channel);
private:
    std::string name_node_;
    ros::NodeHandle nh_; //NameSpace for bridge controller

    typedef struct servo {
        std::string name;
        int channel;
    } servo_t;
    
    std::map<std::string, int> servo;
    
    //-Standard ROS publisher
    ros::Publisher pub_joint;
    //-Standard ROS subscriber
    ros::Subscriber sub_joint;

    std::string tf_joint_string_;
    std::vector<servo_t> list_servo;
    sensor_msgs::JointState joint;
    ros::Timer timer_;

    void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void connectCallback(const ros::SingleSubscriberPublisher& pub);
    void timerCallback(const ros::TimerEvent& event);
};

#endif	/* ROSMAESTROCONTROLLER_H */

