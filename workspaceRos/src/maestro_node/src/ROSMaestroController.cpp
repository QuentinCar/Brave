/* 
 * File:   ROSMaestroController.cpp
 * Author: raffaello
 * 
 * Created on 25 November 2013, 12:52
 */

#include "ROSMaestroController.h"

const std::string tf_string = "tf";
const std::string joint_string = "joint_states";
const std::string command_string = "command";

using namespace std;

ROSMaestroController::ROSMaestroController(std::string name_node, const ros::NodeHandle& nh, TimeoutSerial* serial)
: PololuController(serial), nh_(nh), name_node_(name_node) {

    //JointState position
    pub_joint = nh_.advertise<sensor_msgs::JointState>(name_node + "/" + joint_string, 1,
            boost::bind(&ROSMaestroController::connectCallback, this, _1));
    sub_joint = nh_.subscribe(name_node + "/" + command_string + "/" + joint_string, 1, &ROSMaestroController::jointCallback, this);


    //Timer
    timer_ = nh_.createTimer(ros::Duration(1), &ROSMaestroController::timerCallback, this, false, false);
}

ROSMaestroController::~ROSMaestroController() {
}

void ROSMaestroController::timerCallback(const ros::TimerEvent& event) {
    ros::Time now = ros::Time::now();
    double rate = 1;
    nh_.getParam(name_node_ + "/timer/rate", rate);
    timer_.setPeriod(ros::Duration(1 / rate));
    int i = 0;
    for (map<string, int>::iterator ii = servo.begin(); ii != servo.end(); ++ii) {
        joint.name[i] = (*ii).first;
        joint.position[i] = maestroGetAngle((*ii).second);
        joint.velocity[i] = 0;
        joint.effort[i] = 0;
        i++;
    }
    joint.header.stamp = now;
    pub_joint.publish(joint);

    if (pub_joint.getNumSubscribers() == 0) {
        ROS_INFO("Wait users");
        timer_.stop();
    }
}

void ROSMaestroController::connectCallback(const ros::SingleSubscriberPublisher& pub) {
    ROS_INFO("Connect: %s - %s", pub.getSubscriberName().c_str(), pub.getTopic().c_str());
    timer_.start();
}

void ROSMaestroController::jointCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    const sensor_msgs::JointState* write_joint = msg.get();
    int length = write_joint->name.size();
    for (int i = 0; i < length; ++i) {
        string name = write_joint->name[i];
        //ROS_INFO("-> Servo: %s",name.c_str());
        if (servo.find(name) != servo.end()) {
            int channel = servo[name];
						//ROS_INFO("-> Servo: %s - Ch:%d",name.c_str(), channel);
            cout << "jointCallback" <<endl;
            maestroSetAngle(channel, write_joint->position[i]);
        }
    }
}

void ROSMaestroController::loadConfiguration() {
    if (nh_.hasParam(name_node_ + "/servo")) {
        int length = 0;
        if (nh_.hasParam(name_node_ + "/servo/length")) {
            nh_.getParam(name_node_ + "/servo/length", length);
            joint.name.resize(length);
            joint.position.resize(length);
            joint.velocity.resize(length);
            joint.effort.resize(length);
            for (int i = 0; i < length; ++i) {
                servo_t temp_servo;
                ostringstream convert; // stream used for the conversion
                convert << i; // insert the textual representation of 'repeat' in the characters in the stream
                nh_.getParam(name_node_ + "/servo/name/" + convert.str(), temp_servo.name);
                nh_.getParam(name_node_ + "/servo/channel/" + convert.str(), temp_servo.channel);
                servo[temp_servo.name] = temp_servo.channel;
                //cout << "loadConfiguration" <<endl;
                maestroSetAngle(temp_servo.channel, 1460); //mid pwm signal 1250
            }
            list_servo.resize(length);
        } else {
            ROS_ERROR("Unknown number of servos");
        }
    } else {
        ROS_ERROR("Never configuration for servo");
    }
    //Set timer rate
    double rate = 1;
    if (nh_.hasParam(name_node_ + "/timer/rate")) {
        nh_.getParam(name_node_ + "/timer/rate", rate);
        ROS_DEBUG("Sync parameter /timer/rate: load - %f Hz", rate);
    } else {
        nh_.setParam(name_node_ + "/timer/rate", rate);
        ROS_DEBUG("Sync parameter /timer/rate: set - %f Hz", rate);
    }
    timer_.setPeriod(ros::Duration(1 / rate));
    //Names TF
    if (nh_.hasParam(name_node_ + "/" + tf_string)) {
        ROS_DEBUG("Sync parameter %s: load", tf_string.c_str());
        nh_.param<std::string>("/" + name_node_ + "/" + tf_string + "/" + joint_string, tf_joint_string_, tf_joint_string_);
    } else {
        ROS_DEBUG("Sync parameter %s: set", tf_string.c_str());
        tf_joint_string_ = joint_string;
        nh_.setParam("/" + name_node_ + "/" + tf_string + "/" + joint_string, tf_joint_string_);
    }
    joint.header.frame_id = tf_joint_string_;
}

void ROSMaestroController::addServo(std::string name, int channel) {
    int length = 0;
    servo_t temp_servo;
    ostringstream convert; // stream used for the conversion
    temp_servo.name = name;
    temp_servo.channel = channel;
    nh_.getParam(name_node_ + "/servo/length", length);
    nh_.setParam(name_node_ + "/servo/length", length + 1);
    convert << length; // insert the textual representation of 'repeat' in the characters in the stream
    joint.name.resize(length + 1);
    joint.position.resize(length + 1);
    joint.velocity.resize(length + 1);
    joint.effort.resize(length + 1);
    list_servo.resize(length + 1);
    //list_servo.push_back(servo);
    nh_.setParam(name_node_ + "/servo/name/" + convert.str(), temp_servo.name);
    nh_.setParam(name_node_ + "/servo/channel/" + convert.str(), temp_servo.channel);
    servo[temp_servo.name] = temp_servo.channel;
    cout << "addServo" <<endl;
    maestroSetAngle(temp_servo.channel, 1700);
}
