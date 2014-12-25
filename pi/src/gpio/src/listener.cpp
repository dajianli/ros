#include "ros/ros.h"
#include "std_msgs/String.h"
#include <wiringPi.h>
#include "gpio/pinMode.h"
 

/*
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}*/

void chatterCallback(const gpio::pinModePtr& msg)
{
  ROS_INFO("I heard: pin [%i]  I/O = [%i]", msg->pin, msg->output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gpioExecuteNode");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("gpioTopic", 100, chatterCallback);
    wiringPiSetup();

    ros::spin();
}
