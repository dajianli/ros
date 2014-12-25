#include "ros/ros.h"
#include "std_msgs/String.h"
#include <wiringPi.h>
#include "gpio/pinMode.h"
#include "gpio/pinWrite.h"

/*
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}*/

void chatterCallback(const gpio::pinModePtr& msg)
{
  //ROS_INFO("I heard: pinMode [%i]  I/O = [%i]", msg->pin, msg->output);
  pinMode(msg->pin, msg->output == 1 ? OUTPUT : INPUT );
}

void pinWriteCallback(const gpio::pinWritePtr& msg)
{
  //ROS_INFO("I heard: pinWrite [%i]  I/O = [%i]", msg->pin, msg->high);
  digitalWrite(msg->pin, msg->high == 1 ? HIGH : LOW );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gpioExecuteNode");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("gpioPinMode", 100, chatterCallback);
    
    ros::Subscriber sub2= n.subscribe("gpioPinWrite", 100, pinWriteCallback);
    wiringPiSetup();

    ros::spin();
}
