// follow tutoral http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <wiringPi.h>
#include <sstream>
#include "sp1s/velocity.h"

#define LEFT_PIN 	28
#define RIGHT_PIN 	29
#define PI 		3.1415926
#define DIAMETER 	0.068
#define HOLES_ON_WHEEL  20

// the event counter 
volatile int leftCounter = 0;
volatile int rightCounter = 0;

// -------------------------------------------------------------------------
// xxxInterrupt:  called every time an event occurs
void leftInterrupt(void) {
   leftCounter++;
   //ROS_INFO("I heard: [%d]", leftCounter);
}

void rightInterrupt(void) {
   rightCounter++;
}

// compute the linear velocity in unit 'm/s'
float computeVel( ros::Time current_time, ros::Time last_time,
                  int counter)
{
    double distance = counter * PI * DIAMETER / HOLES_ON_WHEEL;
    double secs = (current_time - last_time).toSec();
    return distance / secs;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_pub");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
    
    wiringPiSetup();
    pinMode(LEFT_PIN, INPUT );
    pinMode(RIGHT_PIN, INPUT );

    // set Pin to generate an interrupt on high-to-low transitions
    // and attach xxxInterrupt() to the interrupt
    if ( wiringPiISR (LEFT_PIN, INT_EDGE_FALLING, &leftInterrupt) < 0 
       ||wiringPiISR (RIGHT_PIN, INT_EDGE_FALLING, &rightInterrupt) < 0 ) {
       fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
       return 1;
    }
    
    ros::Time current_time, last_time;    
    last_time = ros::Time::now();

    ros::Rate loop_rate(4);
    nav_msgs::Odometry odom;

    //set the position
    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = 0;
    float vl, vr;
    while (ros::ok())
    {
        current_time = ros::Time::now();
        vl = computeVel( current_time, last_time, leftCounter);
        vr = computeVel( current_time, last_time, rightCounter);	
        leftCounter = 0;
        rightCounter = 0;

    	//set the velocity
    	odom.child_frame_id = "base_link";
    	odom.twist.twist.linear.x = ( vl + vr ) / 2;
    	odom.twist.twist.linear.y = 0;
    	odom.twist.twist.angular.z = 0.0;

    	//publish the message
    	odom_pub.publish(odom);
        last_time = current_time;
    	ros::spinOnce();

    	loop_rate.sleep();
    }
}
