// follow tutoral http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

#include "ros/ros.h"
#include "std_msgs/String.h"
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
    ros::init(argc, argv, "basecontrol_slave");
    ros::NodeHandle n;
    ros::Publisher slave_pub = n.advertise<sp1s::velocity>("basecontrol_slave", 10);
    
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

    ros::Rate loop_rate(10);
    sp1s::velocity vel;

    while (ros::ok())
    {
        current_time = ros::Time::now();        
        
    	//set the velocity
   	vel.vel_left = computeVel( current_time, last_time, leftCounter);
        vel.vel_right = computeVel( current_time, last_time, rightCounter);
	leftCounter = 0;
        rightCounter = 0;

    	//publish the message
    	slave_pub.publish(vel);
        last_time = current_time;
    	ros::spinOnce();

    	loop_rate.sleep();
    }
}
