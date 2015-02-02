#include "ros/ros.h"
#include "std_msgs/String.h"
#include <wiringPi.h>
#include <sstream>
#include "veldetect.h"
#include "wheelpwm.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pwm2vel_reporter");
    ros::NodeHandle n;
    wiringPiSetup();
    
    int left_forward, left_backward, right_forward, right_backward;
    n.param("left_forward", left_forward, 15);
    n.param("left_backward", left_backward, 18);
    n.param("right_forward", right_forward, 24);
    n.param("right_backward", right_backward, 23);

    CWheelpwm left_pwm(left_forward, left_backward);
    CWheelpwm right_pwm(right_forward, right_backward);
    
    int left_vel_pin, right_vel_pin;
    n.param("left_vel_pin", left_vel_pin, 28);
    n.param("right_vel_pin", right_vel_pin, 29);
    CVeldetect veldetect;
    veldetect.init( left_vel_pin, right_vel_pin );

    int vel_sampling_rate = 1;
    n.param("vel_sampling_rate", vel_sampling_rate, 1); 
    ros::Rate loop_rate(vel_sampling_rate);    
    
    int last_pwm = 0;
    left_pwm.forward(10);
    right_pwm.forward(10);
    ros::spinOnce();
    loop_rate.sleep();

    for(int pwm = 10; pwm < 100; pwm += 10 )
    {
	if(last_pwm > 0)
	{
		double dLeftVel = veldetect.computeLeftVel();
		double dRightVel = veldetect.computeRightVel();
 		ROS_INFO("Left Forword PWM: %d	velocity:%f", last_pwm, dLeftVel);
 		ROS_INFO("Right Forword PWM: %d	velocity:%f", last_pwm, dRightVel);		
	}
        left_pwm.forward(pwm);        
	right_pwm.forward(pwm);
        veldetect.Reset();
	last_pwm = pwm;

    	ros::spinOnce();
    	loop_rate.sleep();
    }

    left_pwm.backward(10);
    right_pwm.backward(10);
    ros::spinOnce();
    loop_rate.sleep();

    for(int pwm = 10; pwm < 100; pwm += 10 )
    {	
	if(last_pwm > 0)
	{
		double dLeftVel = veldetect.computeLeftVel();
		double dRightVel = veldetect.computeRightVel();
 		ROS_INFO("Left Backward PWM: %d		velocity:%f", last_pwm, dLeftVel);
 		ROS_INFO("Right Backward PWM: %d	velocity:%f", last_pwm, dRightVel);		
	}
        left_pwm.backward(pwm);
	right_pwm.backward(pwm);
        veldetect.Reset();
	last_pwm = pwm;

    	ros::spinOnce();
    	loop_rate.sleep();
    }
}
