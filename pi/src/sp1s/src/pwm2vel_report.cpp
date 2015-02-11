#include "ros/ros.h"
#include "std_msgs/String.h"
#include <wiringPi.h>
#include <sstream>
#include "veldetect.h"
#include "wheelpwm.h"
#include "sp1s/wheels_dir.h"

#define DBL_ERROR   0.000000001

// pwm = vel * dFactor + dOffset
// the last velocity in arrVel come from pwm == 100
// pwm step in arrVel is 10
void computeFactorOffset(double* arrVel, long lSize,
		double& dFactor, double& dOffset, double& variance)
{
	// Compute the average dFactor from each pair of samples
	double* arrFactor = new double[lSize - 1];
	double dSum = 0.0;
	double dStepFactor = 0.0;
	long length = 0;
	for(long i=1;i< lSize; i++)
	{
		if(fabs(arrVel[i] - arrVel[i-1]) < DBL_ERROR
				|| arrVel[i-1] < DBL_ERROR)
			continue;

		dStepFactor = 10.0 / ( arrVel[i] - arrVel[i-1] );
		arrFactor[length] = dStepFactor;
		dSum += dStepFactor;
		length++;
	}
    double dAvg = dSum / length;
    dFactor = dAvg;
    dOffset = 100.0 - arrVel[lSize - 1]*dFactor;

    // compute variance
    variance = 0.0;
    double dTargetPwm = 110.0;
    double dComputeVel = 0.0;
    for(long i=lSize - 1; i>= 0; i--)
    {
    	dTargetPwm -= 10.0;
    	if(arrVel[i]< DBL_ERROR)
    		continue;
    	dComputeVel = arrVel[i]*dFactor + dOffset;
    	variance += pow(dComputeVel - dTargetPwm, 2 );
    }
	delete[] arrFactor;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pwm2vel_reporter");
    ros::NodeHandle n;
    ros::Publisher velDir_pub = n.advertise<sp1s::wheels_dir>("wheels_dir", 10);
    wiringPiSetup();
    
    int left_forward, left_backward, right_forward, right_backward;
    n.param("left_forward", left_forward, 1);
    n.param("left_backward", left_backward, 4);
    n.param("right_forward", right_forward, 6);
    n.param("right_backward", right_backward, 5);

    CWheelpwm left_pwm(left_forward, left_backward);
    CWheelpwm right_pwm(right_forward, right_backward);
    
    int left_vel_pin, right_vel_pin;
    n.param("left_vel_pin", left_vel_pin, 28);
    n.param("right_vel_pin", right_vel_pin, 29);
    CVeldetect veldetect;
    veldetect.init( left_vel_pin, right_vel_pin );
/*
    ROS_INFO("left_forward %d", left_forward);
    ROS_INFO("left_backward %d", left_backward);
    ROS_INFO("right_forward %d", right_forward);
    ROS_INFO("right_backward %d", right_backward);
*/
    int vel_sampling_rate = 1;
    n.param("vel_sampling_rate", vel_sampling_rate, 1); 
    ros::Rate loop_rate(vel_sampling_rate);    
    
    // publish forward direction to base_control
    sp1s::wheels_dir dir;
    dir.left_forward = true;
    dir.right_forward = true;
    velDir_pub.publish(dir);

    int last_pwm = 0;
    left_pwm.forward(10);
    right_pwm.forward(10);
    ros::spinOnce();
    loop_rate.sleep();
    double arrLeftVel[10];
    double arrRightVel[10];
    long i = 0;

    for(int pwm = 10; pwm <= 110; pwm += 10 )
    {
		if(last_pwm > 0)
		{
			double dLeftVel = veldetect.computeLeftVel();
			double dRightVel = veldetect.computeRightVel();
			ROS_INFO("Left Forword PWM: %d	velocity:%f", last_pwm, dLeftVel);
			ROS_INFO("Right Forword PWM: %d	velocity:%f", last_pwm, dRightVel);
			i++;
			arrLeftVel[i] = dLeftVel;
			arrRightVel[i] = dRightVel;
		}
		if(pwm > 100 )
			  continue;
		left_pwm.forward(pwm);
		right_pwm.forward(pwm);
		veldetect.Reset();
		last_pwm = pwm;

		ros::spinOnce();
		loop_rate.sleep();
    }

    double dFactor,  dOffset,  variance;
    computeFactorOffset(arrLeftVel+1,9,dFactor,dOffset,variance);
    ROS_INFO("Left Forward Factor: %f	Offset:%f	variance:%f ",
    		dFactor, dOffset, variance);
    computeFactorOffset(arrRightVel+1,9,dFactor,dOffset,variance);
        ROS_INFO("Right Forward Factor: %f	Offset:%f	variance:%f ",
        		dFactor, dOffset, variance);

    // publish backward direction to base_control
    dir.left_forward = false;
    dir.right_forward = false;
    velDir_pub.publish(dir);

    left_pwm.backward(10);
    right_pwm.backward(10);
    ros::spinOnce();
    loop_rate.sleep();
    last_pwm = 0;
    i = 0;

    for(int pwm = 10; pwm <= 110; pwm += 10 )
    {	
		if(last_pwm > 0)
		{
			double dLeftVel = veldetect.computeLeftVel();
			double dRightVel = veldetect.computeRightVel();
			ROS_INFO("Left Backward PWM: %d	velocity:%f", last_pwm, dLeftVel);
			ROS_INFO("Right Backward PWM: %d	velocity:%f", last_pwm, dRightVel);
			i++;
			arrLeftVel[i] = dLeftVel;
			arrRightVel[i] = dRightVel;
		}
		if(pwm > 100 )
			  continue;
        left_pwm.backward(pwm);
        right_pwm.backward(pwm);
        veldetect.Reset();
        last_pwm = pwm;

    	ros::spinOnce();
    	loop_rate.sleep();
    }
    computeFactorOffset(arrLeftVel+1,9,dFactor,dOffset,variance);
        ROS_INFO("Left Forward Factor: %f	Offset:%f	variance:%f ",
        		dFactor, dOffset, variance);
    computeFactorOffset(arrRightVel+1,9,dFactor,dOffset,variance);
        ROS_INFO("Right Forward Factor: %f	Offset:%f	variance:%f ",
        		dFactor, dOffset, variance);
}
