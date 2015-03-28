#include "ros/ros.h"
#include "std_msgs/String.h"
#include <wiringPi.h>
#include <sstream>
#include "veldetect.h"
#include "wheelpwm.h"
#include "sp1s/wheels_dir.h"
#include "dbl_cmp.h"

#define STARTUP_PWM 3

struct VelPwm{
	double velLeft;
	double velRight;
	int    pwmLeft;
	int    pwmRight;
};

// Y = X * dFactor + dOffset
void computeFactorOffset(double* arrX, double* arrY, long lSize,
		double& dFactor, double& dOffset, double& variance)
{
	// Compute the average dFactor from each pair of samples
	double* arrFactor = new double[lSize - 1];
	double dSum = 0.0;
	double dStepFactor = 0.0;
	long length = 0;
	for(long i=1;i< lSize; i++)
	{
		if (DBLCMPEQ(arrY[i],arrY[i - 1])
			|| DBLCMPEQ(arrY[i - 1], 0))
			continue;
		double deltaY = arrY[i] - arrY[i-1];
		double deltaX = arrX[i] - arrX[i-1];
		dStepFactor = deltaY / deltaX;
		arrFactor[length] = dStepFactor;
		dSum += dStepFactor;
		length++;
	}
    double dAvg = dSum / length;
    dFactor = dAvg;
    dOffset = arrY[lSize - 1] - arrX[lSize - 1]*dFactor;

    // compute variance
    variance = 0.0;
    for(long i=1;i< lSize; i++)
    {
	if (DBLCMPEQ(arrX[i], 0))
    		continue;
    	double dComputeY = arrX[i]*dFactor + dOffset;
    	variance += pow(dComputeY - arrY[i], 2 );
    }
    delete[] arrFactor;
}

void GoBackAndForth(int leftPwm, int rightPwm, 
	CWheelpwm *pLeftWheel, CWheelpwm *pRightWheel,
	CVeldetect *pVelDetect, float distance,
	VelPwm *pForwardResult, VelPwm *pBackwardResult)
{
	ros::Rate loop_rate(100);

	// Go forward
	pLeftWheel->forward(10);
	pRightWheel->forward(10);
	loop_rate.sleep();
	
	pLeftWheel->forward(leftPwm);
	pRightWheel->forward(rightPwm);
	pVelDetect->Reset();
	while (DBLCMPLT(pVelDetect->distanceLeft(), distance)
		|| DBLCMPLT(pVelDetect->distanceRight(), distance))
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	//printf("Forward :%f   %f   \n", pVelDetect->distanceLeft(),pVelDetect->distanceRight());
	pForwardResult->velLeft = pVelDetect->computeLeftVel();
	pForwardResult->velRight = pVelDetect->computeRightVel();
	pForwardResult->pwmLeft = leftPwm;
	pForwardResult->pwmRight = rightPwm;

	// Go backward
	pLeftWheel->backward(10);
	pRightWheel->backward(10);
	loop_rate.sleep();
	
	pLeftWheel->backward(leftPwm);
	pRightWheel->backward(rightPwm);
	pVelDetect->Reset();
	while (DBLCMPLT(pVelDetect->distanceLeft(), distance)
		|| DBLCMPLT(pVelDetect->distanceRight(), distance))
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	//printf("Backward:%f   %f   \n", pVelDetect->distanceLeft(),pVelDetect->distanceRight());
	pBackwardResult->velLeft = pVelDetect->computeLeftVel();
	pBackwardResult->velRight = pVelDetect->computeRightVel();
	pBackwardResult->pwmLeft = leftPwm;
	pBackwardResult->pwmRight = rightPwm;

	pLeftWheel->forward(0);
	pRightWheel->forward(0);
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

    int vel_sampling_rate = 1;
    n.param("vel_sampling_rate", vel_sampling_rate, 1); 
    ros::Rate loop_rate(vel_sampling_rate);    
    
    // publish forward direction to base_control
    sp1s::wheels_dir dir;
    dir.left_forward = true;
    dir.right_forward = true;
    velDir_pub.publish(dir);
	
	VelPwm forwards[10], backwards[10]; 
	int iStepPwm = 10 - STARTUP_PWM;
	float DISTANCE = 1.0;

	for (int i = 0; i < 10; i++)
	{
		int pwm = STARTUP_PWM * 10 + i*iStepPwm;
		GoBackAndForth(pwm, pwm + 5, &left_pwm, &right_pwm,
			&veldetect, DISTANCE, forwards + i, backwards + i);		
		sleep(10);
    }

	// stop it
	left_pwm.forward(0);
	right_pwm.forward(0);
	
	double arrVelLeftForward[10],arrVelLeftBackward[10],arrVelRightForward[10],arrVelRightBackward[10];
        double arrPwmLeftForward[10],arrPwmLeftBackward[10],arrPwmRightForward[10],arrPwmRightBackward[10];
	for (int row = 0; row < 10; row++)
	{
		printf("%d   %f   %d    %f    %d   %f   %d    %f", 
			forwards[row].velLeft, forwards[row].pwmLeft, forwards[row].velRight, forwards[row].pwmRight, 
			backwards[row].velLeft, backwards[row].pwmLeft, backwards[row].velRight,backwards[row].pwmRight);
		printf("\n");
		arrVelLeftForward[row] = forwards[row].velLeft;
		arrVelRightForward[row] = forwards[row].velRight;
		arrVelLeftBackward[row] = backwards[row].velLeft;		
		arrVelRightBackward[row] = backwards[row].velRight;
		arrPwmLeftForward[row] = forwards[row].pwmLeft;		
		arrPwmRightForward[row] = forwards[row].pwmRight;
		arrPwmLeftBackward[row] = backwards[row].pwmLeft;
		arrPwmRightBackward[row] = backwards[row].pwmRight;

	}

    double dFactor,  dOffset,  variance;
    computeFactorOffset(arrVelLeftForward,arrPwmLeftForward,10,dFactor,dOffset,variance);
    ROS_INFO("Left Forward Factor: %f	Offset:%f	variance:%f ",
    		dFactor, dOffset, variance);
    computeFactorOffset(arrVelRightForward,arrPwmRightForward,10,dFactor,dOffset,variance);
        ROS_INFO("Right Forward Factor: %f	Offset:%f	variance:%f ",
        		dFactor, dOffset, variance);

    // publish backward direction to base_control
    dir.left_forward = false;
    dir.right_forward = false;
    velDir_pub.publish(dir);

   
    computeFactorOffset(arrVelLeftBackward,arrPwmLeftBackward,10,dFactor,dOffset,variance);
        ROS_INFO("Left backward Factor: %f	Offset:%f	variance:%f ",
        		dFactor, dOffset, variance);
    computeFactorOffset(arrVelRightBackward,arrPwmRightBackward,10,dFactor,dOffset,variance);
        ROS_INFO("Right backward Factor: %f	Offset:%f	variance:%f ",
        		dFactor, dOffset, variance);

}
