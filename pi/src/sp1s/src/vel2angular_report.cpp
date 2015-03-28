#include "ros/ros.h"
#include "std_msgs/String.h"
#include <wiringPi.h>
#include <sstream>
#include "veldetect.h"
#include "wheelpwm.h"
#include "gy85/hmc5883l.h"
#include "AngularVelocity.h"
#include "startpwm.h"
#include "compass_calibrate.h"

#define STARTUP_PWM 4

struct VelocityPair{
	double linear_left;
	double linear_right;
	double angular;
};

void Sample_Linear_Angular_Velocity(
	CWheelpwm * leftWheel, CWheelpwm * rightWheel,
	CVeldetect * pLinearVel, hmc5883l * pCompass,
	VelocityPair * pSample)
{
	AngularVelocity angularMeasure;

	ros::Rate loop_rate(100);
	loop_rate.sleep();
	double heading = pCompass->read_bearing();
	angularMeasure.Start(heading);
	pLinearVel->Reset();
	while (angularMeasure.AngleRotated() < (M_PI * 2) && ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
		heading = pCompass->read_bearing();
		if(angularMeasure.Rotate(heading))
			printf("heading:%f  rotated:%f   \n", heading, angularMeasure.AngleRotated());
	}

	pSample->angular = angularMeasure.AngularVelocityReading();
	pSample->linear_right = pLinearVel->computeRightVel();
	pSample->linear_left = pLinearVel->computeLeftVel();
	if(!leftWheel->IsTurningForward())
		pSample->linear_left *= -1;
	if(!rightWheel->IsTurningForward())
		pSample->linear_right *= -1;
	printf("------------------------------------------------------------------------------------------\n");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel2angular_reporter");
	ros::NodeHandle n;
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

    hmc5883l compass;
    compass.setOffset(-231, -666, -49854);
    /*CCompassCalibrate compassCalibrate(&left_pwm, &right_pwm, &veldetect);
    compassCalibrate.Calibrate(&compass);
    printf("compass_x_offset:%d		compass_y_offset:%d		compass_z_offset:%d	\n",
    		compassCalibrate.m_x_offset,compassCalibrate.m_y_offset,compassCalibrate.m_z_offset);
*/
	VelocityPair sample;
	VelocityPair samples[10][10];
	int iRound = -1;
	int iStepPwm = (10 - STARTUP_PWM );

	
    // Figure out the minimal PWM to start run
    CStartPwm startPwm(&left_pwm, &right_pwm, &veldetect);
    int iStartPwmLeft, iStartPwmRight;
    startPwm.TestStartPwm(iStartPwmLeft, iStartPwmRight);
    iStartPwmLeft += 8;
    iStartPwmRight += 8;
    int iStepPwmLeft = ( 100 - iStartPwmLeft ) / 9;
    int iStepPwmRight = ( 100 - iStartPwmRight ) / 9;
	/*
    // Turn forward around left wheel	
	iRound++;
	for (int i = 0; i < 10; i++)
	{
		left_pwm.forward(0);
		right_pwm.forward(iStartPwmRight + i*iStepPwmRight);
		Sample_Linear_Angular_Velocity(&left_pwm,  &right_pwm,
			&veldetect, &compass, &sample);
		samples[iRound][i] = sample;
	}

	// Turn backward around left wheel
	iRound++;
	for (int i = 0; i < 10; i++)
	{
		left_pwm.forward(0);
		right_pwm.backward(iStartPwmRight + i*iStepPwmRight);
		Sample_Linear_Angular_Velocity(&left_pwm,  &right_pwm,
			&veldetect, &compass, &sample);
		samples[iRound][i] = sample;
	}

	// Turn forward around rightwheel
	iRound++;
	for (int i = 0; i < 10; i++)
	{
		left_pwm.forward(iStartPwmLeft + i * iStepPwmLeft);
		right_pwm.forward(0);
		Sample_Linear_Angular_Velocity(&left_pwm,  &right_pwm,
			&veldetect, &compass, &sample);
		samples[iRound][i] = sample;
	}

	// Turn backward around right wheel
	iRound++;
	for (int i = 0; i < 10; i++)
	{
		left_pwm.backward(iStartPwmLeft + i * iStepPwmLeft);
		right_pwm.forward(0);
		Sample_Linear_Angular_Velocity(&left_pwm,  &right_pwm,
			&veldetect, &compass, &sample);
		samples[iRound][i] = sample;
	}*/

	double lower_co = 0;
	double higher_co = 4;
	int higher_start = 100 - 9 * higher_co;	
	iStartPwmLeft -= 8;
    	iStartPwmRight -= 8;

	// Turn left forward
	iRound++;
	left_pwm.forward(0);
	right_pwm.forward(0);
	sleep(10);
	for (int i = 0; i < 10; i++)
	{
		left_pwm.forward(iStartPwmLeft + i * lower_co);
		right_pwm.forward(higher_start + i * higher_co);
		Sample_Linear_Angular_Velocity(&left_pwm,  &right_pwm,
			&veldetect, &compass, &sample);
		samples[iRound][i] = sample;
	}

	// Turn left backward
	iRound++;
	left_pwm.forward(0);
	right_pwm.forward(0);
	sleep(10);
	for (int i = 0; i < 10; i++)
	{
		left_pwm.backward(iStartPwmLeft + i * lower_co);
		right_pwm.backward(higher_start + i * higher_co);
		Sample_Linear_Angular_Velocity(&left_pwm,  &right_pwm,
			&veldetect, &compass, &sample);
		samples[iRound][i] = sample;
	}

	// Turn right forward
	iRound++;
	left_pwm.forward(0);
	right_pwm.forward(0);
	sleep(10);
	for (int i = 0; i < 10; i++)
	{
		left_pwm.forward(higher_start + i * higher_co);
		right_pwm.forward(iStartPwmRight + i * lower_co);
		Sample_Linear_Angular_Velocity(&left_pwm,  &right_pwm,
			&veldetect, &compass, &sample);
		samples[iRound][i] = sample;
	}

	// Turn right backward
	iRound++;
	left_pwm.forward(0);
	right_pwm.forward(0);
	sleep(10);
	for (int i = 0; i < 10; i++)
	{
		left_pwm.backward(higher_start + i * higher_co);
		right_pwm.backward(iStartPwmRight + i * lower_co);
		Sample_Linear_Angular_Velocity(&left_pwm,  &right_pwm,
			&veldetect, &compass, &sample);
		samples[iRound][i] = sample;
	}
	/*
	iStartPwmLeft += 8;
    	iStartPwmRight += 8;
	// Self rotate clock wise
	iRound++;
	left_pwm.forward(0);
	right_pwm.forward(0);
	sleep(10);
	for(int i = 0; i < 10; i++)
	{
		left_pwm.forward(iStartPwmLeft + i * 5);
		right_pwm.backward(iStartPwmRight + i * 3);
		Sample_Linear_Angular_Velocity(&left_pwm,  &right_pwm,
			&veldetect, &compass, &sample);
		samples[iRound][i] = sample;
	}

	// Self rotate counter clock wise
	iRound++;
	left_pwm.forward(0);
	right_pwm.forward(0);
	sleep(10);
	for(int i = 0; i < 10; i++)
	{
		left_pwm.backward(iStartPwmLeft + i * 3);
		right_pwm.forward(iStartPwmRight + i * 5);
		Sample_Linear_Angular_Velocity(&left_pwm,  &right_pwm,
			&veldetect, &compass, &sample);
		samples[iRound][i] = sample;
	}
	*/
	left_pwm.forward(0);
	right_pwm.forward(0);

	for (int row = 0; row < 10; row++)
	{
		for (int col = 0; col <= iRound; col++)
		{
			sample = samples[col][row];
		 	printf("%f  %f   ", fabs(sample.linear_left - sample.linear_right), sample.angular);
		}
		printf("\n");
	}
}
