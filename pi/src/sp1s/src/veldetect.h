#pragma once

#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>

#define PI 		3.1415926
#define DIAMETER 	0.068
#define HOLES_ON_WHEEL  20


class CVeldetect
{
public:
	void init(int leftPin, int rightPin)
	{
		wiringPiSetup();
		pinMode(leftPin, INPUT );		
		pinMode(rightPin, INPUT );
  		m_last_time = ros::Time::now();  
		m_leftCounter = 0;
		m_rightCounter = 0;

		// set Pin to generate an interrupt on high-to-low transitions
		// and attach xxxInterrupt() to the interrupt
		if ( wiringPiISR (leftPin, INT_EDGE_FALLING, &left_Interrupt) < 0 ||
		     wiringPiISR (rightPin, INT_EDGE_FALLING, &right_Interrupt) < 0) {
		       fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
		       return;
		}
	}
        
        void Reset()
	{
		m_leftCounter = 0;
		m_rightCounter = 0;
		m_last_time = ros::Time::now();
	}

	// compute the linear velocity in unit 'm/s'
	float computeLeftVel()
	{
        	return computeVel(m_leftCounter);
	}
	
	float computeRightVel()
	{
        	return computeVel(m_rightCounter);
	}
        
private:

	void static left_Interrupt(void) {
		m_leftCounter++;
	}
	void static right_Interrupt(void) {
		m_rightCounter++;
	}

	// compute the linear velocity in unit 'm/s'
	float computeVel(int counter)
	{
            m_current_time = ros::Time::now();
	    double distance = counter * PI * DIAMETER / HOLES_ON_WHEEL;
	    double secs = (m_current_time - m_last_time).toSec();	    
	    return distance / secs;
	}
	// the event counter 
	static volatile int m_leftCounter;
	static volatile int m_rightCounter;
	ros::Time m_current_time;
	ros::Time m_last_time;
};

volatile int CVeldetect::m_leftCounter = 0;
volatile int CVeldetect::m_rightCounter = 0;
