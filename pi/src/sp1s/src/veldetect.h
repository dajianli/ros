#pragma once

#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>
#include <math.h>

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
		m_disPerHole = DIAMETER * M_PI / HOLES_ON_WHEEL;
	}
        
        void Reset()
	{
		m_leftCounter = 0;
		m_rightCounter = 0;
		m_last_time = ros::Time::now();
	}

	// compute the linear velocity in unit 'm/s'
	double computeLeftVel()
	{
        	return computeVel(m_leftCounter);
	}
	
	double computeRightVel()
	{
        	return computeVel(m_rightCounter);
	}

	double distanceLeft()
	{
		return computeDistance(m_leftCounter);
	}

	double distanceRight()
	{
		return computeDistance(m_rightCounter);
	}
        
private:

	void static left_Interrupt(void) {
		m_leftCounter++;
	}
	void static right_Interrupt(void) {
		m_rightCounter++;
	}

	// compute the linear velocity in unit 'm/s'
	double computeVel(int counter)
	{
            m_current_time = ros::Time::now();
	    double distance = computeDistance(counter);
	    double secs = (m_current_time - m_last_time).toSec();	    
	    return distance / secs;
	}

	double computeDistance(int counter)
	{
		// div by 2 caused by the hardware issue, 
		// both falling&rising would trigger the counter to increase
		return (double)counter * m_disPerHole / 2;
	}
	// the event counter 
	static volatile int m_leftCounter;
	static volatile int m_rightCounter;
	double m_disPerHole; 
	ros::Time m_current_time;
	ros::Time m_last_time;
};

volatile int CVeldetect::m_leftCounter = 0;
volatile int CVeldetect::m_rightCounter = 0;
