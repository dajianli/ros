#pragma once
#include <wiringPi.h>
#include "veldetect.h"
#include "wheelpwm.h"
#include "dbl_cmp.h"

class CStartPwm
{
public:
	CStartPwm(CWheelpwm * leftWheel, CWheelpwm * rightWheel,
		CVeldetect * pLinearVel)
	{
		m_leftWheel = leftWheel;
		m_rightWheel = rightWheel;
		m_pLinearVel = pLinearVel;
	}
	
	void TestStartPwm(int &leftPwm, int &rightPwm)
	{
		leftPwm = 5;
		rightPwm = 5;
		m_pLinearVel->Reset();
		double leftDist = 0, rightDist = 0;
/*
		while(DBLCMPEQ(leftDist, 0))
		{			
			leftPwm++;			
			m_leftWheel->forward(leftPwm);
			delay(500);
			leftDist = m_pLinearVel->distanceLeft();
		}
		m_leftWheel->forward(0);
		m_pLinearVel->Reset();
		while(DBLCMPEQ(rightDist, 0))
		{			
			rightPwm++;			
			m_rightWheel->forward(rightPwm);
			delay(500);
			rightDist = m_pLinearVel->distanceRight();
		}
		m_rightWheel->forward(0);*/
		
		while(DBLCMPLT(leftDist, 0.05)
			|| DBLCMPLT(rightDist, 0.05) )
		{
			if(DBLCMPLT(leftDist, 0.05))
				leftPwm++;
			if(DBLCMPLT(rightDist, 0.05))
				rightPwm++;
			m_leftWheel->forward(leftPwm);
			m_rightWheel->forward(rightPwm);
			delay(500);
			leftDist = m_pLinearVel->distanceLeft();
			rightDist = m_pLinearVel->distanceRight();
		}
		m_leftWheel->forward(0);
		m_rightWheel->forward(0);
		m_pLinearVel->Reset();
		delay(500);
	}

private:
	CWheelpwm * m_leftWheel;
	CWheelpwm * m_rightWheel;
	CVeldetect * m_pLinearVel;
};
