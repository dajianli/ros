#pragma once
#include "wheelpwm.h"
#include "gy85/hmc5883l.h"
#include "startpwm.h"
#include <sys/time.h>

class CCompassCalibrate
{
public:
	CCompassCalibrate(CWheelpwm * leftWheel, CWheelpwm * rightWheel,
		CVeldetect * pLinearVel)
	{
		m_leftWheel = leftWheel;
		m_rightWheel = rightWheel;
		m_pLinearVel = pLinearVel;
		m_x_offset = 0;
		m_y_offset = 0;
		m_z_offset = 0;
	}
	
	void Calibrate(hmc5883l * compass)
	{		
		CStartPwm startPwm(m_leftWheel, m_rightWheel, m_pLinearVel);
		int iStartPwmLeft, iStartPwmRight;
    		startPwm.TestStartPwm(iStartPwmLeft, iStartPwmRight);
		iStartPwmLeft += 8;
		iStartPwmRight += 8;
		int duration = 50 * 1000; // 50 secs
		int xMin, xMax, yMin, yMax, zMin, zMax;
		xMin = yMin = zMin = 100000;
		xMax = yMax = zMax = -100000;
		m_leftWheel->forward(iStartPwmLeft);
		m_rightWheel->backward(iStartPwmRight);
		compass->read_raw_data();
		delay(10);
		while(duration > 0)
		{
			compass->read_raw_data();
			int x = compass->read_raw_x();
			int y = compass->read_raw_y();
			int z = compass->read_raw_z();
			if(x > xMax )
				xMax = x;
			if(x < xMin )
				xMin = x;
			if(y > yMax )
				yMax = y;
			if(y < yMin )
				yMin = y;
			if(z > zMax )
				zMax = z;
			if(z < zMin )
				xMin = z;
			delay(10);
			duration -= 10;
		}
		m_x_offset = -(xMin + xMax)/2;
		m_y_offset = -(yMin + yMax)/2;
		m_z_offset = -(zMin + zMax)/2;
		compass->setOffset( m_x_offset, m_y_offset, m_z_offset );
		m_leftWheel->forward(0);
		m_rightWheel->forward(0);
		delay(500);
	}
	int m_x_offset;
	int m_y_offset;
	int m_z_offset;
private:
	CWheelpwm * m_leftWheel;
	CWheelpwm * m_rightWheel;
	CVeldetect * m_pLinearVel;
};
