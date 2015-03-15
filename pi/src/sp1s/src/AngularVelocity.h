#pragma once

#include <time.h>
#ifndef WIN32
#include <sys/time.h> 
#else
#include <windows.h>
#define _USE_MATH_DEFINES
#undef _MATH_DEFINES_DEFINED
#endif
#include <math.h>
#include "dbl_cmp.h"



/// accept heading value range [0, 2*PI]
class AngularVelocity
{
public:
	AngularVelocity(){
		m_dStartHeading = 0.0;
		m_dStartTime = 0.0;
		m_dAccumulateOffset = 0.0;
		m_dLastHeading = 0.0;
	}

	void Start(double heading)
	{
		m_dStartHeading = heading;
		m_dLastHeading = heading;
		m_dStartTime = GetTimeNow();
		m_dAccumulateOffset = 0.0;
	}

	bool Rotate(double heading)
	{
		double diff_abs = fabs(heading - m_dLastHeading);

		// Rotating corss border of 0 and 2*PI
		if (diff_abs > M_PI * 1.5)
		{
			// rotating from 0 to 2*PI
			if (heading > M_PI * 1.5)
				m_dAccumulateOffset -= 2 * M_PI;
			else
				m_dAccumulateOffset += 2 * M_PI;
		}
		else if(diff_abs > 1.5)
		{
			return false;
		}
		m_dLastHeading = heading;
		return true;
	}

	double AngularVelocityReading()
	{
		double diff_abs = AngleRotated();
		double now = GetTimeNow();
		double diff_time = now - m_dStartTime;
		if (DBLCMPEQ(diff_time, 0))
			return 0;
		return diff_abs / diff_time;
	}

	double AngleRotated()
	{
		double diff_abs = fabs(m_dStartHeading - LastHeading());
		return diff_abs;
	}
private:
	double GetTimeNow()
	{
#ifndef WIN32
		struct timeval dwNow;
		gettimeofday(&dwNow, NULL);
		return (double)dwNow.tv_sec + (double)dwNow.tv_usec / 1000000;
#else
		time_t clock;
		struct tm tm;
		SYSTEMTIME wtm;

		GetLocalTime(&wtm);
		tm.tm_year = wtm.wYear - 1900;
		tm.tm_mon = wtm.wMonth - 1;
		tm.tm_mday = wtm.wDay;
		tm.tm_hour = wtm.wHour;
		tm.tm_min = wtm.wMinute;
		tm.tm_sec = wtm.wSecond;
		tm.tm_isdst = -1;
		clock = mktime(&tm);
		return (double)clock + wtm.wMilliseconds / 1000;
#endif
	}
	double LastHeading()
	{
		return m_dLastHeading + m_dAccumulateOffset;
	}
	double m_dStartTime;
	double m_dStartHeading;
	double m_dLastHeading;
	double m_dAccumulateOffset;
};
