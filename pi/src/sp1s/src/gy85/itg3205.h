#pragma once
#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <math.h> 
#define  ITG3205_DevAddr  0x68  //device address

class itg3205
{
private:
	struct gyro_dat
	{
		short raw_x;
		short raw_y;
		short raw_z;
		double scaled_x;
		double scaled_y;
		double scaled_z;
	} m_dat;
	int m_fd;
	short m_x_offset;
	short m_y_offset;
	short m_z_offset;

public:
	itg3205(short x_offset = 0, short y_offset = 0, short z_offset = 0)
	{
		//http://wenku.baidu.com/link?url=eROpivZPWepLYTGR70mWz4MxKXh2TRSOfH0aEklzYsySi-ymLmSYl0EYh5_y5tD7ZSCSSBfc8EgeI-lcHluio71Fx_WgP8y0Sl0kz7hTtf_
		m_fd = wiringPiI2CSetup(ITG3205_DevAddr);
		if (-1 == m_fd)
			return;
		wiringPiI2CWriteReg8(m_fd, 0x16, 0x18); //Gyro Full-Scale Range(±2000°/sec)
		wiringPiI2CWriteReg8(m_fd, 0x15, 0x07); //Sampling rate 125Hz 

		m_x_offset = x_offset;
		m_y_offset = y_offset;
		m_z_offset = z_offset;
	}
	
	void setOffset(short x_offset=0, short y_offset=0, short z_offset=0)
	{
		m_x_offset = x_offset;
		m_y_offset = y_offset;
		m_z_offset = z_offset;
	}

	void read_raw_data()
	{
		char x0, y0, z0, x1, y1, z1;
		x1 = wiringPiI2CReadReg8(m_fd, 0x1D);
		x0 = wiringPiI2CReadReg8(m_fd, 0x1E);
		y1 = wiringPiI2CReadReg8(m_fd, 0x1F);
		y0 = wiringPiI2CReadReg8(m_fd, 0x20);
		z1 = wiringPiI2CReadReg8(m_fd, 0x21);
		z0 = wiringPiI2CReadReg8(m_fd, 0x22);
		 
		m_dat.raw_x = (short)(x1 << 8) + (short)x0 + m_x_offset;
		m_dat.raw_y = (short)(y1 << 8) + (short)y0 + m_y_offset;
		m_dat.raw_z = (short)(z1 << 8) + (short)z0 + m_z_offset;
		m_dat.scaled_x = m_dat.raw_x / 14.375 * M_PI / 180;
		m_dat.scaled_y = m_dat.raw_y / 14.375 * M_PI / 180;
		m_dat.scaled_z = m_dat.raw_z / 14.375 * M_PI / 180;
	}

	short read_raw_gyro_x()
	{
		//Return the RAW X gyro value
		return m_dat.raw_x;
	}

	short read_raw_gyro_y()
	{
		//Return the RAW Y gyro value
		return m_dat.raw_y;
	}
	short read_raw_gyro_z()
	{
		//Return the RAW Z gyro value
		return m_dat.raw_z;
	}
	double read_scaled_gyro_x() 
	{
		//Return the SCALED X gyro value in radians/second
		return m_dat.scaled_x;
	}
	double read_scaled_gyro_y()
	{
		//Return the SCALED Y gyro value in radians/second
		return m_dat.scaled_y;
	}
	double read_scaled_gyro_z()
	{
		//Return the SCALED Z gyro value in radians/second
		return m_dat.scaled_z;
	}
};



