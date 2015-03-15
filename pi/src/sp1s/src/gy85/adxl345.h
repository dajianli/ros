#pragma once
#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h> 
#include <math.h> 
#define  ADXL345_DevAddr  0x53  //device address
#define  POWER_CTL		0x2d
#define  DATA_FORMAT	0x31
#define  FIFO_CTL		0x38

#define  AFS_2g		 0
#define  AFS_4g		 1
#define  AFS_8g		 2
#define  AFS_16g		 3

class adxl345
{
private:
	struct acc_dat
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
	adxl345(short x_offset = 0, short y_offset = 0, short z_offset = 0)
	{
		m_fd = wiringPiI2CSetup(ADXL345_DevAddr);
		if (-1 == m_fd)
			return;

		// Set FULL_RES(4mg/LSB) signed int ±16g   
		wiringPiI2CWriteReg8(m_fd, DATA_FORMAT, 0x0b);

		// Wake up the device
		wiringPiI2CWriteReg8(m_fd, POWER_CTL, 0x08);

		// Disable FIFO mode
		wiringPiI2CWriteReg8(m_fd, FIFO_CTL, 0b00000000);

		m_x_offset = x_offset;
		m_y_offset = y_offset;
		m_z_offset = z_offset;
		read_raw_data();
	}
	void read_raw_data()
	{
		char x0, y0, z0, x1, y1, z1;
		x0 = wiringPiI2CReadReg8(m_fd, 0x32);
		x1 = wiringPiI2CReadReg8(m_fd, 0x33);
		y0 = wiringPiI2CReadReg8(m_fd, 0x34);
		y1 = wiringPiI2CReadReg8(m_fd, 0x35);
		z0 = wiringPiI2CReadReg8(m_fd, 0x36);
		z1 = wiringPiI2CReadReg8(m_fd, 0x37);
		m_dat.raw_x = (short)(x1 << 8) + (int)x0 + m_x_offset;
		m_dat.raw_y = (short)(y1 << 8) + (int)y0 + m_y_offset;
		m_dat.raw_z = (short)(z1 << 8) + (int)z0 + m_z_offset;
		m_dat.scaled_x = m_dat.raw_x * 0.004;
		m_dat.scaled_y = m_dat.raw_y * 0.004;
		m_dat.scaled_z = m_dat.raw_z * 0.004;
	}

	double distance(double x, double y)
	{
		//Returns the distance between two point in 2d space//
		return sqrt((x * x) + (y * y));
	}

	double read_x_rotation(double x, double y, double z) {
		//Returns the rotation around the X axis in radians//
		if(z>=0)
			return (atan2(y, distance(x, z)));
		else
			return (atan2(y, -distance(x, z)));
	}

	double read_y_rotation(double x, double y, double z) {
		//Returns the rotation around the Y axis in radians//
		if(z>=0)
			return (-atan2(x, distance(y, z)));
		else
			return (-atan2(x, -distance(y, z)));
	}

	short read_raw_accel_x() {
		//Return the RAW X accelerometer value//
		return m_dat.raw_x;
	}

	short read_raw_accel_y() {
		//Return the RAW Y accelerometer value//
		return  m_dat.raw_y;
	}

	short read_raw_accel_z() {
		//Return the RAW Z accelerometer value//
		return  m_dat.raw_z;
	}

	double read_scaled_accel_x() {
		//Return the SCALED X accelerometer value//
		return  m_dat.scaled_x;
	}

	double read_scaled_accel_y() {
		//Return the SCALED Y accelerometer value//
		return  m_dat.scaled_y;
	}

	double read_scaled_accel_z() {
		//Return the SCALED Z accelerometer value//
		return  m_dat.scaled_z;
	}


	double read_pitch() {
		//Calculate the current pitch value in radians//
		double x = read_scaled_accel_x();
		double y = read_scaled_accel_y();
		double z = read_scaled_accel_z();
		return read_y_rotation(x, y, z);
	}

	double read_roll() {
		//Calculate the current roll value in radians//
		double x = read_scaled_accel_x();
		double y = read_scaled_accel_y();
		double z = read_scaled_accel_z();
		return read_x_rotation(x, y, z);
	}

};



