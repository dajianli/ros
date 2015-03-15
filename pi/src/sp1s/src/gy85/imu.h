#pragma once
#include "hmc5883l.h"
#include "adxl345.h"
#include "itg3205.h"
#include <sys/time.h>

#define K 0.9
#define K1 (1-K)

class IMU
{
private:
	hmc5883l compass;
	adxl345 accelerometer;
	itg3205 gyroscope;
	
	double _pitch;
	double _roll;
	double _yaw;
	double _last_time;

	void comp_filter( double time_diff )
	{		
		_pitch = K * (_pitch + gyroscope.read_scaled_gyro_y() * time_diff) + (K1 * accelerometer.read_pitch());
        	_roll = K * (_roll + gyroscope.read_scaled_gyro_x() * time_diff) + (K1 * accelerometer.read_roll());
	}
	
	void read_all()
	{
		gyroscope.read_raw_data();
		accelerometer.read_raw_data();		
	}

public:
	IMU()
	{
		compass.setOffset(769, -779, -277);
		_pitch = 0.0;
		_roll = 0.0;
		_yaw = 0.0;
		
		// take a reading from the device to allow it to settle after config changes
        	read_all();
		// now take another to act a starting value
        	read_all();
		
		_pitch = accelerometer.read_pitch();
		_roll = accelerometer.read_roll();

		struct timeval  tv;		
		gettimeofday(&tv, NULL);
		_last_time = (double)(tv.tv_sec) + (double)(tv.tv_usec) / 1000000 ; 
		
	}

	void read_pitch_roll_yaw(double &pitch, double &roll, double &yaw)
	{
		read_all();
		struct timeval  tv;		
		gettimeofday(&tv, NULL);
		double now = (double)(tv.tv_sec) + (double)(tv.tv_usec) / 1000000 ; 
        	double time_diff = now - _last_time;
        	_last_time = now;
		comp_filter(time_diff);
		
		gettimeofday(&tv, NULL);


		_yaw = compass.read_compensated_bearing(_pitch, _roll);
	
		pitch = _pitch;
		roll = _roll;
		yaw = _yaw;
	}

	void set_compass_offsets(short x_offset, short y_offset, short z_offset)
	{
        	compass.setOffset(x_offset, y_offset, z_offset);
	}

	void set_gyro_offsets(short x_offset, short y_offset, short z_offset)
	{
		gyroscope.setOffset(x_offset, y_offset, z_offset);
	}
};

