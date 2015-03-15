#pragma once
#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <math.h>

#define  HMC5883L_DevAddr  0x1e  //device address
#define  CONF_REG_A   0
#define  CONF_REG_B   1
#define  MODE_REG     2
#define  DATA_XOUT_H   3
#define  DATA_XOUT_L   4
#define  DATA_ZOUT_H   5
#define  DATA_ZOUT_L   6
#define  DATA_YOUT_H   7
#define  DATA_YOUT_L   8
#define  TWO_PI M_PI*2


class hmc5883l
{
private:
	struct compass_dat
	{
		short row_x;
		short row_y;
		short row_z;
		double scaled_x;
		double scaled_y;
		double scaled_z;
	} m_data;

	int m_fd;
	short m_x_offset;
	short m_y_offset;
	short m_z_offset;
	double m_gain_scale;

public:
	hmc5883l(short x_offset=0, short y_offset=0, short z_offset=0)
	{
	//Data sheet: http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/HMC5883L_3-Axis_Digital_Compass_IC.pdf
		
		m_fd = wiringPiI2CSetup(HMC5883L_DevAddr);
		m_x_offset = x_offset;
		m_y_offset = y_offset;
		m_z_offset = z_offset;
		
		// Set the number of samples
		char samples = 3;
		char rate = 6;
		char conf_a = (samples << 5) + (rate << 2);
		wiringPiI2CWriteReg8(m_fd, CONF_REG_A, conf_a);
		
		// Set the gain
		char gain = 1;
		m_gain_scale = 0.92;
		char conf_b = gain << 5;
		wiringPiI2CWriteReg8(m_fd, CONF_REG_B, conf_b);

		// Set the operation mode - CONTINUOUS
		wiringPiI2CWriteReg8(m_fd, MODE_REG, 0);

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
 		x1 = wiringPiI2CReadReg8(m_fd, DATA_XOUT_H);
		x0 = wiringPiI2CReadReg8(m_fd, DATA_XOUT_L);
		y1 = wiringPiI2CReadReg8(m_fd, DATA_YOUT_H);
		y0 = wiringPiI2CReadReg8(m_fd, DATA_YOUT_L);
		z1 = wiringPiI2CReadReg8(m_fd, DATA_ZOUT_H);
		z0 = wiringPiI2CReadReg8(m_fd, DATA_ZOUT_L);
		m_data.row_x = (short)(x1 << 8) + (short)x0 + m_x_offset;
		m_data.row_y = (short)(y1 << 8) + (short)y0 + m_y_offset;
		m_data.row_z = (short)(z1 << 8) + (short)z0 + m_z_offset;
		m_data.scaled_x = m_data.row_x * m_gain_scale;
		m_data.scaled_y = m_data.row_y * m_gain_scale;
		m_data.scaled_z = m_data.row_z * m_gain_scale;
	}
	
	double read_bearing()
        {
        	//Read a bearing from the sensor assuming the sensor is level         
        	read_raw_data();

       	 	double bearing = atan2(read_scaled_y(), read_scaled_x());
		if ( bearing < 0 )
		    return bearing + (TWO_PI);
		else
		    return bearing;
	}


	// pitch roll : radian angle
        double read_compensated_bearing(double pitch, double roll) 
        {
        	//Calculate a bearing taking in to account the current pitch and roll of the device as supplied as parameters
        	
        	read_raw_data();
        	double cos_pitch = cos(pitch);
        	double sin_pitch = sin(pitch);
        
        	double cos_roll = cos(roll);
        	double sin_roll = sin(roll);
    
        	double Xh = (m_data.scaled_x * cos_pitch) + (m_data.scaled_z * sin_pitch);
        	double Yh = (m_data.scaled_x * sin_pitch * sin_roll) + (m_data.scaled_y * cos_roll) - (m_data.scaled_z * sin_roll * cos_pitch);
        
        	double bearing = atan2(Yh, Xh);
		//if(Xh < 0 )
		//    bearing += M_PI/2;
        	if ( bearing < 0 )
		    return bearing + (TWO_PI);
		else
		    return bearing;
	}    
    
	short read_raw_x()
	{
        	return m_data.row_x;
	}
    
    	short read_raw_y()
	{
        	return m_data.row_y;
	}
    
        short read_raw_z()
	{
        	return m_data.row_z;
	}

    	double read_scaled_x()
	{
        	return m_data.scaled_x;
	}

        double read_scaled_y()
	{
        	return m_data.scaled_y;
	}

        double read_scaled_z()
	{
        	return m_data.scaled_z;
	}
};



