#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h> 
#define  DevAddr  0x68  //device address
#define  X_OFFSET 35
#define  Y_OFFSET -5
#define  Z_OFFSET -6
struct gyro_dat
{
	short row_t;
	short row_x;
	short row_y;
	short row_z;
	double x;
	double y;
	double z;
	double t;
};

void itg3205_init(int fd)
{
//http://wenku.baidu.com/link?url=eROpivZPWepLYTGR70mWz4MxKXh2TRSOfH0aEklzYsySi-ymLmSYl0EYh5_y5tD7ZSCSSBfc8EgeI-lcHluio71Fx_WgP8y0Sl0kz7hTtf_
	wiringPiI2CWriteReg8(fd, 0x16, 0x18); //Gyro Full-Scale Range(±2000°/sec)
	wiringPiI2CWriteReg8(fd, 0x15, 0x07); //Sampling rate 125Hz 
}
struct gyro_dat itg3205_read_xyz(int fd)
{
	char t0, x0, y0, z0, t1, x1, y1, z1;
	struct gyro_dat gyro_xyz;
	t0 = wiringPiI2CReadReg8(fd, 0x1B);
	t1 = wiringPiI2CReadReg8(fd, 0x1C);
	x1 = wiringPiI2CReadReg8(fd, 0x1D);
	x0 = wiringPiI2CReadReg8(fd, 0x1E);
	y1 = wiringPiI2CReadReg8(fd, 0x1F);
	y0 = wiringPiI2CReadReg8(fd, 0x20);
	z1 = wiringPiI2CReadReg8(fd, 0x21);
	z0 = wiringPiI2CReadReg8(fd, 0x22);
	gyro_xyz.row_t = (short)(t1 << 8) + (short)t0;
	gyro_xyz.row_x = (short)(x1 << 8) + (short)x0 + X_OFFSET;
	gyro_xyz.row_y = (short)(y1 << 8) + (short)y0 + Y_OFFSET;
	gyro_xyz.row_z = (short)(z1 << 8) + (short)z0 + Z_OFFSET;
	gyro_xyz.x = gyro_xyz.row_x / 14.375;
	gyro_xyz.y = gyro_xyz.row_y / 14.375;
	gyro_xyz.z = gyro_xyz.row_z / 14.375;
	gyro_xyz.t = gyro_xyz.row_t / 280.0;
	return gyro_xyz;
}

int main(void)
{
	int fd;
	struct gyro_dat gyro_xyz;
	fd = wiringPiI2CSetup(DevAddr);
	if (-1 == fd){
		perror("I2C device setup error");
	}

	itg3205_init(fd);
	float fx, fy, fz;
	while (1){
		gyro_xyz = itg3205_read_xyz(fd);
		printf("x: %f	y: %f	z: %f	row_x: %d	row_y: %d	row_z: %d\n",
			 gyro_xyz.x, gyro_xyz.y, gyro_xyz.z,/* gyro_xyz.t, */
			 gyro_xyz.row_x, gyro_xyz.row_y, gyro_xyz.row_z/*, gyro_xyz.row_t*/);
		sleep(1);
	}
	return 0;
}


