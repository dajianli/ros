#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h> 
#define  DevAddr  0x53  //device address
struct acc_dat
{
	int x;
	int y;
	int z;
};

void adxl345_init(int fd)
{
	wiringPiI2CWriteReg8(fd, 0x31, 0x0b); //FULL_RES(4mg/LSB) signed int ±16g   
	wiringPiI2CWriteReg8(fd, 0x2d, 0x08);
	// wiringPiI2CWriteReg8(fd, 0x2e, 0x00);  
	wiringPiI2CWriteReg8(fd, 0x1e, 0x00);
	wiringPiI2CWriteReg8(fd, 0x1f, 0x00);
	wiringPiI2CWriteReg8(fd, 0x20, 0x00);
	wiringPiI2CWriteReg8(fd, 0x21, 0x00);
	wiringPiI2CWriteReg8(fd, 0x22, 0x00);
	wiringPiI2CWriteReg8(fd, 0x23, 0x00);
	wiringPiI2CWriteReg8(fd, 0x24, 0x01);
	wiringPiI2CWriteReg8(fd, 0x25, 0x0f);
	wiringPiI2CWriteReg8(fd, 0x26, 0x2b);
	wiringPiI2CWriteReg8(fd, 0x27, 0x00);
	wiringPiI2CWriteReg8(fd, 0x28, 0x09);
	wiringPiI2CWriteReg8(fd, 0x29, 0xff);
	wiringPiI2CWriteReg8(fd, 0x2a, 0x80);
	wiringPiI2CWriteReg8(fd, 0x2c, 0x0a);
	wiringPiI2CWriteReg8(fd, 0x2f, 0x00);
	wiringPiI2CWriteReg8(fd, 0x38, 0x9f);
}
struct acc_dat adxl345_read_xyz(int fd)
{
	char x0, y0, z0, x1, y1, z1;
	struct acc_dat acc_xyz;
	x0 = wiringPiI2CReadReg8(fd, 0x32);
	x1 = wiringPiI2CReadReg8(fd, 0x33);
	y0 = wiringPiI2CReadReg8(fd, 0x34);
	y1 = wiringPiI2CReadReg8(fd, 0x35);
	z0 = wiringPiI2CReadReg8(fd, 0x36);
	z1 = wiringPiI2CReadReg8(fd, 0x37);
	acc_xyz.x = (short)(x1 << 8) + (int)x0;
	acc_xyz.y = (short)(y1 << 8) + (int)y0;
	acc_xyz.z = (short)(z1 << 8) + (int)z0;
	return acc_xyz;
}

int main(void)
{
	int fd;
	struct acc_dat acc_xyz;
	fd = wiringPiI2CSetup(DevAddr);
	if (-1 == fd){
		perror("I2C device setup error");
	}

	adxl345_init(fd);
	float fx, fy, fz;
	while (1){
		acc_xyz = adxl345_read_xyz(fd);
		fx = (float)acc_xyz.x / 250.0;
		fy = (float)acc_xyz.y / 250.0;
		fz = (float)acc_xyz.z / 250.0;
		//printf("x: %05d  y: %05d  z: %05d\n", acc_xyz.x, acc_xyz.y, acc_xyz.z);
		printf("x: %f   y: %f   z: %f\n", fx, fy, fz);
		sleep(1);
	}
	return 0;
}


