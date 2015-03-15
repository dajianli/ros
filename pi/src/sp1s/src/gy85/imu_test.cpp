#include "imu.h"
#include <unistd.h>


int main(void)
{
	IMU imu;
	imu.set_compass_offsets(750, -900, -200);
	imu.set_gyro_offsets(35, -5, -6);
	double pitch, roll, yaw;
	while(true)
	{
		
		imu.read_pitch_roll_yaw(pitch, roll, yaw);
		printf(" pitch: %.5f 	row: %.5f 	yaw: %.5f\n",
			pitch, roll, yaw);
		sleep(0.1);
	}
}
