#include "hmc5883l.h"
#include "adxl345.h"
#include "itg3205.h"
#include <unistd.h>

int main(void)
{
	hmc5883l compass(769, -779, -277);
	adxl345 accelor;
	itg3205 gyro(35, -5, -6);;
	while(1){
/*
	printf("hmc588l bearing: %.2f x: row_x: %d row_y: %d row_z: %d\n",
			 compass.read_bearing() * 180 / M_PI,
			 compass.read_raw_x(), compass.read_raw_y(), compass.read_raw_z() );

	accelor.read_raw_data();
	printf("adxl345  ax: %.5f 	ay: %.5f 	az: %.5f\n",
		accelor.read_scaled_accel_x(),accelor.read_scaled_accel_y(),accelor.read_scaled_accel_z());
*/
	gyro.read_raw_data();
	printf("itg3205  gx: %d 	gy: %d 	gz: %d\n",
		gyro.read_raw_gyro_x(),gyro.read_raw_gyro_y(),gyro.read_raw_gyro_z());
	//printf("itg3205  gx: %.5f		gy: %.5f		gz: %.5f\n",
	//	gyro.read_scaled_gyro_x(),gyro.read_scaled_gyro_y(),gyro.read_scaled_gyro_z());
	
	sleep(0.1);
	}
	return 0;
}

