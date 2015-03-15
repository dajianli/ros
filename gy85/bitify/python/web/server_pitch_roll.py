
#!/usr/bin/python
import web
import smbus
import math
import time
from bitify.python.sensors.adxl345 import ADXL345
from bitify.python.sensors.itg3205 import ITG3205

urls = (
    '/', 'index'
)

bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards
adxl345 = 0x53       # This is the address value read via the i2cdetect command
itg3205 = 0x68

accelerometer = ADXL345(bus, adxl345, "IMU-accelerometer", ADXL345.AFS_16g)
gyro = ITG3205(bus, itg3205, "gyro")

last_x = 0.0
last_y = 0.0

last_time = time.time()
first_time = True

'''
def read_byte(address, adr):
    return bus.read_byte_data(address, adr)

def read_word(address, adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(address, adr):
    val = read_word(address, adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val
'''
def dist(a,b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -radians

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return radians


class index:
    def __init__(self):
	accelerometer.read_raw_data() #read once to enable the configured setting

    def GET(self):
	accelerometer.read_raw_data()
	accel_xout_scaled = accelerometer.read_scaled_accel_x()
        accel_yout_scaled = accelerometer.read_scaled_accel_y()
        accel_zout_scaled = accelerometer.read_scaled_accel_z()
	rotation_x = get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
	rotation_y = get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)

	global last_time
	global last_x
	global last_y
	global first_time

	now = time.time()
        time_diff = now - last_time
	last_time = now

	if(first_time):
	  first_time = False
	  last_x = rotation_x
	  last_y = rotation_y
	  print(' first_time ')   
	else:
	  gyro.read_raw_data()
	  gyro_xout_scaled = gyro.read_scaled_gyro_x()
	  gyro_yout_scaled = gyro.read_scaled_gyro_y()
	  gyro_zout_scaled = gyro.read_scaled_gyro_z()
	  gyro_x_delta = (gyro_xout_scaled * time_diff) 
    	  gyro_y_delta = (gyro_yout_scaled * time_diff)
	  K = 0.9
	  K1 = 1 - K
	  last_x = K * (last_x + gyro_x_delta) + (K1 * rotation_x)
    	  last_y = K * (last_y + gyro_y_delta) + (K1 * rotation_y)

        return str(last_x)+" "+str(last_y)

if __name__ == "__main__":

    app = web.application(urls, globals())
    app.run()

