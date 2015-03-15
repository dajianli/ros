#!/usr/bin/env python

import web  # web.py
import smbus

import bitify.python.sensors.imu as imu
from bitify.python.utils.i2cutils import i2c_raspberry_pi_bus_number

urls = (
    '/', 'index'
)

bus = smbus.SMBus(i2c_raspberry_pi_bus_number())
imu_controller = imu.IMU(bus, 0x68, 0x53, 0x1e, "IMU")

imu_controller.set_compass_offsets(750, -900, -200)

app = web.application(urls, globals())

class index:
    def GET(self):
        (pitch, roll, yaw) = imu_controller.read_pitch_roll_yaw()
        result = "%.6f %.6f %.6f" % (pitch, roll, yaw)
        return result

if __name__ == "__main__":
    app.run()
