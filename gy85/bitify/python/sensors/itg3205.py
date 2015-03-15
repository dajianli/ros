import math
import smbus

import bitify.python.utils.i2cutils as I2CUtils

class ITG3205(object):
    '''
    Simple ITG3205 implementation
    Datasheet: http://wenku.baidu.com/link?url=eROpivZPWepLYTGR70mWz4MxKXh2TRSOfH0aEklzYsySi-ymLmSYl0EYh5_y5tD7ZSCSSBfc8EgeI-lcHluio71Fx_WgP8y0Sl0kz7hTtf_
    '''

    CTRL_DLPF = 0x16
    CTRL_SAMPLING_RATE_DIV = 0x15   

    GYRO_START_BLOCK = 0x1D
    GYRO_XOUT_H = 0x1D
    GYRO_XOUT_L = 0x1E
    GYRO_YOUT_H = 0x1F
    GYRO_YOUT_L = 0x20
    GYRO_ZOUT_H = 0x21
    GYRO_ZOUT_L = 0x22
	
    X_OFFSET = 35
    Y_OFFSET = -5
    Z_OFFSET = -6
    
    GYRO_SCALE = 1/14.375 #LSB/(14.375 deg/s)
    
    def __init__(self, bus, address, name):
        '''
        Constructor
        '''

        self.bus = bus
        self.address = address
        self.name = name
        
        self.gyro_raw_x = 0
        self.gyro_raw_y = 0
        self.gyro_raw_z = 0
        
        self.gyro_scaled_x = 0
        self.gyro_scaled_y = 0
        self.gyro_scaled_z = 0
        
        self.raw_temp = 0
        self.scaled_temp = 0

	#Gyro Full-Scale Range(+ve -ve 2000 deg/sec)
	I2CUtils.i2c_write_byte(self.bus, self.address, ITG3205.CTRL_DLPF, 0x18);
	#Sampling rate 125Hz 
	I2CUtils.i2c_write_byte(self.bus, self.address, ITG3205.CTRL_SAMPLING_RATE_DIV, 0x07); 

        '''
        # Wake up the deice and get output for each of the three axes,X, Y & Z
        I2CUtils.i2c_write_byte(self.bus, self.address, L3G4200D.CTRL_REG1, 0b00001111)
        
        # Select Big endian so we can use existing I2C library and include the scaling
        ctrl_reg4 = 1 << 6 | L3G4200D.GYRO_SCALE[fs_scale][1] << 4
        I2CUtils.i2c_write_byte(self.bus, self.address, L3G4200D.CTRL_REG4, ctrl_reg4)
	'''

    def read_raw_data(self):
        
        self.gyro_raw_x = I2CUtils.i2c_read_word_signed(self.bus, self.address, ITG3205.GYRO_XOUT_H) + ITG3205.X_OFFSET
        self.gyro_raw_y = I2CUtils.i2c_read_word_signed(self.bus, self.address, ITG3205.GYRO_YOUT_H) + ITG3205.Y_OFFSET
        self.gyro_raw_z = I2CUtils.i2c_read_word_signed(self.bus, self.address, ITG3205.GYRO_ZOUT_H) + ITG3205.Z_OFFSET

        # We convert these to radians for consistency and so we can easily combine later in the filter
        self.gyro_scaled_x = math.radians(self.gyro_raw_x * ITG3205.GYRO_SCALE)
        self.gyro_scaled_y = math.radians(self.gyro_raw_y * ITG3205.GYRO_SCALE)
        self.gyro_scaled_z = math.radians(self.gyro_raw_z * ITG3205.GYRO_SCALE)

    def read_raw_gyro_x(self):
        '''Return the RAW X gyro value'''
        return self.gyro_raw_x
        
    def read_raw_gyro_y(self):
        '''Return the RAW Y gyro value'''
        return self.gyro_raw_y
        
    def read_raw_gyro_z(self):
        '''Return the RAW Z gyro value'''
        return self.gyro_raw_z
    
    def read_scaled_gyro_x(self):
        '''Return the SCALED X gyro value in radians/second'''
        return self.gyro_scaled_x

    def read_scaled_gyro_y(self):
        '''Return the SCALED Y gyro value in radians/second'''
        return self.gyro_scaled_y

    def read_scaled_gyro_z(self):
        '''Return the SCALED Z gyro value in radians/second'''
        return self.gyro_scaled_z

if __name__ == "__main__":
    bus = smbus.SMBus(I2CUtils.i2c_raspberry_pi_bus_number())
    gyro=ITG3205(bus, 0x68, "gyro")
    gyro.read_raw_data()
    print gyro.read_scaled_gyro_x()
    print gyro.read_scaled_gyro_y()
    print gyro.read_scaled_gyro_z()
