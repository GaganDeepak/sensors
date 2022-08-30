"""MPU6050 Interface with RaspberryPi I2C port"""

# The MIT License (MIT)
# Copyright (c) 2022 Gagan Deepak and contributors
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from Registers import MPURegisters as mpu6050
from smbus import SMBus
import time

Debug = True # Set this to false when using !

class MPU6050:
    """
    MPU6050 GY-521 Interface with RaspberryPi
    """

    def __init__(self, address, bus=1):
        # Init parameters
        self.address = address
        self.bus = SMBus(bus)
        
        '''
        Initial Conditions: 
        WRITE DATA on MPU6050_PWR_MGMT_1
        By setting 0x01 means , it is 00000 001, we have to focus on last three bits.
        Device Reset Bit is set to 0.
        Sleep Bit is set to 0.
        CYCLE Bit is set to 0.
        TEMP_DIS is set to 0, as it enables the temperature sensor by default.
        CLKSEL Bit is 001, that means we are going with 1, PLL with X axis gyroscopic reference. 
        '''
        self.power_manage(clock_source=1)
        self.sample_rate_divider()
        self.configuration()
        
        
    
    def power_manage(self, reset: bool = False,sleep: bool = False,cycle: bool = False,
                    temp_sense_disable: bool = False,clock_source: int = 0,value: int = 0):
            '''
            Sets the PWR_MGMT_1 register of mpu6050

            :param reset: True = resets all internal registers to their default values
            :type reset: bool
            :param sleep: True = puts the MPU-6050 into sleep mode
            :type sleep: bool
            :param cycle: True = MPU-6050 will cycle between sleep mode and waking up
            :type cycle: bool
            :param temp_sense_disable: True = disables the temperature sensor
            :type temp_sense_disable: bool
            :param clock_source: Specifies the clock source of the device
            :type clock_source: int 3-bit unsigned value
            :param value: The value to set the PWR_MGMT_1 register to
            :type value: int 8-bit unsigned value
            '''
            

            if value != 0:
                self.bus.write_byte_data(mpu6050.ADDRESS_DEFAULT,mpu6050.PWR_MGMT_1,value)
            else:
                byte = self.bus.read_byte_data(mpu6050.ADDRESS_DEFAULT,mpu6050.PWR_MGMT_1)
                value = (reset*128 + sleep*64 + cycle*32 + temp_sense_disable*8 + clock_source) | byte
                self.bus.write_byte_data(mpu6050.ADDRESS_DEFAULT,mpu6050.PWR_MGMT_1,value)
    
    '''
    Method to calculate value from multiple registers
    For example we have TEMP_OUT_H and TEMP_OUT_L , 0x41 and 0x42, to calculate temperature
    '''
    def read_i2c_byte_data(self,register):
        '''
        Read two i2c registers and combine them,
        :param register:address to read two bytes from
        :type register:int
        :return: int data(address+address+1)
        '''
        
        HIGH = self.bus.read_byte_data(mpu6050.ADDRESS_DEFAULT,register)
        LOW = self.bus.read_byte_data(mpu6050.ADDRESS_DEFAULT,register + 1)
        
        #Calculation
        result_value = (HIGH << 8) + LOW 
        
        if(result_value >= 0x8000):
            return -((65535 - result_value) + 1)
        else:
            return result_value
        
        
    def get_temperature(self,fahrenheit: bool =False):
        """
        Reads the temperature from the temperature sensor on MPU-6050.
        :param fahrenheit:True returns the tempature in fahrenheit
        :return: Temperature in Fahrenheit or celsius
        """
        
        temp_register_value = self.read_i2c_byte_data(mpu6050.TEMP_OUT)
        
        #Formula to compute the temp register value to temperature in degree celsius
        # Temperature in degree celsius = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
        temperature_value = (temp_register_value/340) + 36.53
        
        if(fahrenheit):
            # In Fahrenheit
            # Conversion (0°C × 9/5) + 32 = 32°F
            fahrenheit_value = (temperature_value *(9/5)) + 32            
            return fahrenheit_value
        
        else:
            # In Celsius 
            return temperature_value
        
        
    def reset(self):
        """
            Resets the whole MPU6050. 
              
            Reset Value for all registers is 0x00 ,
            but MPU6050_PWR_MGMT_1 will be 0x40 and WHO_AM_I will be 0x68 
        """
        
        self.power_manage(reset = True)
        print("Device Resetting...") if Debug else None
        time.sleep(0.01)

    def sample_rate_divider(self,value: int = 0):
        '''
        WRITE DATA on MPU6050_SMPRT_DIV
        By setting 0x00 means, it is 00000000, it is a 8 bit unsigned value.
        Formula : Sample Rate = Gyroscope_Output_Rate/(1 + MPU6050_SMPLRT_DIV)
        initially we are turning off digital low pass filter (DLPF = 000 or 111), therefore Gyroscope_Output_Rate = 8Khz
        We will get maximum samples through this.
        :param value: sampling rate
        :type value: int 8-bit unsigned value
        '''
        self.bus.write_byte_data(mpu6050.ADDRESS_DEFAULT,mpu6050.SMPRT_DIV,value)

    def configuration(self,ext_sync: int = 0,Dig_low_pass_filter: int = 0, value: int = 0):
        '''
        WRITE DATA on MPU6050_CONFIG
        We will set it to 0x00, as we are not using DLPF and EXT_SYNC_SET, and we are keeping as low or 0.
        :param ext_sync: Configures the FSYNC pin sampling
        :type ext_sync: int 3-bit unsigned value
        :param Dig_low_pass_filter: Configures the DLPF setting
        :type Dig_low_pass_filter: int 3-bit unsigned value
        :param value: configuration value
        :type value: int 6-bit unsigned value
        '''
        if value == 0:
            value = (ext_sync<<3) | Dig_low_pass_filter
        self.bus.write_byte_data(mpu6050.ADDRESS_DEFAULT,mpu6050.CONFIG,value)
    
    def gyro_config(self,XG_ST: bool = False, YG_ST: bool = False, ZG_ST:bool = False, FULL_SCALE_RANGE: int = 0, value: int = 0):
        """This register is used to trigger gyroscope self-test and configure the gyroscopes full scale range.
        :param XG_ST: True = Setting this bit causes the X axis gyroscope to perform self test
        :type XG_ST: bool
        :param YG_ST: True = Setting this bit causes the Y axis gyroscope to perform self test
        :type YG_ST: bool
        :param ZG_ST: True = Setting this bit causes the Z axis gyroscope to perform self test
        :type ZG_ST: bool
        :param Full_Scale_Range : Selects the full scale range of gyroscopes
        :type Full_Scale_Range: int 2-bit unsigned value
        """
        if value == 0:
            value = XG_ST*128 + YG_ST*64 + ZG_ST*32+ (FULL_SCALE_RANGE<<3) | 0
        self.bus.write_byte_data(mpu6050.ADDRESS_DEFAULT,mpu6050.GYRO_CONFIG,value)

    def accel_config(self,XA_ST: bool = False,YA_ST: bool = False,ZA_ST: bool = False,FULL_SCALE_RANGE: int = 0,value: int = 0):
        """This register is used to trigger accel self-test and configure the accel scopes full scale range.
        :param XA_ST: True = Setting this bit causes the X axis accel to perform self test
        :type XA_ST: bool
        :param YA_ST: True = Setting this bit causes the Y axis accel to perform self test
        :type YA_ST: bool
        :param ZA_ST: True = Setting this bit causes the Z axis accel to perform self test
        :type ZA_ST: bool
        :param FULL_SCALE_RANGE : Selects the full scale range of accelerometer
        :type FULL_SCALE_Range: int 2-bit unsigned value
        """
        if value == 0:
            value = XA_ST*128 + YA_ST*64 + ZA_ST*32+ (FULL_SCALE_RANGE<<3) | 0
        self.bus.write_byte_data(mpu6050.ADDRESS_DEFAULT,mpu6050.ACCEL_CONFIG,value)
    
    def read_accelerometer(self,ACCEL_XOUT: bool = True, ACCEL_YOUT: bool = True,ACCEL_ZOUT: bool = True):
        """
        Fetches Recent accelerometer values
        """
        if ACCEL_XOUT:
            ACCEL_XOUT_data = self.bus.read_i2c_byte_data(mpu6050.ACCEL_XOUT_H)

        if ACCEL_YOUT:
            ACCEL_YOUT_data = self.bus.read_i2c_byte_data(mpu6050.ACCEL_XOUT_H+2)

        if ACCEL_ZOUT:
            ACCEL_ZOUT_data = self.bus.read_i2c_byte_data(mpu6050.ACCEL_XOUT_H+4)

        return {'x': ACCEL_XOUT_data, 'y': ACCEL_YOUT_data, 'z': ACCEL_ZOUT_data}
    
    def read_gyroscope(self,GYRO_XOUT: bool = True, GYRO_YOUT: bool = True,GYRO_ZOUT: bool = True):
        """
        Fetches Recent gyroscope values
        """
        if GYRO_XOUT:
            GYRO_XOUT_data = self.bus.read_i2c_byte_data(mpu6050.GYRO_XOUT_H)
        if GYRO_YOUT:
            GYRO_YOUT_data = self.bus.read_i2c_byte_data(mpu6050.GYRO_XOUT_H+2)
        if GYRO_ZOUT:
            GYRO_ZOUT_data = self.bus.read_i2c_byte_data(mpu6050.GYRO_XOUT_H+4)
        
        return {'x': GYRO_XOUT_data, 'y': GYRO_YOUT_data, 'z': GYRO_ZOUT_data}