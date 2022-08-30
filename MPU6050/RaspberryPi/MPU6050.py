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

from Registers import MPURegisters
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
        self.bus.write_byte_data(MPURegisters.MPU6050_ADDRESS_DEFAULT,MPURegisters.MPU6050_PWR_MGMT_1,0x01)
        
        '''
        WRITE DATA on MPU6050_SMPRT_DIV
        By setting 0x00 means, it is 00000000, it is a 8 bit unsigned value.
        Formula : Sample Rate = Gyroscope_Output_Rate/(1 + MPU6050_SMPLRT_DIV)
        initially we are turning off digital low pass filter (DLPF = 000 or 111), therefore Gyroscope_Output_Rate = 8Khz
        We will get maximum samples through this.
        '''
        self.bus.write_byte_data(MPURegisters.MPU6050_ADDRESS_DEFAULT,MPURegisters.MPU6050_SMPRT_DIV,0x00)
        
        '''
        WRITE DATA on MPU6050_CONFIG
        We will set it to 0x00, as we are not using DLPF and EXT_SYNC_SET, and we are keeping as low or 0.
        '''
        self.bus.write_byte_data(MPURegisters.MPU6050_ADDRESS_DEFAULT,MPURegisters.MPU6050_CONFIG,0x00)
        
        
    
    def power_manage(self,mode = 2):
            '''
            @name : power_manage(self,mode = 2)
            @params : 1. SLEEP : When set to true, this bit puts the MPU into sleep mode
                    2. TEMPERATURE_CONTROL : When set to true, enables the onboard temperature sensor.
                    3. CLOCK_SELECT : It will be selected as per user requirement, user can value between 0 to 7
                    
            @desc : This method will give more access to power management register and manage their MPU6050 in more optimised way
                    By default mode will be 2.
            
            Created modes based on sleep and temperature control
            | Sleep | Temperature |
            |   0   |      0      |  Mode - 1
            |   0   |      1      |  Mode - 2
            |   1   |      0      |  Mode - 3
            |   1   |      1      |  Mode - 4
            
            '''
            
            # Handle sleep mode
            # Tried to implement match case, but again that same error :(, so again going with if elif
            
            if(mode == 1):
                write_data = 0x09
            elif(mode == 2):
                write_data = 0x01
            elif(mode == 3):
                write_data = 0x2A #Please checkout this
            elif(mode == 4):
                write_data = 0x21
                    
                    
            self.bus.write_byte_data(MPURegisters.MPU6050_ADDRESS_DEFAULT,MPURegisters.MPU6050_PWR_MGMT_1,write_data)
    
    '''
    Method to calculate value from multiple registers
    For example we have TEMP_OUT_H and TEMP_OUT_L , 0x41 and 0x42, to calculate temperature
    '''
    def read_i2c_data_multiple_registers(self,register):
        '''
        Read two i2c registers and combine them,
        returns the combined read results.
        '''
        
        HIGH = self.bus.read_byte_data(MPURegisters.MPU6050_ADDRESS_DEFAULT,register)
        LOW = self.bus.read_byte_data(MPURegisters.MPU6050_ADDRESS_DEFAULT,register + 1)
        
        #Calculation
        result_value = (HIGH << 8) + LOW 
        
        if(result_value >= 0x8000):
            return -((65535 - result_value) + 1)
        else:
            return result_value
        
        
    def get_temperature(self,fahrenheit=False):
        """
        Reads the temperature from the temperature sensor on MPU-6050.
        Returns the temperature in degrees Celcius or Fahrenheit.
        """
        
        temp_register_value = self.read_i2c_data_multiple_registers(MPURegisters.MPU6050_TEMP_OUT)
        
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
        
        self.bus.write_byte_data(MPURegisters.MPU6050_ADDRESS_DEFAULT,MPURegisters.MPU6050_PWR_MGMT_1,0x80)
        print("Device Resetting...") if Debug else None
        time.sleep(0.01)
        
        
        
        
        
        
        
         
        
        
    
        
        