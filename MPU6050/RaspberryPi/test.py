from operator import mod
from MPU6050 import MPU6050
import time

mpu = MPU6050(0x68)

# Reset Test
# mpu.reset()

# Temperature Test
while True:
    print("Temperature: ")
    print(mpu.get_temperature())
    
    print("Temperature in Fahrenheit: ")
    print(mpu.get_temperature(fahrenheit=True))
    
    time.sleep(0.5)




    




