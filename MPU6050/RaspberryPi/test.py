from MPU6050 import MPU6050
import time

mpu = MPU6050(0x68)

mpu.reset()
mpu.power_manage()
mpu.gyro_config()
mpu.accel_config()

state = time.time()

while True:
    
    #g_values = mpu.read_gyroscope()
    #a_values = mpu.read_accelerometer()

    print(f"g values are {mpu.read_gyroscope()}")
    print(f"a values are {mpu.read_accelerometer()}")
    
    time.sleep(0.5)
    