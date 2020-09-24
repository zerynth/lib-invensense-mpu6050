
################################################################################
# Motion Sensor Example
#
# Created: 2020-08-07
# Author: S. Torneo
#
################################################################################

import streams
from invensense.mpu6050 import mpu6050

streams.serial()

try:
    # Setup sensor 
    print("start...")
    mpu = mpu6050.MPU6050(I2C0)
    print("Ready!")
    print("--------------------------------------------------------")
except Exception as e:
    print("Error: ",e)

try:
    while True:
        if (mpu.is_data_ready()):
            temp, acc, gyro = mpu.get_values()
            print("Temperature: ", temp, "C")
            print("Accelerometer: ", acc)
            print("Gyroscope: ", gyro)
            print("--------------------------------------------------------")
        sleep(2000)
except Exception as e:
    print("Error2: ",e)
