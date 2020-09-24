
################################################################################
# Motion Detection Example
#
# Created: 2020-08-25
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
    mpu.setup_motion()
except Exception as e:
    print("Error: ",e)
    
try:
    while True:
        if (mpu.is_motion_detected()):
            print("Motion captured")
        sleep(200)
except Exception as e:
    print("Error2: ",e)
