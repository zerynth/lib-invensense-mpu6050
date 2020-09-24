# Zerynth - libs - invensense-mpu6050/mpu6050.py
#
# Zerynth library for MPU6050 motion sensor
#
# @Author: Stefano Torneo
#
# @Date: 2020-08-07
# @Last Modified by: 
# @Last Modified time:

"""
.. module:: MPU6050

**************
MPU6050 Module
**************

.. _datasheet: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
               https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
               http://www.i2cdevlib.com/devices/mpu6050#registers

This module contains the Zerynth driver for MPU6050 digital motion sensor. 
The MPU6050 features three 16-bit analog-to-digital converters (ADCs) for digitizing the gyroscope outputs
and three 16-bit ADCs for digitizing the accelerometer outputs. For precision tracking of both fast and slow
motions, the parts feature a user-programmable gyroscope full-scale range of ±250, ±500, ±1000, and
±2000°/sec (dps) and a user-programmable accelerometer full-scale range of ±2g, ±4g, ±8g, and ±16g.
Communication with all registers of the device is performed using I2C at 400kHz.
Additional features include an embedded temperature sensor.

"""

import i2c

GRAVITIY_MS2 = 9.80665

# two's complement
#
# @param      v      integer value to be converted
# @param      n_bit  number of bits of v's representation
#
# @return     the two's complement of v
#
def _tc(v, n_bit=16):
    mask = 2**(n_bit - 1)
    return -(v & mask) + (v & ~mask)

# Define some constants from the datasheet

MPU6050_ADDRESS = 0x68 # 0x69 when AD0 pin to Vcc

# Scale Modifiers
ACCEL_SENSITIVITY_2G = 16384.0
ACCEL_SENSITIVITY_4G = 8192.0
ACCEL_SENSITIVITY_8G = 4096.0
ACCEL_SENSITIVITY_16G = 2048.0

GYRO_SENSITIVITY_250DPS = 131.0
GYRO_SENSITIVITY_500DPS = 65.5
GYRO_SENSITIVITY_1000DPS = 32.8
GYRO_SENSITIVITY_2000DPS = 16.4

# MPU-6050 Registers
REG_WHO_AM_I = 0x75
PWR_MGMT_1 = 0x6B
PWR_MGMT_2 = 0x6C
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B

ACCEL_XOUT0 = 0x3B
ACCEL_XOUT1 = 0x3C
ACCEL_YOUT0 = 0x3D
ACCEL_YOUT1 = 0x3E
ACCEL_ZOUT0 = 0x3F
ACCEL_ZOUT1 = 0x40

TEMP_OUT0 = 0x41
TEMP_OUT1 = 0x42

GYRO_XOUT0 = 0x43
GYRO_XOUT1 = 0x44
GYRO_YOUT0 = 0x45
GYRO_YOUT1 = 0x46
GYRO_ZOUT0 = 0x47
GYRO_ZOUT1 = 0x48

REG_CONFIG = 0x1A

# Motion register
REG_INT_ENABLE = 0x38
REG_INT_STATUS = 0x3A
REG_MOT_DETECT_CTRL = 0x69
REG_MOT_DETECT_STATUS = 0x61
REG_MOT_THRESHOLD = 0x1F
REG_MOT_DURATION = 0x20
REG_ZMOT_THRESHOLD = 0x21
REG_ZMOT_DURATION = 0x22

# Motion values
DELAY = 3
THRESHOLD = 2
DURATION = 5
ZTHRESHOLD = 4
ZDURATION = 2
FF_EN = False
MOT_EN = False
ZMOT_EN = False
    
class MPU6050(i2c.I2C):
    """
    
===============
 MPU6050 class
===============

.. class:: MPU6050(drvname, addr=0x68, clk=400000)

    Creates an intance of the MPU6050 class.

    :param drvname: I2C Bus used '( I2C0, ... )'
    :param addr: Slave address, default 0x68
    :param clk: Clock speed, default 400kHz
    
    Temperature, accelerometer and gyroscope value can be easily obtained from the sensor: ::

        from invensense.mpu6050 import mpu6050

        ...

        mpu = mpu6050.MPU6050(I2C0)

        temp, acc, gyro = mpu.get_values()

    """

    # dictionary of accel full-scale ranges
    accel_fullscale = {
        '2': 0,
        '4': 1,
        '8': 2,
        '16': 3
    }

    # list of accel sensitivity
    accel_sensitivity = [
        ACCEL_SENSITIVITY_2G,
        ACCEL_SENSITIVITY_4G,
        ACCEL_SENSITIVITY_8G,
        ACCEL_SENSITIVITY_16G
    ]

    # dictionary of gyro full-scale ranges
    gyro_fullscale = {
        '250': 0,
        '500': 1,
        '1000': 2,
        '2000': 3
    }

    # list of gyro sensitivity
    gyro_sensitivity = [
        GYRO_SENSITIVITY_250DPS,
        GYRO_SENSITIVITY_500DPS,
        GYRO_SENSITIVITY_1000DPS,
        GYRO_SENSITIVITY_2000DPS
    ]

    def __init__(self, drvname, addr=0x68, clk=400000):
        
        if (addr != 0x68 and addr != 0x69):
            raise ValueError

        i2c.I2C.__init__(self,drvname,addr,clk)
        try:
            self.start()
        except PeripheralError as e:
            print(e)
        
        # Check MPU6050 Who Am I Register
        if (self.write_read(REG_WHO_AM_I, n=1)[0] != 0x68):
            raise ValueError

        # Set Clock source
        self.set_clock_source(1)
        # Set scales
        self.set_accel_fullscale(2)
        self.set_gyro_fullscale(2000)
        # Set dlpf mode
        self.set_dlpf_mode(0)
        # Disable Sleep Mode
        self.set_sleep_mode(False)

    # MPU-6050 Methods

    ##
    ## @brief      Get the value of bit SLEEP from PWR_MGMT_1 register.
    ##
    ## @param      self
    ## @return     value of bit SLEEP from PWR_MGMT_1 register.
    ##
    def is_sleep_mode(self):
        value = self.write_read(PWR_MGMT_1, n=1)[0]
        return ((value >> 6) & 1)

    ##
    ## @brief      Set the bit SLEEP of PWR_MGMT_1 register, according to the value of param state. 
    ##
    ## @param      self
    ## @param      state    boolean value that is the value of bit SLEEP of PWR_MGMT_1 register to set.
    ## @return     nothing
    ##
    def set_sleep_mode(self, state):
        if (state != True and state != False):
            raise ValueError

        value = self.write_read(PWR_MGMT_1, n=1)[0]

        if (state):
            value |= (1 << 6)
        else: 
            value &= ~(1 << 6)

        self.write_bytes(PWR_MGMT_1, value)
    
    def set_dlpf_mode(self, dlpf):
        """
    .. method:: set_dlpf_mode(dlpf)

        **Parameters**:

        **dlpf**: is the DLPF mode to set. Values range accepted are 0-7 (see datasheet).

        Set the DLPF mode.

        """
        if (dlpf not in [0, 1, 2, 3, 4, 5, 6, 7]):
            raise ValueError

        value = self.write_read(REG_CONFIG, n=1)[0]
        value &= 0b11111000
        value |= dlpf
        self.write_bytes(REG_CONFIG, value)
    
    def set_dhpf_mode(self, dhpf):
        """
    .. method:: set_dhpf_mode(dhpf)

        **Parameters**:

        **dhpf**: is the DHPF mode to set. Values accepted are 0, 1, 2, 3, 4 or 7.

        ======== =====================
         dhpf        DHPF mode
        ======== =====================
         0         Reset
         1         On @ 5 Hz
         2         On @ 2.5 Hz
         3         On @ 1.25 Hz
         4         On @ 0.63 Hz
         7         Hold
        ======== =====================

        Set the DHPF mode.

        """
        if (dhpf not in [0, 1, 2, 3, 4, 7]):
            raise ValueError

        value = self.write_read(ACCEL_CONFIG, n=1)[0]
        value &= 0b11111000
        value |= dhpf
        self.write_bytes(ACCEL_CONFIG, value)

    def get_clock_source(self):
        """
    .. method:: get_clock_source()

        Return the clock source the sensor is set to.

        """
        clock_source = self.write_read(PWR_MGMT_1, n=1)[0]
        clock_source &= 0b00000111

        return clock_source

    def set_clock_source(self, clksel):
        """
    .. method:: set_clock_source(clksel)

        **Parameters**:

        **clksel**: is the clock source to set. Values accepted are 0, 1, 2, 3, 4, 5 or 7.

        ======== =====================
         clksel    Clock source
        ======== =====================
         0         Internal 8MHz oscillator
         1         PLL with X axis gyroscope reference
         2         PLL with Y axis gyroscope reference
         3         PLL with Z axis gyroscope reference
         4         PLL with external 32.768kHz reference
         5         PLL with external 19.2MHz reference
         6         Reserved
         7         Stops the clock and keeps the timing generator
        ======== =====================

        Set the clock source. 

        """
        if (clksel not in [0, 1, 2, 3, 4, 5, 7]):
            raise ValueError

        value = self.write_read(PWR_MGMT_1, n=1)[0]
        value &= 0b11111000
        value |=  clksel
        self.write_bytes(PWR_MGMT_1, value)

    def get_temp(self):
        """
    .. method:: get_temp()

        Return the temperature in degrees Celsius.

        """
        # Read the raw data from the registers
        data = self.write_read(TEMP_OUT0, n=2)
        raw_temp = _tc(data[0] << 8 | data[1])
        
        # Get the actual temperature using the formule given in the
        # MPU-6050 Register Map and Descriptions revision 4.2, page 30
        actual_temp = (raw_temp / 340) + 36.53

        # Return the temperature
        return actual_temp
    
    def set_accel_fullscale(self, full_scale):
        """
    .. method:: set_accel_fullscale(full_scale)

        :param full_scale: is the full-scale range to set the accelerometer to. Possible values are 2, 4, 8 or 16.
        
        Set the full-scale range of the accelerometer.

        """
        if (full_scale not in [2, 4, 8, 16]):
            raise ValueError

        # First change it to 0x00 to make sure we write the correct value later
        self.write_bytes(ACCEL_CONFIG, 0x00)

        # get corrisponding full-scale value from dictionary
        full_scale = self.accel_fullscale[str(full_scale)]

        # Write the new full-scale to the ACCEL_CONFIG register
        value = self.write_read(ACCEL_CONFIG, n=1)[0]
        value &= 0b11100111
        value |= (full_scale << 3)
        self.write_bytes(ACCEL_CONFIG, value)

    def get_accel_fullscale(self):
        """
    .. method:: get_accel_fullscale()
        
        Return the full-scale value the accelerometer is set to.
        When something went wrong, it returns -1.
        
        """
        # Get the raw value
        raw_data = self.write_read(ACCEL_CONFIG, n=1)[0]
        raw_data &= 0b00011000
        raw_data >>= 3

        # get accel full-scale
        values = ['2', '4', '8', '16']
        for i in range(4):
            if (raw_data == self.accel_fullscale[values[i]]):
                return values[i]
        return -1

    def get_accel_values(self, g = False):
        """
    .. method:: get_accel_values(g = False)

        :param g: is the format of accelerometer values. 
                  If g = False is m/s^2, otherwise is g. 
                  Default value is False.
        
        Return the X, Y and Z accelerometer values in a dictionary.
        
        """
        if (g != True and g != False):
            raise ValueError

        # Read the raw data from the registers
        data = self.write_read(ACCEL_XOUT0, n=6)
        x = _tc(data[0] << 8 | data[1]) # X-axis value
        y = _tc(data[2] << 8 | data[3]) # Y-axis value
        z = _tc(data[4] << 8 | data[5]) # Z-axis value

        full_scale = self.get_accel_fullscale()

        # set default accel sensitivity
        accel_sensitivity = ACCEL_SENSITIVITY_2G

        # get accel full-scale
        values = ['2', '4', '8', '16']
        for i in range(len(values)):
            if (full_scale == self.accel_fullscale[values[i]]):
                accel_sensitivity = self.accel_sensitivity[i]

        x = x / accel_sensitivity
        y = y / accel_sensitivity
        z = z / accel_sensitivity

        if g is True:
            return {'x': x, 'y': y, 'z': z}
        elif g is False:
            x = x * GRAVITIY_MS2
            y = y * GRAVITIY_MS2
            z = z * GRAVITIY_MS2
            return {'x': x, 'y': y, 'z': z}

    def set_gyro_fullscale(self, full_scale):
        """
    .. method:: set_gyro_fullscale(full_scale)

        :param full_scale: is the full-scale range to set the gyroscope to. Values accepted: 250, 500, 1000 or 2000.
        
        Set the full-scale range of the gyroscope.
        
        """
        if (full_scale not in [250, 500, 1000, 2000]):
            raise ValueError

        # First change it to 0x00 to make sure we write the correct value later
        self.write_bytes(GYRO_CONFIG, 0x00)

        # get gyro full-scale from dictionary
        full_scale = self.gyro_fullscale[str(full_scale)]

        # Write the new full-scale to the ACCEL_CONFIG register
        value = self.write_read(GYRO_CONFIG, n=1)[0]
        value &= 0b11100111
        value |= (full_scale << 3)
        self.write_bytes(GYRO_CONFIG, value)

    def get_gyro_fullscale(self):
        """
    .. method:: get_gyro_fullscale()
        
        Return the full-scale value the gyroscope is set to.
        When something went wrong, it returns -1.
        
        """
        # Get the raw value
        raw_data = self.write_read(GYRO_CONFIG, n=1)[0]
        raw_data &= 0b00011000
        raw_data >>= 3

        # get gyro full-scale
        values = ['250', '500', '1000', '2000']
        for i in range(len(values)):
            if (raw_data == self.gyro_fullscale[values[i]]):
                return values[i]
        return -1

    def get_gyro_values(self):
        """
    .. method:: get_gyro_values()
        
        Return the X, Y and Z gyroscope values in a dictionary.
        
        """
        # Read the raw data from the registers
        data = self.write_read(GYRO_XOUT0, n=6)
        x = _tc(data[0] << 8 | data[1]) # X-axis value
        y = _tc(data[2] << 8 | data[3]) # Y-axis value
        z = _tc(data[4] << 8 | data[5]) # Z-axis value

        full_scale = self.get_gyro_fullscale()

        # set default gyro sensitivity
        gyro_sensitivity = GYRO_SENSITIVITY_250DPS

        # get gyro sensitivity
        values = ['250', '500', '1000', '2000']
        for i in range(len(values)):
            if (full_scale == self.gyro_fullscale[values[i]]):
                gyro_sensitivity = self.gyro_sensitivity[i]

        x = x / gyro_sensitivity
        y = y / gyro_sensitivity
        z = z / gyro_sensitivity

        return {'x': x, 'y': y, 'z': z}

    def get_values(self, g=False):
        """
    .. method:: get_values(g = False)
        
        :param g: is the format of accelerometer values. 
                  If g = False is m/s^2, otherwise is g. 
                  Default value is False.

        Return the values of temperature, gyroscope and accelerometer in a list [temp, accel, gyro].
        
        """
        if (g != True and g != False):
            raise ValueError

        temp = self.get_temp()
        accel = self.get_accel_values(g)
        gyro = self.get_gyro_values()

        return [temp, accel, gyro]

    ##
    ## @brief      Set the value of the register in the position indicated, according to the param state.
    ##
    ## @param      self
    ## @param      reg  is the reg where to write.
    ## @param      pos  is the position of register where to write.
    ## @param      state    boolean value to set the value of the register in the position indicated.
    ## @return     nothing
    ##
    def write_register_bit(self, reg, pos, state):
        if (state != True and state != False):
            raise ValueError

        if (pos < 0):
            raise ValueError

        value = self.write_read(reg, n=1)[0]

        if (state):
            value |= (1 << pos)
        else: 
            value &= ~(1 << pos)

        self.write_bytes(reg, value)

    ##
    ## @brief      Set delay to detect motion, according to the value of param delay.
    ##
    ## @param      self
    ## @param      delay    is the delay to set. Values accepted: 0, 1, 2 or 3.
    ## @return     nothing
    ##
    def set_accel_power_on_delay(self, delay):
        if (delay < 0 or delay > 3):
            raise ValueError

        value = self.write_read(REG_MOT_DETECT_CTRL, n=1)[0]
        value &= 0b11001111
        value |= (delay << 4)
        self.write_bytes(REG_MOT_DETECT_CTRL, value)
    
    ##
    ## @brief      Set the bit FF_EN of INT_ENABLE register, according to the value of param state.
    ##             When set to 1, this bit enables Free Fall detection to generate an interrupt.
    ##
    ## @param      self
    ## @param      state    boolean value to set the bit FF_EN of INT_ENABLE register.
    ## @return     nothing
    ##
    def set_free_fall(self, state):
        if (state != True and state != False):
            raise ValueError

        self.write_register_bit(REG_INT_ENABLE, 7, state)
    
    ##
    ## @brief      Set the bit ZMOT_EN of INT_ENABLE register, according to the value of param state.
    ##             When set to 1, this bit enables Zero Motion detection to generate an interrupt.
    ##
    ## @param      self
    ## @param      state    boolean value to set the bit ZMOT_EN of INT_ENABLE register.
    ## @return     nothing
    ##
    def set_zero_motion(self, state):
        if (state != True and state != False):
            raise ValueError

        self.write_register_bit(REG_INT_ENABLE, 5, state)
    
    ##
    ## @brief      Set the bit MOT_EN of INT_ENABLE register, according to the value of param state.
    ##             When set to 1, this bit enables Motion detection to generate an interrupt.
    ##
    ## @param      self
    ## @param      state    boolean value to set the bit MOT_EN of INT_ENABLE register.
    ## @return     nothing
    ##
    def set_motion(self, state):
        if (state != True and state != False):
            raise ValueError

        self.write_register_bit(REG_INT_ENABLE, 6, state)
    
    ##
    ## @brief      Set the Motion detection threshold according to the param passed.
    ##
    ## @param      self
    ## @param      treshold     is the threshold to set.
    ## @return     nothing
    ##
    def set_motion_detection_threshold(self, threshold):
        if (threshold < 0):
            raise ValueError

        self.write_bytes(REG_MOT_THRESHOLD, threshold)

    ##
    ## @brief      Set the duration counter threshold according to the param passed.
    ##
    ## @param      self
    ## @param      duration     is the duration to set.
    ## @return     nothing
    ##
    def set_motion_detection_duration(self, duration):
        if (duration < 0):
            raise ValueError

        self.write_bytes(REG_MOT_DURATION, duration)
    
    ##
    ## @brief      Set the Zero Motion detection threshold according to the param passed.
    ##
    ## @param      self
    ## @param      threshold     is the threshold to set.
    ## @return     nothing
    ##
    def set_zero_motion_detection_threshold(self, threshold):
        if (threshold < 0):
            raise ValueError

        self.write_bytes(REG_ZMOT_THRESHOLD, threshold)

    ##
    ## @brief      Set the duration counter threshold according to the param passed.
    ##
    ## @param      self
    ## @param      duration     is the duration to set.
    ## @return     nothing
    ##
    def set_zero_motion_detection_duration(self, duration):
        if (duration < 0):
            raise ValueError

        self.write_bytes(REG_ZMOT_DURATION, duration)
    
    def is_motion_detected(self):
        """
    .. method:: is_motion_detected()

        Return 1 if a motion has been detected, otherwise 0.
        
        """
        data = self.write_read(REG_INT_STATUS, n=1)[0]
        is_activity = ((data >> 6) & 1)
        return is_activity
    
    def is_data_ready(self):
        """
    .. method:: is_data_ready()

        Return 1 if data is ready, otherwise 0.
        
        """
        data = self.write_read(REG_INT_STATUS, n=1)[0]
        is_ready = ((data >> 0) & 1)
        return is_ready

    def setup_motion(self):
        """
    .. method:: setup_motion()
        
        Set the configuration for motion detection.
        
        """
        self.set_accel_power_on_delay(DELAY)
        self.set_free_fall(FF_EN)
        self.set_zero_motion(ZMOT_EN)
        self.set_motion(MOT_EN)
        self.set_dhpf_mode(1)
        self.set_motion_detection_threshold(THRESHOLD)
        self.set_motion_detection_duration(DURATION)
        self.set_zero_motion_detection_threshold(ZDURATION)
        self.set_zero_motion_detection_duration(ZTHRESHOLD)