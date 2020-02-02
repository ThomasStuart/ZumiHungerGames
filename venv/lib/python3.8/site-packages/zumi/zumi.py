import os
import smbus2
import time
import configparser
from zumi.protocol import Accelerometer, Gyro, Device, MPURegister, Axis
from zumi.protocol import CommandType, MotorCommand, Command
import RPi.GPIO as GPIO
import math
import logging
from uuid import getnode as get_mac

def preboot_to_postboot():
    from zumi.util.screen import Screen
    bus = smbus2.SMBus(1)

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(4, GPIO.IN)

    while GPIO.input(4) == 1:
        continue

    time.sleep(1)

    bus.write_byte(0x04, 0b11000000)

    screen = Screen()
    screen.draw_image_by_name('wakingup_witheyes')


def _is_in_range(value, low, high):
    return low <= value <= high


def _convert_to_arduino_speed(arduino_speed):
    if arduino_speed < 0:
        arduino_speed *= -1
        arduino_speed += 127
    return arduino_speed

def _create_command(command_type, command, extra_bits=0):
    command_type = command_type << 6
    command = command << 3
    return command_type | command | extra_bits

def _bit_read(byte, index):
    return byte >> index & 1 == 1


def _bit_set(byte, index):
    return byte | (1 << index)


def _bit_clear(byte, index):
    return byte & ~(1 << index)


class ValueOutOfRangeError(Exception):
    pass



#credit to https://github.com/RigacciOrg/py-qmc5883l
class QMC5883L:
    """Interface for the QMC5883l 3-Axis Magnetic Sensor."""

    def __init__(self,bus):

        self.DFLT_ADDRESS = 0x0d     #ADDRESS for the QMC Compass module
        self.REG_XOUT_LSB = 0x00     # Output Data Registers for magnetic sensor.
        self.REG_XOUT_MSB = 0x01
        self.REG_YOUT_LSB = 0x02
        self.REG_YOUT_MSB = 0x03
        self.REG_ZOUT_LSB = 0x04
        self.REG_ZOUT_MSB = 0x05
        self.REG_STATUS_1 = 0x06     # Status Register.
        self.REG_TOUT_LSB = 0x07     # Output Data Registers for temperature.
        self.REG_TOUT_MSB = 0x08
        self.REG_CONTROL_1 = 0x09    # Control Register #1.
        self.REG_CONTROL_2 = 0x0a    # Control Register #2.
        self.REG_RST_PERIOD = 0x0b   # SET/RESET Period Register.
        self.REG_CHIP_ID = 0x0d      # Chip ID register.

        # Flags for Status Register #1.
        self.STAT_DRDY = 0b00000001  # Data Ready.
        self.STAT_OVL = 0b00000010   # Overflow flag.
        self.STAT_DOR = 0b00000100   # Data skipped for reading.

        # Flags for Status Register #2.
        self.INT_ENB = 0b00000001    # Interrupt Pin Enabling.
        self.POL_PNT = 0b01000000    # Pointer Roll-over.
        self.SOFT_RST = 0b10000000   # Soft Reset.

        # Flags for Control Register 1.
        self.MODE_STBY = 0b00000000  # Standby mode.
        self.MODE_CONT = 0b00000001  # Continuous read mode.
        self.ODR_10HZ = 0b00000000   # Output Data Rate Hz.
        self.ODR_50HZ = 0b00000100
        self.ODR_100HZ = 0b00001000
        self.ODR_200HZ = 0b00001100
        self.RNG_2G = 0b00000000     # Range 2 Gauss: for magnetic-clean environments.
        self.RNG_8G = 0b00010000     # Range 8 Gauss: for strong magnetic fields.
        self.OSR_512 = 0b00000000    # Over Sample Rate 512: less noise, more power.
        self.OSR_256 = 0b01000000
        self.OSR_128 = 0b10000000
        self.OSR_64 = 0b11000000     # Over Sample Rate 64: more noise, less power.

        self.output_data_rate=self.ODR_200HZ
        self.output_range=self.RNG_8G
        self.oversampling_rate=self.OSR_512
        self.address = self.DFLT_ADDRESS
        self.bus = bus

        self._declination = 0.0
        self._calibration = [[1.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0],
                             [0.0, 0.0, 1.0]]
        chip_id = self._read_byte(self.REG_CHIP_ID)
        self.mode_cont = (self.MODE_CONT | self.output_data_rate | self.output_range
                          | self.oversampling_rate)
        self.mode_stby = (self.MODE_STBY | self.ODR_10HZ | self.RNG_2G | self.OSR_64)
        self.mode_continuous()

    def mode_continuous(self):
        """Set the device in continuous read mode."""
        self._write_byte(self.REG_CONTROL_2, self.SOFT_RST)  # Soft reset.
        self._write_byte(self.REG_CONTROL_2, self.INT_ENB)  # Disable interrupt.
        self._write_byte(self.REG_RST_PERIOD, 0x01)  # Define SET/RESET period.
        self._write_byte(self.REG_CONTROL_1, self.mode_cont)  # Set operation mode.

    def mode_standby(self):
        """Set the device in standby mode."""
        self._write_byte(self.REG_CONTROL_2, self.SOFT_RST)
        self._write_byte(self.REG_CONTROL_2, self.INT_ENB)
        self._write_byte(self.REG_RST_PERIOD, 0x01)
        self._write_byte(self.REG_CONTROL_1, self.mode_stby)  # Set operation mode.

    def _write_byte(self, registry, value):
        self.bus.write_byte_data(self.address, registry, value)
        time.sleep(0.01)

    def _read_byte(self, registry):
        return self.bus.read_byte_data(self.address, registry)

    def _read_word(self, registry):
        """Read a two bytes value stored as LSB and MSB."""
        low = self.bus.read_byte_data(self.address, registry)
        high = self.bus.read_byte_data(self.address, registry + 1)
        val = (high << 8) + low
        return val

    def _read_word_2c(self, registry):
        """Calculate the 2's complement of a two bytes value."""
        val = self._read_word(registry)
        if val >= 0x8000:  # 32768
            return val - 0x10000  # 65536
        else:
            return val

    def get_data(self):
        """Read data from magnetic and temperature data registers."""
        i = 0
        [x, y, z, t] = [None, None, None, None]

        while i < 20:  # Timeout after about 0.20 seconds.
            status = self._read_byte(self.REG_STATUS_1)
            if status & self.STAT_OVL:
                # Some values have reached an overflow.
                msg = ("Magnetic sensor overflow.")
                if self.output_range == self.RNG_2G:
                    msg += " Consider switching to RNG_8G output range."
                logging.warning(msg)
            if status & self.STAT_DOR:
                # Previous measure was read partially, sensor in Data Lock.
                x = self._read_word_2c(self.REG_XOUT_LSB)
                y = self._read_word_2c(self.REG_YOUT_LSB)
                z = self._read_word_2c(self.REG_ZOUT_LSB)
                continue
            if status & self.STAT_DRDY:
                # Data is ready to read.
                x = self._read_word_2c(self.REG_XOUT_LSB)
                y = self._read_word_2c(self.REG_YOUT_LSB)
                z = self._read_word_2c(self.REG_ZOUT_LSB)
                t = self._read_word_2c(self.REG_TOUT_LSB)
                break
            else:
                time.sleep(0.005)
                # Waiting for DRDY.
                i += 1
        return [x, y, z, t]

    def get_magnet_raw(self):
        """Get the 3 axis values from magnetic sensor."""
        [x, y, z, t] = self.get_data()
        return [x, y, z]

    def get_magnet(self):
        """Return the horizontal magnetic sensor vector with (x, y) calibration applied."""
        [x, y, z] = self.get_magnet_raw()
        if x is None or y is None:
            [x1, y1] = [x, y]
        else:
            c = self._calibration
            x1 = x * c[0][0] + y * c[0][1] + c[0][2]
            y1 = x * c[1][0] + y * c[1][1] + c[1][2]
        return [x1, y1]

    def get_bearing_raw(self):
        """Horizontal bearing (in degrees) from magnetic value X and Y."""
        [x, y, z] = self.get_magnet_raw()
        if x is None or y is None:
            return None
        else:
            b = math.degrees(math.atan2(y, x))
            if b < 0:
                b += 360.0
            return b

    def get_bearing(self):
        """Horizontal bearing, adjusted by calibration and declination."""
        [x, y] = self.get_magnet()
        if x is None or y is None:
            return None
        else:
            b = math.degrees(math.atan2(y, x))
            if b < 0:
                b += 360.0
            b += self._declination
            if b < 0.0:
                b += 360.0
            elif b >= 360.0:
                b -= 360.0
        return b

    def get_temp(self):
        """Raw (uncalibrated) data from temperature sensor."""
        [x, y, z, t] = self.get_data()
        return t

    def set_declination(self, value):
        """Set the magnetic declination, in degrees."""
        # magnetic declination http://www.magnetic-declination.com/what-is-magnetic-declination.php
        # use this site to find declination http://www.magnetic-declination.com/
        try:
            d = float(value)
            if d < -180.0 or d > 180.0:
                logging.error(u'Declination must be >= -180 and <= 180.')
            else:
                self._declination = d
        except:
            logging.error(u'Declination must be a float value.')

    def get_declination(self):
        """Return the current set value of magnetic declination."""
        return self._declination

    def set_calibration(self, value):
        """Set the 3x3 matrix for horizontal (x, y) magnetic vector calibration."""
        c = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        try:
            for i in range(0, 3):
                for j in range(0, 3):
                    c[i][j] = float(value[i][j])
            self._calibration = c
        except:
            logging.error(u'Calibration must be a 3x3 float matrix.')

    def get_calibration(self):
        """Return the current set value of the calibration matrix."""
        return self._calibration

    declination = property(fget=get_declination,
                           fset=set_declination,
                           doc=u'Magnetic declination to adjust bearing.')

    calibration = property(fget=get_calibration,
                           fset=set_calibration,
                           doc=u'Transformation matrix to adjust (x, y) magnetic vector.')

class MPU:
    def __init__(self, bus):
        """
        Initialize registers for the MPU6050. Please consult the MPU6050 spec sheet for more details
        """
        self.bus = bus
        self.offset_path = "/home/pi/offsets.txt"

        """
        Register DIV / 19
        Determines sample rate by dividing the gyroscope output rate by the sent value. By default, the gyroscope 
        outputs at 8khz, the same as the internal oscillator

        Register PWR_M / 107
        Determines power source and clock source. Currently, the rightmost bit is set, meaning that the X axis of the
        gyroscope is used to help synchronize the clock

        Register CONFIG / 26
        Configures the MPU6050 chip. Specifically, it allows one to capture the sample rate of an external device
        using the FSYNC pin, and allows one to set a Digital Low Pass Filter. Currently, all bits are cleared, meaning
        that neither feature is enabled

        Register GYRO_CONFIG / 27
        Configures the gyroscope. Specifically, it allows one to set the full range of the gyroscopes' readings, as
        well as allow the gyroscope to perform a self-diagnostics test. Currently, only bits 3 and 4 are set, meaning
        that the gyroscope can read a max/min of 2000/-2000 degrees per second, and the gyroscope will not perform a
        self-diagnostics test

        Register INT_EN / 56
        Allows one to enable interruptions from different sources. Currently, the rightmost bit is set, which means
        that an interrupt occurs whenever data is ready, or when all the sensor registers have been written to
        """
        self.ACCEL_CONFIG = 0x1C
        self.GYRO_CONFIG = 0x1B

        self.PWR_MGMT_1 = 0x6B
        self.PWR_MGMT_2 = 0x6C

        self.DIV = 0x19
        self.CONFIG = 0x1A
        self.INT_EN = 0x38

        # Pre-defined ranges I2C registers
        self.ACCEL_RANGE_2G = 0x00
        self.ACCEL_RANGE_4G = 0x08
        self.ACCEL_RANGE_8G = 0x10
        self.ACCEL_RANGE_16G = 0x18

        # Scale Modifiers to convert to G
        self.ACCEL_SCALE_MODIFIER_2G = 16384.0
        self.ACCEL_SCALE_MODIFIER_4G = 8192.0
        self.ACCEL_SCALE_MODIFIER_8G = 4096.0
        self.ACCEL_SCALE_MODIFIER_16G = 2048.0
        #-----------------------
        self.GYRO_RANGE_250DEG = 0x00
        self.GYRO_RANGE_500DEG = 0x08
        self.GYRO_RANGE_1000DEG = 0x10
        self.GYRO_RANGE_2000DEG = 0x18

        #scale modifiers to convert to degrees per second
        self.GYRO_SCALE_MODIFIER_250DEG = 131.0
        self.GYRO_SCALE_MODIFIER_500DEG = 65.5
        self.GYRO_SCALE_MODIFIER_1000DEG = 32.8
        self.GYRO_SCALE_MODIFIER_2000DEG = 16.4

        self.DISABLE_FSYNC = 0

        #write to sample rate register
        self.bus.write_byte_data(Device.MPU, self.DIV, 7)

        #wakeup the MPU6050
        self.bus.write_byte_data(Device.MPU, self.PWR_MGMT_1, 0x00)

        #disable FSYNC, set 260 hz acc filtering and 256 hz Gyro filtering
        self.bus.write_byte_data(Device.MPU, self.CONFIG, self.DISABLE_FSYNC)

        #set gyro range to +-1000 degrees a second
        self.bus.write_byte_data(Device.MPU, self.GYRO_CONFIG, self.GYRO_RANGE_1000DEG)

        #set accel range to +-2G
        self.bus.write_byte_data(Device.MPU, self.ACCEL_CONFIG, self.ACCEL_RANGE_2G)

        time.sleep(0.3)

        #just calls Axis object with the correct scalar parameter
        self.calibrate_gyro = Axis(scalar=self.GYRO_SCALE_MODIFIER_1000DEG)
        self.calibrate_accel = Axis(scalar=self.ACCEL_SCALE_MODIFIER_2G)

        # If offset file exists and mac address matches, read calibration
        if self.check_mpu_offsets(self.offset_path):
            print("Gyroscope previously calibrated")

            lines = [line.rstrip('\n') for line in open(self.offset_path)]
            self.calibrate_gyro.X = float(lines[1])
            self.calibrate_gyro.Y = float(lines[2])
            self.calibrate_gyro.Z = float(lines[3])
            self.calibrate_accel.X = float(lines[4])
            self.calibrate_accel.Y = float(lines[5])
            self.calibrate_accel.Z = float(lines[6])

        else:
            print("Calibrating gyroscope now")
            self.calibrate_MPU()


    def set_gyro_250_scale(self):
        self.bus.write_byte_data(Device.MPU, self.GYRO_CONFIG, self.GYRO_RANGE_250DEG)
        time.sleep(0.1)
        self.calibrate_gyro = Axis(scalar=self.GYRO_SCALE_MODIFIER_250DEG)
        self.calibrate_MPU()

    def set_gyro_500_scale(self):

        self.bus.write_byte_data(Device.MPU, self.GYRO_CONFIG, self.GYRO_RANGE_500DEG)
        time.sleep(0.1)
        self.calibrate_gyro = Axis(scalar=self.GYRO_SCALE_MODIFIER_500DEG)
        self.calibrate_MPU()

    def set_gyro_1000_scale(self):

        self.bus.write_byte_data(Device.MPU, self.GYRO_CONFIG, self.GYRO_RANGE_1000DEG)
        time.sleep(0.1)
        self.calibrate_gyro = Axis(scalar=self.GYRO_SCALE_MODIFIER_1000DEG)
        self.calibrate_MPU()

    def set_gyro_2000_scale(self):

        self.bus.write_byte_data(Device.MPU, self.GYRO_CONFIG, self.GYRO_RANGE_2000DEG)
        time.sleep(0.1)
        self.calibrate_gyro = Axis(scalar=self.GYRO_SCALE_MODIFIER_2000DEG)
        self.calibrate_MPU()

    def set_acc_2g_scale(self):
        self.bus.write_byte_data(Device.MPU, self.ACCEL_CONFIG, self.ACCEL_RANGE_2G)
        time.sleep(0.1)
        self.calibrate_accel = Axis(scalar=self.ACCEL_SCALE_MODIFIER_2G)

    def set_acc_4g_scale(self):
        self.bus.write_byte_data(Device.MPU, self.ACCEL_CONFIG, self.ACCEL_RANGE_4G)
        time.sleep(0.1)
        self.calibrate_accel = Axis(scalar=self.ACCEL_SCALE_MODIFIER_4G)

    def set_acc_8g_scale(self):
        self.bus.write_byte_data(Device.MPU, self.ACCEL_CONFIG, self.ACCEL_RANGE_8G)
        time.sleep(0.1)
        self.calibrate_accel = Axis(scalar=self.ACCEL_SCALE_MODIFIER_8G)

    def set_acc_16g_scale(self):
        self.bus.write_byte_data(Device.MPU, self.ACCEL_CONFIG, self.ACCEL_RANGE_16G)
        time.sleep(0.1)
        self.calibrate_accel = Axis(scalar=self.ACCEL_SCALE_MODIFIER_16G)

    def check_mpu_offsets(self, offset_path):
        calibrated = False
        # First check if offset file exists
        if os.path.isfile(offset_path):
            print("Gyro offset file already exists.")
            lines = [line.rstrip('\n') for line in open(offset_path)]
            # Check if the number makes sense
            if float(lines[0]) > 0:
                # Check if current mac address is same
                if get_mac() - int(float(lines[0])) == 0:
                    print("Verified Pi Zero is the same")
                    calibrated = True
                else:
                    print("Pi Zero is different.")
        return calibrated

    def print_offsets(self):
        print("gyro offsets", self.calibrate_gyro.X, self.calibrate_gyro.Y, self.calibrate_gyro.Z)
        print("accel offsets", self.calibrate_accel.X, self.calibrate_accel.Y, self.calibrate_accel.Z)

    def calibrate_MPU(self, count=100):
        """
        Reads from previous offsets unless file does not exist or recalibrate is manually set to True.
        Take the average of multiple readings from a sensor. The result is the error of the sensor
        Args: Number of sensor readings to take and a boolean for recalibration.
        """
        print("Creating new offset file...")

        for i in range(count):
            if i%50 == 0 :
                print(".")
            current_accel = self.read_multiple_raw_MPU_data(Accelerometer.X, Accelerometer.Y, Accelerometer.Z)
            current_gyro = self.read_multiple_raw_MPU_data(Gyro.X, Gyro.Y, Gyro.Z)
            self.calibrate_accel.add_value(current_accel)
            self.calibrate_gyro.add_value(current_gyro)

        self.calibrate_gyro.calculate_error(count)
        self.calibrate_accel.calculate_error(count)
        self.write_txt_data()

    def write_txt_data(self):
        file = open("/home/pi/offsets.txt", 'w')

        # Write mac address first
        mac = get_mac()
        file.write(str(mac)+"\n")

        # Write mpu6050 data
        file.write(str(self.calibrate_gyro.X)+"\n")
        file.write(str(self.calibrate_gyro.Y)+"\n")
        file.write(str(self.calibrate_gyro.Z)+ "\n")
        file.write(str(self.calibrate_accel.X)+"\n")
        file.write(str(self.calibrate_accel.Y)+ "\n")
        file.write(str(self.calibrate_accel.Z)+ "\n")
        file.close()

    def get_temp(self):
        """
        Reads the temperature from the onboard temperature sensor of the MPU-6050.
        Returns the temperature in degrees Celcius.
        """
        TEMP_OUT0 = 0x41
        raw_temp = self.__read_raw_MPU_data(TEMP_OUT0)
        #convert to celsius
        actual_temp = (raw_temp / 340) + 36.53
        return actual_temp

    def get_z_ang_speed(self):
        GYRO_ZOUT0 = 0x47
        z_angular_speed = self.__read_raw_MPU_data(GYRO_ZOUT0)
        return z_angular_speed

    def __read_raw_MPU_data(self, addr):
        """
        Read a high and low byte from the MPU6050
        Args: 1 hex address, from which the MPU6050 will obtain a sensor reading
        """
        high = self.bus.read_byte_data(Device.MPU, addr)
        low = self.bus.read_byte_data(Device.MPU, addr + 1)
        value = ((high << 8) | low)
        if value > 32768:
            value = value - 65536
        return value

    def read_multiple_raw_MPU_data(self, *args):
        """
        Read a high and low byte from the MPU6050
        Args: 1 hex address, from which the MPU6050 will obtain a sensor reading
        """
        result = []
        for item in args:
            result.append(self.__read_raw_MPU_data(item))
        return result

    def read_MPU_data(self, data_index):
        """
        Returns a single MPU6050 reading
        Args: 1 integer, from 0 to 5. Each number corresponds to a different MPU6050 sensor reading:

        0 / Accelerometer X-axis
        1 / Accelerometer Y-axis
        2 / Accelerometer Z-axis
        3 / Gyroscope X-axis
        4 / Gyroscope Y-axis
        5 / Gyroscope Z-axis
        """
        data = self.read_all_MPU_data()
        return data[data_index]

    def read_all_MPU_data(self):
        """
        Returns the calibrated readings for the accelerometer and gyroscope in the form of a list. Use
        get_mpu_data(data_index) to return a single value instead
        Args: None
        """
        data = self.read_multiple_raw_MPU_data(Accelerometer.X, Accelerometer.Y, Accelerometer.Z,
                                                 Gyro.X, Gyro.Y, Gyro.Z)
        scalar_data = [self.calibrate_accel.scalar, self.calibrate_accel.scalar, self.calibrate_accel.scalar,
                       self.calibrate_gyro.scalar, self.calibrate_gyro.scalar, self.calibrate_gyro.scalar]


        calibration_data = [0, 0, 0, self.calibrate_gyro.X, self.calibrate_gyro.Y, self.calibrate_gyro.Z]

        #convert raw 10 bit data to into correct units, g for acceleration and degrees/second for gyro
        data = [data / scalar_data for data, scalar_data in zip(data, scalar_data)]

        #remove the offset for the data points.
        data = [data - calibration_data for data, calibration_data in zip(data, calibration_data)]
        # subtracting error
        return data


class Zumi:
    def __init__(self):
        print(" Starting Zumi ")
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(4, GPIO.IN)
        self.check_arduino_is_master()
        self.bus = smbus2.SMBus(1)
        self.disable_arduino_preboot()

        self.MAX_USER_SPEED = 80
        self.MIN_I2C_DELAY = 0.01
        self.stop()
        self.is_moving = 0
        self.LED_user_pattern = 0
        self.LED_signals = 0
        self.all_lights_off()
        self.play_note(0,0)

        self.mpu_list = [0, 0, 0, 0, 0, 0, 0]
        self.angle_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.time_past = time.time()
        self.PID_time_past = time.time()
        self.error_sum = 0
        self.error_past = 0
        self.mpu = MPU(self.bus)
        self.sensors_found = self.check_i2c_sensors()

        for sensor in self.sensors_found:
            if( sensor == 0xd):
                #if this board has the compass
                # then create the compass object
                self.compass = QMC5883L(self.bus)

    def check_i2c_sensors(self):
        sensor_list = []
        #go through all addresses and
        #check if any device found
        for device in range(128):
            try:
                self.bus.read_byte(device)
                #print(hex(device))
                #sensor_list.append(hex(device))
                sensor_list.append(device)
                if(device == 0x3c):
                    print("OLED Screen detected")
                if(device == 0x68):
                    print("Gyroscope & Accelerometer detected")
                if(device == 0xd):
                    print("Compass detected")
                if(device == 0x4):
                    print("Zumi board detected")
            except: # exception if read_byte fails
                pass
        return sensor_list

    def check_arduino_is_master(self):
        if(GPIO.input(4) == 1):
            print("Zumi board I2C busy, wait for white LED.")
        while GPIO.input(4) == 1:
            time.sleep(0.1)
        print("Pi Zero I2C is available")

    def disable_arduino_preboot(self):
        arduino_address = 0x04
        while 1:
            try:
                self.bus.write_byte(arduino_address, 0b11000000)
                break
            except IOError:
                time.sleep(0.1)
                continue

    def reset_PID(self):
        '''
        The error values for error integral and error past
        will need to be reset every time the
        PID needs to be used for driving straight or turning
        '''

        self.PID_time_past = time.time()
        self.error_sum = 0
        self.error_past = 0

    def reset_gyro(self):
        '''
        Resets the angles list and the "time past" variable
        used in the gyro angle calculation
        '''
        self.angle_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.time_past = time.time()

    def reset_drive(self):
        '''
        calls reset PID and reset Gyro
        '''
        self.reset_PID()
        self.reset_gyro()

    def clamp(self, n, smallest, largest):
        '''
        :param n: input number that you wish to clamp
        :param smallest: lowest value n can be
        :param largest:  largest value n should be
        :return:
        '''
        return max(smallest, min(n, largest))


    def get_tilt(self, x_acc, y_acc, z_acc):
        '''
        #:param x_acc: x acceleration in multiples of 1g = 9.8m/s^2
        #:param y_acc: y acceleration in multiples of 1g = 9.8m/s^2
        #:param z_acc: z acceleration in multiples of 1g = 9.8m/s^2
        #:return: orientation state, rotation angle x, rotation angle y , rotation angle z

        #using the acceleration values one can find the rotation angles
        #with respect to the strongest force being applied to Zumi
        #this force is usually gravity

         #   orientation state
         #-1 = unknown
         # 0 = probably falling or moving between states
         # 1 = camera straight up
         # 2 = camera facing down
         # 3 = on right side
         # 4 = on left side
         # 5 = wheels on floor
         # 6 = wheels facing up (upside down)
         # 7 = accelerating faster than 1 g
        '''

        vector_acc = ((y_acc ** 2) + (x_acc ** 2) + (z_acc ** 2)) ** 0.5

        x_perc = x_acc * abs(x_acc) / vector_acc ** 2
        y_perc = y_acc * abs(y_acc) / vector_acc ** 2
        z_perc = z_acc * abs(z_acc) / vector_acc ** 2

        r_x = 2 * 180.0 / math.pi * math.atan(x_perc)
        r_y = 2 * 180.0 / math.pi * math.atan(y_perc)
        r_z = 2 * 180.0 / math.pi * math.atan(z_perc)

        orientation_state = -1
        threshold = 0.55
        # threshold percent of gravity on that axis so 0.55 = 55%

        # MPU sometimes overshoots and
        # reads more than 1g. Not all axis are the same
        if vector_acc < 1.25:
            if x_perc > threshold:
                orientation_state = 1
            elif x_perc < -1 * threshold:
                orientation_state = 2
            elif y_perc > threshold:
                orientation_state = 3
            elif y_perc < -1 * threshold:
                orientation_state = 4
            elif z_perc > threshold:
                orientation_state = 5
            elif z_perc < -1 * threshold:
                orientation_state = 6
            else:
                orientation_state = 0
        elif vector_acc >1.25:
            orientation_state = 7

        return orientation_state, r_x, r_y, r_z

    def get_orientation(self):
        '''
        reads the MPU list and calculates the orientation
        '''
        x_acc = self.mpu_list[0]
        y_acc = self.mpu_list[1]
        z_acc = self.mpu_list[2]

        return self.get_tilt(x_acc, y_acc, z_acc)[0]

    def update_angles(self):

        '''
        #Method that reads angular speeds and update the list of angles
        #angle_list = [Gyro x,Gyro y,Gyro z,Acc x,Acc y,Comp x,Comp y]
        #The first 3 are angles produced from the gyroscope readings
        #The Acceleration angles are produced by finding the tilt with respect to gravity
        #The complementary filtered angles are produced
        #by combining both Gyro and Acc angles for the x and y axis
        #gyro_multiplier: adjust if gyro readings not scaled correctly
        #:return: angle_list = [Gyro x,Gyro y,Gyro z,Acc x,Acc y,Comp x,Comp y]
        '''

        # update all the mpu readings
        self.mpu_list = self.mpu.read_all_MPU_data()
        time_now = time.time()
        dt = (time_now - self.time_past)
        self.time_past = time_now

        x_acc = self.mpu_list[0]
        y_acc = self.mpu_list[1]
        z_acc = self.mpu_list[2]

        # In order to get the correct gyro speed multiply by gyro_multiplier
        # adjust if your Zumi does not get accurate angular speed readings
        angular_speed_x = self.mpu_list[3]
        angular_speed_y = self.mpu_list[4]
        angular_speed_z = self.mpu_list[5]

        # integrate the angle: add change in angle to previous angle
        self.angle_list[0] = self.angle_list[0] + angular_speed_x * dt
        self.angle_list[1] = self.angle_list[1] + angular_speed_y * dt
        self.angle_list[2] = self.angle_list[2] + angular_speed_z * dt

        # calculate tilt angles for roll and pitch with respect to the strongest force
        orient, r_x, r_y, r_z = self.get_tilt(x_acc, y_acc, z_acc)

        dist_y_z = math.hypot(y_acc, z_acc)
        dist_x_z = math.hypot(x_acc, z_acc)

        # x axis tilt angle
        if orient == 5 or orient == 3 or orient == 4:
            self.angle_list[3] = 180.0 / math.pi * math.atan2(y_acc, z_acc)

        elif orient == 2 or orient == 1:
            self.angle_list[3] = 180.0 / math.pi * math.atan(y_acc / dist_x_z)

        else:
            self.angle_list[3] = self.angle_list[0]

        # y axis tilt angle
        if orient == 5 or orient == 2 or orient == 1:
            self.angle_list[4] = -1*180.0 / math.pi * math.atan2(x_acc, z_acc)

        elif orient == 3 or orient == 4:
            self.angle_list[4] = -1*180.0 / math.pi * math.atan(x_acc / dist_y_z)
        else:
            self.angle_list[4] = self.angle_list[1]

        # complementary filter
        alpha = 0.02
        # take 98% of the gyro angle and add it to 2% of the acc angle
        self.angle_list[5] = self.angle_list[0] * (1 - alpha) + self.angle_list[3] * alpha
        self.angle_list[6] = self.angle_list[1] * (1 - alpha) + self.angle_list[4] * alpha

        # rotation angles r_x,r_y,r_z with respect to strongest acceleration
        # usually [gravity]. These angles dont work below (-90) or above (+90)
        self.angle_list[7] = r_x
        self.angle_list[8] = r_y
        self.angle_list[9] = r_z

        return self.angle_list

    def read_z_angle(self):
        # updates and reads only the  Z angle
        angle_list = self.update_angles()
        return angle_list[2]


    def drive_at_angle(self, max_speed, base_speed, desired_angle, k_p, k_d, k_i, min_error):
        '''
        #Use this method if you wish to drive Zumi in a straight line or turn.

        #:param max_speed: This number will cap the max speed for the motors
        #:param base_speed: This is the "forward/reverse" speed Zumi will drive at.
        # set to 0 if you wish not to move forward or reverse.
        #:param desired_angle: This will be the angle Zumi will try to drive towards. Can be a positive or negative number
        #:param k_p: constant for proportional controller.
        #:param k_d: constant for derivative controller.
        #:param k_i: constant for integral controller.
        #:param min_error: the minimum error that the motors will stop trying to correct for
        #:return: error
        '''

        # update the angles and grab the Z angle
        angles_now_list = self.update_angles()
        angle_z = angles_now_list[2]

        # calculate error values
        error = desired_angle - angle_z
        self.error_sum = self.error_sum + error
        error_change = error - self.error_past
        self.error_past = error

        # find change in time
        dt = time.time() - self.PID_time_past
        self.PID_time_past = time.time()

        # PID components
        e_p = (k_p * error)
        e_d = (k_d * error_change / dt)
        e_i = (k_i * self.error_sum * dt)
        speed_offset = int(e_p + e_d + e_i)

        # if wihin the error threshold set motors to 0 speed
        if abs(error) < min_error:
            speed_offset = 0

        # input speed for all funtions that use drive_at_angle are capped at +-MAX_USER_SPEED=80
        base_speed = self.clamp(int(base_speed), -1*self.MAX_USER_SPEED, self.MAX_USER_SPEED)

        # make sure motor speeds never go above 127 or below - 127
        final_left_speed = self.clamp(base_speed - speed_offset, -1 * max_speed, max_speed)
        final_right_speed = self.clamp(base_speed + speed_offset, -1 * max_speed, max_speed)

        # send I2C msg to set motors to these values, with no acceleration
        self.control_motors(final_right_speed, final_left_speed, 0)
        # make sure to pause in order not to send to many I2C msg and get I/O Error
        return error

    def turn(self, desired_angle, duration=1.5, max_speed=25, accuracy=1):
        '''
        #:param degrees: the degrees you want to face can be positive or negative
        #:param duration: the number of seconds you want Zumi to "try" to turn
        #:param max_speed=20: maximum speed for turning
        #:param accuracy: the tolerance of +-  degrees.
        #example accuracy of 1 is desired_angle +-1 degrees
        #:return: nothing
        # uses drive_at_angle()
        '''

        # keep the max speed low for turning
        k_p = 0.6
        k_i = 0.001
        k_d = 0.001
        start_time = time.time()
        self.reset_PID()
        speed_now = 0
        accel_period = 1.0
        # acceleration to start motors
        # ----------------------------------------
        if duration >= accel_period:
            while time.time() - start_time < accel_period:
                if speed_now < max_speed:
                    speed_now = speed_now + 1
                self.drive_at_angle(abs(speed_now), 0, desired_angle, k_p, k_d, k_i, accuracy)
        elif duration < accel_period:
            while time.time() - start_time < abs(duration*0.6):
                if speed_now < max_speed:
                    speed_now = speed_now + 1
                self.drive_at_angle(abs(speed_now), 0, desired_angle, k_p, k_d, k_i, accuracy)
        # ----------------------------------------
        while time.time() - start_time < abs(duration):
            self.drive_at_angle(max_speed, 0, desired_angle, k_p, k_d, k_i, accuracy)
        self.stop()

    def go_straight(self, speed, desired_angle, max_speed=127,accuracy=0.8):
        '''
        #:WARNING ONLY WORKS INSIDE A LOOP!
        #:calling it only once does not drive straight
        #:param speed: the forward speed you want Zumi to drive at. Should only input a positive number
        #:param duration: number of seconds you want Zumi to try to drive forward
        #:param desired_angle: the desired angle
        #:return: nothing
        # uses drive_at_angle()
        '''

        k_p = 2.9
        k_i = 0.01
        k_d = 0.05
        self.drive_at_angle(max_speed, abs(speed), desired_angle, k_p, k_d, k_i, accuracy)

    def go_reverse(self, speed, desired_angle,max_speed=127,accuracy=0.8):
        '''
        #:WARNING ONLY WORKS INSIDE A LOOP!calling it only once does not drive straight
        #:param speed: the forward speed you want Zumi to drive at. Should only input a positive number
        #:param duration: number of seconds you want Zumi to try to drive forward
        #:param desired_angle: the desired angle
        #:return: nothing
        # uses drive_at_angle()
        '''

        k_p = 2.9
        k_i = 0.01
        k_d = 0.05
        self.drive_at_angle(max_speed, -1*abs(speed), desired_angle, k_p, k_d, k_i, accuracy)

    def turn_left(self, desired_angle=90, duration=1):
        # will turn left given an angle
        # duration is amount of time you will allow zumi to try to turn
        self.turn(self.read_z_angle() + abs(desired_angle),duration)

    def turn_right(self, desired_angle=-90,duration=1):
        # will turn right given an angle
        # duration is amount of time you will allow zumi to try to turn
        self.turn(self.read_z_angle() + -1*abs(desired_angle),duration)

    def left_u_turn(self, speed=30, step=4, delay=0.02):
        #:param speed: the forward speed you want Zumi to drive at.
        #:param step: the angle step size as it goes from (0 - 180)
        #:param delay: the delay between each angle step
        #:return: nothing
        init_ang_z = self.read_z_angle()
        for i in range(0, 181, step):
            self.go_straight(speed, init_ang_z+i)
            time.sleep(delay)
        self.turn(init_ang_z+180, 1)

    def right_u_turn(self, speed=30, step=4, delay=0.02):
        #:param speed: the forward speed you want Zumi to drive at.
        #:param step: the angle step size as it goes from (0 - 180)
        #:param delay: the delay between each angle step
        #:return: nothing
        init_ang_z = self.read_z_angle()
        for i in range(0, 181, step):
            self.go_straight(speed, init_ang_z-i)
            time.sleep(delay)
        self.turn(init_ang_z-180, 1)

    def j_turn(self, speed=100, step=4, delay=0.005):
        #:param speed: the forward speed you want Zumi to drive at.
        #:param step: the angle step size
        #:param delay: the delay between each angle step
        #:return: nothing
        init_ang_z = self.read_z_angle()
        self.forward(speed, 1, init_ang_z)
        for i in range(0, 120, step):
            self.go_straight(50, init_ang_z-i)
            time.sleep(delay)
        for i in range(120, 180, step):
            self.go_reverse(50, init_ang_z-i)
            time.sleep(delay)
        self.reverse(speed, 1, init_ang_z-180)
        self.stop()

    def parallel_park(self, speed=15, step=1, delay=0.01):
        init_ang_z = self.read_z_angle()
        ang = 55
        for i in range(0, 10):
            self.go_reverse(speed, init_ang_z)
        for i in range(0, ang, step):
            self.go_reverse(speed, init_ang_z+i)
            time.sleep(delay)
        for i in range(0, ang, step):
            self.go_reverse(speed,  init_ang_z+ang-i)
            time.sleep(0)
        self.forward(speed, 0.5, init_ang_z)
        self.stop()

    def circle(self, speed=30, step=2, direction=1, delay=0.02):
        #:param speed: the forward speed you want Zumi to drive at.
        #:param step: the angle step size as it goes from (0 - 360)
        #:param direction: -1 = right , +1 = left
        #:param delay: the delay between each angle step
        #:return: nothing
        direction = self.clamp(direction,-1,1)
        init_ang_z = self.read_z_angle()
        for i in range (0, 361, step):
            self.go_straight(speed,init_ang_z+direction*i)
            time.sleep(delay)
        self.turn(init_ang_z+direction*360, 1)

    def right_circle(self, speed=30, step=2):
        self.circle(speed, step, -1)

    def circle_right(self, speed=30, step=2):
        self.circle(speed, step, -1)

    def left_circle(self, speed=30, step=2):
        self.circle(speed, step, 1)

    def circle_left(self, speed=30, step=2):
        self.circle(speed, step, 1)

    def figure_8(self, speed=30, step=3, delay=0.02):
        #:param speed: the forward speed you want Zumi to drive at.
        #:param step: the angle step size as it goes from (0 - 360)
        #:param delay: the delay between each angle step
        #:return: nothing
        init_ang_z = self.read_z_angle()
        for i in range(0, 361, step):
            self.go_straight(speed, init_ang_z-i)
            time.sleep(delay)
        init_ang_z = self.read_z_angle()
        for i in range(0, 361, step):
            self.go_straight(speed, init_ang_z+i)
            time.sleep(delay)
        self.stop()

    def square(self, speed=40, seconds=1, direction=1):
        # direction will need to be either -1 or 1 to change direction
        direction = self.clamp(direction,-1,1)
        starting_angle = self.read_z_angle()
        self.forward(speed, seconds, starting_angle+direction*0)
        self.forward(speed, seconds, starting_angle+direction*90)
        self.forward(speed, seconds, starting_angle+direction*180)
        self.forward(speed, seconds, starting_angle+direction*270)
        self.turn(starting_angle+direction*360, 1)
        self.stop()

    def square_left(self, speed=40, seconds=1.0):
        direction = 1
        self.square(speed, seconds, direction)

    def square_right(self, speed=40, seconds=1.0):
        direction = -1
        self.square(speed, seconds, direction)

    def rectangle(self, speed=40, seconds=1.0, direction=1, ratio=2):
        #direction will need to be either -1 or 1 to change direction
        direction = self.clamp(direction,-1,1)
        starting_angle = self.read_z_angle()
        self.forward(speed, seconds, starting_angle+direction*0)
        self.forward(speed, seconds*ratio, starting_angle+direction*90)
        self.forward(speed, seconds, starting_angle+direction*180)
        self.forward(speed, seconds*ratio, starting_angle+direction*270)
        self.turn(starting_angle+direction*360, 1)
        self.stop()

    def rectangle_left(self, speed=40, seconds=1.0, ratio=2):
        direction = 1
        self.rectangle(speed, seconds, direction, ratio)

    def rectangle_right(self, speed=40, seconds=1.0, ratio=2):
        direction = -1
        self.rectangle(speed, seconds, direction, ratio)

    def triangle(self, speed=40, seconds=1.5, direction=1):
        #direction will need to be either -1 or 1 to change direction
        direction = self.clamp(direction,-1,1)
        starting_angle = self.read_z_angle()
        self.forward(speed, seconds, starting_angle+direction*0)
        self.forward(speed, seconds, starting_angle+direction*120)
        self.forward(speed, seconds, starting_angle+direction*240)
        self.turn(starting_angle+direction*360, 1)
        self.stop()

    def triangle_left(self, speed=40, seconds=1.5):
        direction = 1
        self.triangle(speed, seconds, direction)

    def triangle_right(self, speed=40, seconds=1.5):
        direction = -1
        self.triangle(speed, seconds, direction)

    def forward(self, speed=40, duration=1.0, desired_angle=123456,accuracy = 1.0):
        '''
        :param speed: the forward speed you want Zumi to drive at. Should only input a positive number
        :param duration: number of seconds you want Zumi to try to drive forward
        :param desired_angle: the desired angle
        :return:
        '''
        if(desired_angle == 123456):
            #if no input find the z angle and go in that direction
            desired_angle = self.read_z_angle()

        max_speed = 127
        k_p = 2.9
        k_i = 0.01
        k_d = 0.05

        self.reset_PID()
        start_time = time.time()
        speed_now = 0
        accel_period = 0.7
        #acceleration to start motors
        #----------------------------------------
        if duration >= accel_period:
            while time.time() - start_time < accel_period:
                if(speed_now < max_speed):
                    speed_now = speed_now + 1
                self.drive_at_angle(60, abs(speed_now), desired_angle, k_p, k_d, k_i, accuracy)
        elif duration < accel_period:
            while time.time() - start_time < abs(duration*0.6):
                if(speed_now < max_speed):
                    speed_now = speed_now + 1
                self.drive_at_angle(60, abs(speed_now), desired_angle, k_p, k_d, k_i, accuracy)
        #-----------------------------------------

        #stay with constant max speed
        while (time.time() - start_time) < abs(duration):
            self.drive_at_angle(max_speed, abs(speed), desired_angle, k_p, k_d, k_i, accuracy)
        self.stop()

    def reverse(self, speed=40, duration=1.0, desired_angle=123456,accuracy = 1.0):
        '''
        :param speed: the forward speed you want Zumi to drive at. Should only input a positive number
        :param duration: number of seconds you want Zumi to try to drive forward
        :param desired_angle: the desired angle
        :return:
        '''
        #123456 is just a dummy number
        if(desired_angle == 123456):
            #if no input find the z angle and go in that direction
            desired_angle = self.read_z_angle()

        max_speed = 127
        k_p = 2.9
        k_i = 0.01
        k_d = 0.01

        self.reset_PID()
        start_time = time.time()
        speed_now= 0
        accel_period = 0.7
        #----------------------------------------
        if duration >= accel_period:
            while time.time() - start_time < accel_period:
                if(speed_now < max_speed):
                    speed_now = speed_now + 1
                self.drive_at_angle(60,-1*abs(speed_now), desired_angle, k_p, k_d, k_i, accuracy)
        elif duration < accel_period:
            while time.time() - start_time < abs(duration*0.6):
                if(speed_now < max_speed):
                    speed_now = speed_now + 1
                self.drive_at_angle(60, -1*abs(speed_now), desired_angle, k_p, k_d, k_i, accuracy)
        #----------------------------------------
        while (time.time() - start_time) < abs(duration):
            self.drive_at_angle(max_speed, -1*abs(speed), desired_angle, k_p, k_d, k_i, accuracy)
        self.stop()

    def send_LED_byte(self):

        LED_byte = self.LED_user_pattern
        for i in range(5):
            if _bit_read(self.LED_signals, (4-i)) ==1:
                j = 4-i
                if j == 0:
                    LED_byte = 0b11110000
                elif j == 1:
                    # 1x1x 0x0x - Left turn signal
                    LED_byte = _bit_clear(LED_byte, 1)
                    LED_byte = _bit_clear(LED_byte, 3)
                    LED_byte = _bit_set(LED_byte, 5)
                    LED_byte = _bit_set(LED_byte, 7)
                elif j == 2:
                    # x1x1 x0x0 - Right turn signal
                    LED_byte = _bit_clear(LED_byte, 0)
                    LED_byte = _bit_clear(LED_byte, 2)
                    LED_byte = _bit_set(LED_byte, 4)
                    LED_byte = _bit_set(LED_byte, 6)
                elif j == 3:
                    # xx11 xx11 - Brake lights
                    LED_byte = _bit_set(LED_byte, 0)
                    LED_byte = _bit_set(LED_byte, 1)
                    LED_byte = _bit_set(LED_byte, 4)
                    LED_byte = _bit_set(LED_byte, 5)
                else:
                    # 11xx 11xx - Headlights
                    LED_byte = _bit_set(LED_byte, 2)
                    LED_byte = _bit_set(LED_byte, 3)
                    LED_byte = _bit_set(LED_byte, 6)
                    LED_byte = _bit_set(LED_byte, 7)

        try:
            self.bus.write_byte_data(Device.Arduino, 0b10000000, LED_byte)
            time.sleep(self.MIN_I2C_DELAY)
        except IOError:
            time.sleep(0.05)
            self.stop()

    def stop(self, acceleration=0):
        """
        Zumi stops
        acceleration parameter no longer
        does anything in new version
        """
        try:
            self.is_moving = 0
            self.bus.write_byte(Device.Arduino, _create_command(CommandType.Motor, MotorCommand.Stop, acceleration))
            time.sleep(self.MIN_I2C_DELAY)
        except ValueOutOfRangeError as e:
            print(e)
            raise

    def hard_brake(self):
        """
        Zumi stops immediately
        Args: None
        """
        self.stop()

    def control_motors(self, left, right, acceleration=0):
        """
        #Sets the speed of each motor. The changes take place immediately;
        #if one were to call control_motors(30, 30) while
        #Zumi was stopped, Zumi would immediately drive forward
        #Args: 2 integers, from -126 to 127, for the left and right motors.
        #on the arduino side control_motors(-126,-126) will result
        #in analog write 176/255 in reverse for both motors. and control_motors(127,127) -> (177,177)
        #Acceleration parameter is no longer used it will be removed in later versions
        """
        left = self.clamp( left, -126, 127)
        right = self.clamp( right, -126, 127)
        left = _convert_to_arduino_speed(left)
        right = _convert_to_arduino_speed(right)
        try:
            self.bus.write_i2c_block_data(Device.Arduino, _create_command(CommandType.Motor,MotorCommand.SetMotorsToSpeed, acceleration),[left, right])
            time.sleep(self.MIN_I2C_DELAY)
        except IOError:
            time.sleep(self.MIN_I2C_DELAY)
            self.stop()



    def get_all_arduino_data(self):
        """
        #Gets the whole message sent by the arduino board.
        """
        i2c_message_length = 10
        try:
            data = self.bus.read_i2c_block_data(Device.Arduino, 192, i2c_message_length)
            time.sleep(self.MIN_I2C_DELAY)
        except IOError:
            return self.get_all_arduino_data()
        return data

    def get_all_IR_data(self):
        """
        #Get the readings from all 6 IR sensors. Returns a list with a length of 6, with values between 0 and 255.
        #Args: None
        """
        try:
            data = self.bus.read_i2c_block_data(Device.Arduino, 192, 6)
            time.sleep(self.MIN_I2C_DELAY)
        except IOError:
            return self.get_all_IR_data()
        return data

    def get_IR_data(self, ir_sensor_index):
        """
        #Get the reading from a single IR sensor. Returns a value between 0 and 255.
        #Args: 1 integer, from 0 to 5. Each number corresponds to a different IR sensor:

        #0 / IR.FRONT_RIGHT = Front right sensor
        #1 / IR.BOTTOM_RIGHT = Bottom right sensor
        #2 / IR.BACK_RIGHT = Back right sensor
        #3 / IR.BOTTOM_LEFT = Bottom left sensor
        #4 / IR.BACK_LEFT = Back left sensor
        #5 / IR.FRONT_LEFT = Front left sensor
        """

        try:
            if not _is_in_range(ir_sensor_index, 0, 5):
                raise ValueOutOfRangeError("IR sensor index must be a value between 0 and 5")
            data = self.bus.read_i2c_block_data(Device.Arduino, 192, 6)
        except IOError:
            return self.get_IR_data(ir_sensor_index)
        except ValueOutOfRangeError as e:
            print(e)
            raise
        return data[ir_sensor_index]

    def get_battery_voltage(self):
        """
        #Get the reading from battery level
        #Only works in Zumi Arduino Firmware 1.00 and above
        #The code also makes sure
        #that the Arduino board is not using the I2C channel
        """
        if(GPIO.input(4) == 0): #check arduino is ok to send i2c
            try:
                data = self.bus.read_i2c_block_data(Device.Arduino, 192, 10)
                time.sleep(self.MIN_I2C_DELAY)
                # On the arduino side of things the voltage
                # measured on the battery will always be
                # below 4.2 volts, to be sent over its
                # multiplied by 15, then here divided by 15
                if data[8]==255: #this indicates the arduino firmware version is 1.00
                    voltage = data[6]/15
                else: #then the firmware may be above Arduino Zumi 1.10 firmware
                    voltage = (data[6]/58)+1
                    #arduino sends over outputArray[6] = (batteryVoltage - 1) * 58;

            except IOError:
                voltage = -1
        else:
            voltage = -1
        #data[6]/60 = voltage
        return voltage

    def get_battery_percent(self):
        try:
            usb_voltage_detected = 1.7
            min_battery_voltage = 3.45
            max_battery_voltage = 4.14

            battery_voltage = self.get_battery_voltage()

            if battery_voltage > min_battery_voltage:
                battery_percentage = (battery_voltage - min_battery_voltage) / (
                            max_battery_voltage - min_battery_voltage) * 100

            if battery_percentage > 100:
                battery_percentage = 100
            elif battery_percentage < 0:
                battery_percentage = 0
        except:
            battery_percentage = -1

        return int(battery_percentage)


    def get_vcc_voltage(self):
        """
        #Get the reading from 5v vcc level
        #Only works in Zumi Arduino Firmware 1.10 and above
        """
        try:
            data = self.bus.read_i2c_block_data(Device.Arduino, 192, 10)
            time.sleep(self.MIN_I2C_DELAY)

            if data[8]==255: #this indicates the arduino firmware version is 1.00
                voltage = 0.00
            else: #then the firmware may be above Arduino Zumi 1.10 firmware
                voltage = (data[7]/48)+3
                #zumi board sends over outputArray[7] = (checkInputVCC() - 3) * 48;

        except IOError:
            voltage = -1

        return voltage



    def get_board_firmware_version(self):
        """
        #Get the arduino firmware version
        #works on Zumi Arduino Firmware 1.00 and above
        """
        if(GPIO.input(4) == 0): #check arduino is ok to send i2c
            try:
                data = self.bus.read_i2c_block_data(Device.Arduino, 192, 10)
                time.sleep(self.MIN_I2C_DELAY)
                if data[8] == 255:#if the message is empty then it most likely is an old board version
                    version = 1.00
                else:
                    version = (data[8]/10)+1.1

            except IOError:
                version = -1
        else:
            version = -1
            return version


    def all_lights_on(self):
        """
        Turns all lights on
        Args: None
        """
        self.LED_user_pattern = 255
        self.LED_signals = 0
        self.send_LED_byte()

    def all_lights_off(self):
        """
        Turns all lights off
        Args: None
        """
        self.LED_user_pattern = 0
        self.LED_signals = 0
        self.send_LED_byte()

    def headlights_on(self):
        """
        Turn the front lights on
        Args: None
        """
        self.LED_signals = _bit_set(self.LED_signals, 4)
        self.send_LED_byte()

    def headlights_off(self):
        """
        Turns the front lights off
        Args: None
        """
        self.LED_signals = _bit_clear(self.LED_signals, 4)
        self.send_LED_byte()

    def brake_lights_on(self):
        """
        Turn the back lights on
        Args: None
        """
        self.LED_signals = _bit_set(self.LED_signals, 3)
        self.send_LED_byte()

    def brake_lights_off(self):
        """
        Turns the back lights off
        Args: None
        """
        self.LED_signals = _bit_clear(self.LED_signals, 3)
        self.send_LED_byte()

    def hazard_lights_on(self):
        """
        Flashes all lights
        Args: None
        """
        self.LED_signals = _bit_set(self.LED_signals, 0)
        self.send_LED_byte()

    def hazard_lights_off(self):
        """
        Turns off all lights
        Args: None
        """
        self.LED_signals = _bit_clear(self.LED_signals, 0)
        self.send_LED_byte()

    def signal_left_on(self):
        """
        Flashes the left-front and left-back lights
        Args: None
        """
        self.LED_signals = _bit_clear(self.LED_signals, 2)
        self.LED_signals = _bit_set(self.LED_signals, 1)
        self.send_LED_byte()

    def signal_left_off(self):
        """
        Turns off all lights
        Args: None
        """
        self.LED_signals = _bit_clear(self.LED_signals, 1)
        self.send_LED_byte()

    def signal_right_on(self):
        """
        Flashes the right-front and right-back lights
        Args: None
        """
        self.LED_signals = _bit_clear(self.LED_signals, 1)
        self.LED_signals = _bit_set(self.LED_signals, 2)
        self.send_LED_byte()

    def signal_right_off(self):
        """
        Turns off all lights
        Args: None
        """
        self.LED_signals = _bit_clear(self.LED_signals, 2)
        self.send_LED_byte()

    def led_on(self, index):
        try:
            if not _is_in_range(index, 0, 3):
                raise ValueOutOfRangeError("Expected an index between 0 and 3, but got " + str(index) + "instead.")
            self.LED_user_pattern = _bit_set(self.LED_user_pattern, 3 - index)
            self.LED_user_pattern = _bit_set(self.LED_user_pattern, 3 - index + 4)
            self.send_LED_byte()
        except ValueOutOfRangeError as e:
            print(e)
            raise

    def led_off(self, index):
        try:
            if not _is_in_range(index, 0, 3):
                raise ValueOutOfRangeError("Expected an index between 0 and 3, but got " + str(index) + "instead.")
            self.LED_user_pattern = _bit_clear(self.LED_user_pattern, 3 - index)
            self.LED_user_pattern = _bit_clear(self.LED_user_pattern, 3 - index + 4)
            self.send_LED_byte()
        except ValueOutOfRangeError as e:
            print(e)
            raise

    def led_flash(self, index):
        try:
            if not _is_in_range(index, 0, 3):
                raise ValueOutOfRangeError("Expected an index between 0 and 3, but got " + str(index) + "instead.")
            self.LED_user_pattern = _bit_set(self.LED_user_pattern, 3 - index)
            self.LED_user_pattern = _bit_clear(self.LED_user_pattern, 3 - index + 4)
            self.send_LED_byte()
        except ValueOutOfRangeError as e:
            print(e)
            raise


    def play_note(self, note_type, note_duration=500):
        """
        Play a note, from C2 - B6.
        Args: 2 integers. The first is from 0-60, and the second is from 0 to 2500. The first integer determines the
        note being played. 0 is a rest. Any number after 0 will play a note, starting from C2, and going all
        the way up to B6. The second integer is the duration of the note, in milliseconds. While any integer in this
        range can be entered, the integer will be rounded down to a multiple of 10.
        """

        note_type = self.clamp(int(note_type),0,60)
        note_duration = self.clamp(int(note_duration),0,2500)
        #TODO Avoid error when rounded down to 0 , plays forever
        try:
            command = CommandType.Audio
            command = command << 6
            command |= note_type
            self.bus.write_byte_data(Device.Arduino, command, int(note_duration/10))
            time.sleep(note_duration/1000+0.015)
        except ValueOutOfRangeError as e:
            print(e)
            raise

    # The following methods work with Zumi Arduino Firmware 1.00 and above
    def send_byte_to_arduino(self,byte):
        arduino_address = 0x04
        self.bus.write_byte(arduino_address, byte)
        time.sleep(self.MIN_I2C_DELAY)

    def z_led_on(self):
        self.send_byte_to_arduino(193)

    def z_led_off(self):
        self.send_byte_to_arduino(194)

    def sound_on(self):
        self.send_byte_to_arduino(195)

    def sound_off(self):
        self.send_byte_to_arduino(196)

    def enable_individual_led_control(self):
        self.send_byte_to_arduino(197)

    def disable_individual_led_control(self):
        self.send_byte_to_arduino(198)

    def front_left_led_on(self):
        self.send_byte_to_arduino(199)

    def front_left_led_off(self):
        self.send_byte_to_arduino(200)

    def front_right_led_on(self):
        self.send_byte_to_arduino(201)

    def front_right_led_off(self):
        self.send_byte_to_arduino(202)

    def back_left_led_on(self):
        self.send_byte_to_arduino(203)

    def back_left_led_off(self):
        self.send_byte_to_arduino(204)

    def back_right_led_on(self):
        self.send_byte_to_arduino(205)

    def back_right_led_off(self):
        self.send_byte_to_arduino(206)

    def IR_emitter_on(self):
        self.send_byte_to_arduino(207)

    def IR_emitter_off(self):
        self.send_byte_to_arduino(208)

    def IR_readings_on(self):
        self.send_byte_to_arduino(209)

    def IR_readings_off(self):
        self.send_byte_to_arduino(210)



if __name__ == '__main__':
    zumi = Zumi()

    print("battery voltage = ",zumi.get_battery_voltage())
    zumi.headlights_on()
    zumi.play_note(100,100)
