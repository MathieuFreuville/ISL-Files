

from math import atan, sqrt, pi
import board
import busio
import time

class MPU6050:

    # Global Variables
    GRAVITY = 9.80665
    i2c = None
    address = None
    Local_Acc_scale= 0
    Local_Acc_scale_modifier = 0
    Local_Gyr_scale= 0
    Local_Gyr_scale_modifier = 0

    Acc_Angle_X = 0
    Acc_Angle_Y = 0
    Acc_Angle_Z = 0
    Gyr_Angle_X = 0
    Gyr_Angle_Y = 0
    Gyr_Angle_Z = 0
    Gyr_Rate_X = 0
    Gyr_Rate_Y = 0
    Gyr_Rate_Z = 0

    Comp_Calculation = False
    Comp_Gyr_factor = None
    Comp_Filter_X = None
    Comp_Filter_Y = None
    Comp_Filter_Z = None
    Comp_Angle_X = None
    Comp_Angle_Y = None
    Comp_Angle_Z = None

    Gyr_X_CalibValue = 0
    Gyr_Y_CalibValue = 0
    Gyr_Z_CalibValue = 0
    Acc_X_CalibValue = 0
    Acc_Y_CalibValue = 0
    Acc_Z_CalibValue = 0

    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    FILTER_BW_256=0x00
    FILTER_BW_188=0x01
    FILTER_BW_98=0x02
    FILTER_BW_42=0x03
    FILTER_BW_20=0x04
    FILTER_BW_10=0x05
    FILTER_BW_5=0x06

    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    TEMP_OUT0 = 0x41

    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47

    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B
    MPU_CONFIG = 0x1A

    def __init__(self, address = 0x68, compl_filter = False):

        self.address = address
        self.Comp_Calculation = compl_filter        # Enable complementary filter calculations
        if self.Comp_Calculation:
            self.Comp_Gyr_factor = 0.96             # set the default ratio to 0.96 / 0.04
            self.Comp_Acc_factor = 0.04
        # Wake up the MPU-6050 since it starts in sleep mode
        self.i2c = busio.I2C(board.SCL, board.SDA)
        # print("I2C devices:", [hex(i) for i in self.i2c.scan()])
        wake_up_buffer = bytearray(2)
        wake_up_buffer[0] = self.PWR_MGMT_1
        wake_up_buffer[1] = 0x00
        self.i2c.writeto( self.address, wake_up_buffer)
        time.sleep(0.5)

        # Get sensor temperature
        print('MPU6050 Sensor Temperature : ' + str(self.get_temp()))

        #initialize Accelerometers Angles

    def set_complementaryfilter_ratios (self, gyro):
        self.Comp_Gyr_factor = gyro
        self.Comp_Acc_factor = round( (1-gyro), 3)

    def write_i2c_register(self, register, value):

        buffer = bytearray(2)
        buffer[0] = register
        buffer[1] = value

        self.i2c.writeto(self.address, buffer)

    def read_i2c_byte(self, register):

        value = bytearray(1)
        self.i2c.writeto(self.address, bytes([register]))
        self.i2c.readfrom_into(self.address, value)
        return value

    def read_i2c_word(self, register):

        # Read the data from the 2 registers starting at value "register"
        byte_array = bytearray(2)
        self.i2c.writeto(self.address, bytes([register]))
        self.i2c.readfrom_into(self.address, byte_array)
        word = byte_array[0] << 8 | byte_array[1]
        if word >= 0x8000:
            return -((65535 - word) + 1)
        else:
            return word

    def read_i2c_3words(self, start_register):

        # Read the data from the 2 registers starting at value "register"
        byte_array = bytearray(6)
        self.i2c.writeto(self.address, bytes([start_register]))
        self.i2c.readfrom_into(self.address, byte_array)

        words = [byte_array[0] << 8 | byte_array[1], byte_array[2] << 8 | byte_array[3],
                 byte_array[4] << 8 | byte_array[5]]

        for i in range(len(words)):
            if words[i] >= 0x8000:
                words[i] = -((65535 - words[i]) + 1)
        return words

    def read_i2c_6words(self, start_register):

        # Read the data from the 6 registers starting at value "register"
        # Only used for reading all gyros + All acceleros at once
        byte_array = bytearray(14)
        self.i2c.writeto(self.address, bytes([start_register]))
        self.i2c.readfrom_into(self.address, byte_array)

        words = [byte_array[0] << 8 | byte_array[1], byte_array[2] << 8 | byte_array[3],
                 byte_array[4] << 8 | byte_array[5], byte_array[6] << 8 | byte_array[7],
                 byte_array[8] << 8 | byte_array[9], byte_array[10] << 8 | byte_array[11],
                 byte_array[12] << 8 | byte_array[13]]
        # Remove temperature fom the list of words
        words.pop(3)

        for i in range(len(words)):
            if words[i] >= 0x8000:
                words[i] = -((65535 - words[i]) + 1)
        return words

    def set_accel_range(self, accel_range):

        # First change it to 0x00 to make sure we write the correct value later
        self.write_i2c_register(self.ACCEL_CONFIG, 0x00)
        time.sleep(0.5)

        # Write the new range to the ACCEL_CONFIG register
        self.write_i2c_register(self.ACCEL_CONFIG, accel_range)
        time.sleep(0.5)

        # Verify the range by re reading the register
        read_register = self.read_i2c_byte(self.ACCEL_CONFIG)
        if not int.from_bytes(read_register) == accel_range:
            print('Acc range has not been properly written. Written: ' + str(accel_range) + ' read : ' + str(read_register))
            print('Acc range has not been properly written. Written: ' + str(accel_range) + ' read : ' + str(read_register))

        # Save the range in a local variable
        self.Local_Acc_scale = accel_range

        if self.Local_Acc_scale == self.ACCEL_RANGE_2G:
            self.Local_Acc_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
        elif self.Local_Acc_scale == self.ACCEL_RANGE_4G:
            self.Local_Acc_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
        elif self.Local_Acc_scale == self.ACCEL_RANGE_8G:
            self.Local_Acc_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
        elif self.Local_Acc_scale == self.ACCEL_RANGE_16G :
            self.Local_Acc_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
        else:
            print("Unknown range - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
            self.Local_Acc_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
            self.write_i2c_register(self.ACCEL_CONFIG, self.ACCEL_RANGE_2G)

        print('Accelerometer range as read from register = ' + str(self.read_accel_range()))

    def set_gyro_range(self, gyro_range):

        # First change it to 0x00 to make sure we write the correct value later
        self.write_i2c_register(self.GYRO_CONFIG, 0x00)
        time.sleep(0.5)

        # Write the new range to the Gyro_CONFIG register
        self.write_i2c_register(self.GYRO_CONFIG, gyro_range)
        time.sleep(0.5)

        # Verify the range by re reading the register
        read_register = self.read_i2c_byte(self.GYRO_CONFIG)
        if not int.from_bytes(read_register) == gyro_range:
            print('Gyro range has not been properly written. Written: ' + str(gyro_range) + ' read : ' + str(read_register))
            print('Gyro range has not been properly written. Written: ' + str(gyro_range) + ' read : ' + str(read_register))

        # Save the range in a local variable
        self.Local_Gyr_scale = gyro_range

        if self.Local_Gyr_scale == self.GYRO_RANGE_250DEG:
            self.Local_Gyr_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
        elif self.Local_Gyr_scale == self.GYRO_RANGE_500DEG:
            self.Local_Gyr_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
        elif self.Local_Gyr_scale == self.GYRO_RANGE_1000DEG:
            self.Local_Gyr_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
        elif self.Local_Gyr_scale == self.GYRO_RANGE_2000DEG :
            self.Local_Gyr_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
        else:
            print("Unknown range - Gyro_scale_modifier set to self.GYRO_SCALE_MODIFIER_250DEG")
            self.Local_Gyr_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
            self.write_i2c_register(self.GYRO_CONFIG, self.GYRO_RANGE_250DEG)
        print('Gyro range as read from register = ' + str(self.read_gyro_range()))

    def set_filter_range(self, filter_range = FILTER_BW_256):

        # Keep the current EXT_SYNC_SET configuration in bits 3, 4, 5 in the MPU_CONFIG register
        raw_data = self.read_i2c_byte(self.MPU_CONFIG)
        EXT_SYNC_SET = raw_data[0] & 0b00111000
        self.write_i2c_register(self.MPU_CONFIG, EXT_SYNC_SET | filter_range)

        # raw_data = self.read_i2c_byte(self.MPU_CONFIG)
        # print(raw_data)

        print('Filter cutoff frequency as read from register = ' + str(self.read_filter_range()))

    def initialize_gyros_angles(self):

        angles= [self.Acc_X_CalibValue / self.Local_Acc_scale_modifier,
                 self.Acc_Y_CalibValue / self.Local_Acc_scale_modifier,
                 self.Acc_Z_CalibValue / self.Local_Acc_scale_modifier]
        self.Acc_Angle_X = atan( angles[1] / sqrt( pow(angles[0],2) + pow(angles[2],2))) * 180/pi        # Roll around X axis
        self.Acc_Angle_Y  = atan(-angles[0] / sqrt( pow(angles[1],2) + pow(angles[2],2))) * 180/pi        # Pitch around Y axis
        self.Acc_Angle_Z  = atan( angles[2] / sqrt( pow(angles[0],2) + pow(angles[1],2))) * 180/pi        # Yaw around z axis

        self.Gyr_Angle_X = self.Acc_Angle_X
        self.Gyr_Angle_Y = self.Acc_Angle_Y
        self.Gyr_Angle_Z = self.Acc_Angle_Z
        self.Comp_Angle_X = self.Acc_Angle_X
        self.Comp_Angle_Y = self.Acc_Angle_Y
        self.Comp_Angle_Z = self.Acc_Angle_Z

        '''acc_angles = self.get_accel_angles()
        self.Gyr_Angle_X = acc_angles['x']
        self.Gyr_Angle_Y = acc_angles['y']
        self.Gyr_Angle_Z = acc_angles['z']
        self.Comp_Angle_X = acc_angles['x']
        self.Comp_Angle_Y = acc_angles['y']
        self.Comp_Angle_Z = acc_angles['z']'''

    def read_local_ranges(self):

        if self.Local_Acc_scale == self.ACCEL_RANGE_2G:     acc = 2
        elif self.Local_Acc_scale == self.ACCEL_RANGE_4G:   acc = 4
        elif self.Local_Acc_scale == self.ACCEL_RANGE_8G:   acc = 8
        elif self.Local_Acc_scale == self.ACCEL_RANGE_16G:  acc = 16
        else: return -1

        if self.Local_Gyr_scale == self.GYRO_RANGE_250DEG: gyr = 250
        elif self.Local_Gyr_scale == self.GYRO_RANGE_500DEG: gyr = 500
        elif self.Local_Gyr_scale == self.GYRO_RANGE_1000DEG: gyr = 1000
        elif self.Local_Gyr_scale == self.GYRO_RANGE_2000DEG: gyr = 2000
        else: return -1

        print('Acc (g): ' + str(acc) + ' Gyr : ' + str(gyr))

    def read_local_modifiers(self):

        if self.Local_Acc_scale_modifier == self.ACCEL_SCALE_MODIFIER_2G: acc = 2
        elif self.Local_Acc_scale_modifier == self.ACCEL_SCALE_MODIFIER_4G: acc = 4
        elif self.Local_Acc_scale_modifier == self.ACCEL_SCALE_MODIFIER_8G: acc = 8
        elif self.Local_Acc_scale_modifier == self.ACCEL_SCALE_MODIFIER_16G: acc = 16
        else:  return -1

        if self.Local_Gyr_scale_modifier == self.GYRO_RANGE_250DEG: gyr = 250
        elif self.Local_Gyr_scale_modifier == self.GYRO_RANGE_500DEG:  gyr = 500
        elif self.Local_Gyr_scale_modifier == self.GYRO_RANGE_1000DEG:gyr = 1000
        elif self.Local_Gyr_scale_modifier == self.GYRO_RANGE_2000DEG: gyr = 2000
        else: return -1

        print('Acc (g): ' + str(acc) + ' Gyr : ' + str(gyr))

    def read_accel_range(self, raw=False):

        raw_data = self.read_i2c_byte(self.ACCEL_CONFIG)

        if raw is True: return raw_data
        elif raw is False:
            if raw_data[0] == self.ACCEL_RANGE_2G: return 2
            elif raw_data[0] == self.ACCEL_RANGE_4G: return 4
            elif raw_data[0] == self.ACCEL_RANGE_8G: return 8
            elif raw_data[0] == self.ACCEL_RANGE_16G:return 16
            else: return -1

    def read_gyro_range(self, raw=False):

        raw_data = bytearray(1)
        raw_data = self.read_i2c_byte(self.GYRO_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data[0] == self.GYRO_RANGE_250DEG:
                return 250
            elif raw_data[0] == self.GYRO_RANGE_500DEG:
                return 500
            elif raw_data[0] == self.GYRO_RANGE_1000DEG:
                return 1000
            elif raw_data[0] == self.GYRO_RANGE_2000DEG:
                return 2000
            else:
                return -1

    def read_filter_range(self, raw=False):

        raw_data = bytearray(1)
        raw_data = self.read_i2c_byte(self.MPU_CONFIG)

        # Digital lowpass filter cutoff value is stored in bits 2-0 of the MPU_CONFIG register
        raw_data[0] = raw_data[0] & 0b00000111

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data[0] == self.FILTER_BW_5:
                return 5
            elif raw_data[0] == self.FILTER_BW_10:
                return 10
            elif raw_data[0] == self.FILTER_BW_20:
                return 20
            elif raw_data[0] == self.FILTER_BW_42:
                return 42
            elif raw_data[0] == self.FILTER_BW_98:
                return 98
            elif raw_data[0] == self.FILTER_BW_188:
                return 188
            elif raw_data[0] == self.FILTER_BW_256:
                return 256
            else:
                return -1

    def get_temp(self):

        raw_temp = self.read_i2c_word(self.TEMP_OUT0)

        # Get the actual temperature using the formule given in the
        # MPU-6050 Register Map and Descriptions revision 4.2, page 30
        actual_temp = (raw_temp / 340.0) + 36.53
        return actual_temp

    def get_accel_data(self, g=False):

        # Get the accelerometers vectors in m/s2 or in g
        values = self.read_i2c_3words(self.ACCEL_XOUT0)

        values[0] /= self.Local_Acc_scale_modifier
        values[1] /= self.Local_Acc_scale_modifier
        values[2] /= self.Local_Acc_scale_modifier

        if g is True:
            return {'x': values[0], 'y': values[1], 'z': values[2]}
        elif g is False:
            values[0] *= self.GRAVITY
            values[1] *= self.GRAVITY
            values[2] *= self.GRAVITY
            return {'x': values[0], 'y': values[1], 'z': values[2]}

    def get_accel_angles(self):

        angles = self.read_i2c_3words(self.ACCEL_XOUT0)
        angles[0] /= self.Local_Acc_scale_modifier
        angles[1] /= self.Local_Acc_scale_modifier
        angles[2] /= self.Local_Acc_scale_modifier
        self.Acc_Angle_X = atan( angles[1] / sqrt( pow(angles[0],2) + pow(angles[2],2))) * 180/pi        # Roll around X axis
        self.Acc_Angle_Y  = atan(-angles[0] / sqrt( pow(angles[1],2) + pow(angles[2],2))) * 180/pi        # Pitch around Y axis
        self.Acc_Angle_Z  = atan( angles[2] / sqrt( pow(angles[0],2) + pow(angles[1],2))) * 180/pi        # Yaw around z axis

        return {'x': self.Acc_Angle_X , 'y': self.Acc_Angle_Y , 'z': self.Acc_Angle_Z }

    def get_gyro_raw_data(self):

        gyros = self.read_i2c_3words(self.GYRO_XOUT0)
        return {'x': gyros[0], 'y': gyros[1], 'z': gyros[2]}

    def get_gyro_raw_corrected_data(self):

        gyros = self.read_i2c_3words(self.GYRO_XOUT0)
        return {'x': gyros[0] - self.Gyr_X_CalibValue, 'y': gyros[1]- self.Gyr_Y_CalibValue, 'z': gyros[2]- self.Gyr_Z_CalibValue}

    def get_gyro_angular_speed(self):

        gyros = self.read_i2c_3words(self.GYRO_XOUT0)
        gyros[0] -= self.Gyr_X_CalibValue
        gyros[1] -= self.Gyr_Y_CalibValue
        gyros[2] -= self.Gyr_Z_CalibValue

        gyros[0] = gyros[0] /  self.Local_Gyr_scale_modifier
        gyros[1] = gyros[1] /  self.Local_Gyr_scale_modifier
        gyros[2] = gyros[2] /  self.Local_Gyr_scale_modifier

        return {'x': gyros[0], 'y': gyros[1], 'z': gyros[2]}

    def get_gyro_angular_displacement(self, dt):

        gyros = self.read_i2c_3words(self.GYRO_XOUT0)

        gyros[0] = dt * (gyros[0] - self.Gyr_X_CalibValue) /  self.Local_Gyr_scale_modifier
        gyros[1] = dt * (gyros[1] - self.Gyr_Y_CalibValue) /  self.Local_Gyr_scale_modifier
        gyros[2] = dt * (gyros[2] - self.Gyr_Z_CalibValue) /  self.Local_Gyr_scale_modifier

        return {'x': gyros[0], 'y': gyros[1], 'z': gyros[2]}

    def compute_gyro_angles(self, dt):

        gyros = self.read_i2c_3words(self.GYRO_XOUT0)

        gyros[0] = dt * (gyros[0] - self.Gyr_X_CalibValue) /  self.Local_Gyr_scale_modifier
        gyros[1] = dt * (gyros[1] - self.Gyr_Y_CalibValue) /  self.Local_Gyr_scale_modifier
        gyros[2] = dt * (gyros[2] - self.Gyr_Z_CalibValue) /  self.Local_Gyr_scale_modifier

        self.Gyr_Angle_X += gyros[0]
        self.Gyr_Angle_Y += gyros[1]
        self.Gyr_Angle_Z += gyros[2]

        return {'x': self.Gyr_Angle_X , 'y': self.Gyr_Angle_Y , 'z': self.Gyr_Angle_Z }

    def calibrate_gyros(self):

        # Zeroing correction occurs on raw data
        # Reinitialize corrections
        self.Gyr_X_CalibValue = 0
        self.Gyr_Y_CalibValue = 0
        self.Gyr_Z_CalibValue = 0

        # Temp buffer for calculating
        buf_x = 0
        buf_y = 0
        buf_z = 0
        i = 0

        while i < 500:
            buf_x += self.read_i2c_word(self.GYRO_XOUT0)
            buf_y += self.read_i2c_word(self.GYRO_YOUT0)
            buf_z += self.read_i2c_word(self.GYRO_ZOUT0)
            time.sleep(0.01)
            i += 1
        self.Gyr_X_CalibValue = buf_x / (i-1)
        self.Gyr_Y_CalibValue = buf_y / (i-1)
        self.Gyr_Z_CalibValue = buf_z / (i-1)
        print('Gyro. raw calib. values: X= ' + str(self.Gyr_X_CalibValue) + ' Y= ' + str(self.Gyr_Y_CalibValue) + ' Z= ' + str(self.Gyr_Z_CalibValue) )

    def calibrate_acceleros(self):

        # Zeroing correction occurs on raw data
        # Reinitialize corrections
        self.Acc_X_CalibValue = 0
        self.Acc_Y_CalibValue = 0
        self.Acc_Z_CalibValue = 0

        # Temp buffer for calculating
        buf_x = 0
        buf_y = 0
        buf_z = 0
        i = 0

        while i < 500:
            buf_x += self.read_i2c_word(self.ACCEL_XOUT0)
            buf_y += self.read_i2c_word(self.ACCEL_YOUT0)
            buf_z += self.read_i2c_word(self.ACCEL_ZOUT0)
            time.sleep(0.01)
            i += 1
        self.Acc_X_CalibValue = buf_x / (i-1)
        self.Acc_Y_CalibValue = buf_y / (i-1)
        self.Acc_Z_CalibValue = buf_z / (i-1)
        print('Acc. raw calib. values: X= ' + str(self.Acc_X_CalibValue) + ' Y= ' + str(self.Acc_Y_CalibValue) + ' Z= ' + str(self.Acc_Z_CalibValue) )

    def get_6axis(self, dt):

        byte_array = bytearray(14)
        self.i2c.writeto(self.address, bytes([self.ACCEL_XOUT0]))
        self.i2c.readfrom_into(self.address, byte_array)

        words = [byte_array[0] << 8 | byte_array[1], byte_array[2] << 8 | byte_array[3],
                 byte_array[4] << 8 | byte_array[5], byte_array[6] << 8 | byte_array[7],
                 byte_array[8] << 8 | byte_array[9], byte_array[10] << 8 | byte_array[11],
                 byte_array[12] << 8 | byte_array[13]]

        # Remove temperature fom the list of words
        words.pop(3)

        for i in range(len(words)):
            if words[i] >= 0x8000:
                words[i] = -((65535 - words[i]) + 1)

        # Compute accelerometers angles
        words[0] /= self.Local_Acc_scale_modifier
        words[1] /= self.Local_Acc_scale_modifier
        words[2] /= self.Local_Acc_scale_modifier
        self.Acc_Angle_X = atan( words[1] / sqrt( pow(words[0],2) + pow(words[2],2))) * 180/pi         # Pitch around X axis
        self.Acc_Angle_Y  = atan(-words[0] / sqrt( pow(words[1],2) + pow(words[2],2))) * 180/pi        # Roll around Y axis
        self.Acc_Angle_Z  = atan( words[2] / sqrt( pow(words[0],2) + pow(words[1],2))) * 180/pi        # Yaw around z axis

        # Compute gyroscopes rates
        self.Gyr_Rate_X = (words[3] - self.Gyr_X_CalibValue) /  self.Local_Gyr_scale_modifier
        self.Gyr_Rate_Y = (words[4] - self.Gyr_Y_CalibValue) /  self.Local_Gyr_scale_modifier
        self.Gyr_Rate_Z = (words[5] - self.Gyr_Z_CalibValue) /  self.Local_Gyr_scale_modifier

        # Compute gyroscopes variations
        words[3] = dt * (words[3] - self.Gyr_X_CalibValue) /  self.Local_Gyr_scale_modifier       # Roll around X axis
        words[4] = dt * (words[4] - self.Gyr_Y_CalibValue) /  self.Local_Gyr_scale_modifier       # Pitch around Y axis
        words[5] = dt * (words[5] - self.Gyr_Z_CalibValue) /  self.Local_Gyr_scale_modifier       # Yaw arond Z axis

        # Compute gyroscopes angles
        self.Gyr_Angle_X += words[3]
        self.Gyr_Angle_Y += words[4]
        self.Gyr_Angle_Z += words[5]

        if self.Comp_Calculation :
            # Compute complementary angles
            self.Comp_Angle_X += words[3]
            self.Comp_Angle_Y += words[4]
            self.Comp_Angle_Z += words[5]
            self.Comp_Angle_X = self.Comp_Gyr_factor * self.Comp_Angle_X + self.Comp_Acc_factor * self.Acc_Angle_X
            self.Comp_Angle_Y = self.Comp_Gyr_factor * self.Comp_Angle_Y + self.Comp_Acc_factor * self.Acc_Angle_Y
            self.Comp_Angle_Z = self.Comp_Gyr_factor * self.Comp_Angle_Z + self.Comp_Acc_factor * self.Acc_Angle_Z

        return {'xg': self.Gyr_Angle_X ,'xa': self.Acc_Angle_X,
                'yg': self.Gyr_Angle_Y ,'ya': self.Acc_Angle_Y,
                'zg': self.Gyr_Angle_Z, 'za': self.Acc_Angle_Z }


