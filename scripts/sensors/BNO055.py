import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

import time
from devices.i2c_device import i2cDevice
from devices.tca9548a import TCA9548A

# I2C addresses
BNO055_ADDRESS_A                     = 0x28
BNO055_ADDRESS_B                     = 0x29
BNO055_ID                            = 0xA0

# Page id register definition
BNO055_PAGE_ID_ADDR                  = 0X07

# PAGE0 REGISTER DEFINITION START
BNO055_CHIP_ID_ADDR                  = 0x00
BNO055_ACCEL_REV_ID_ADDR             = 0x01
BNO055_MAG_REV_ID_ADDR               = 0x02
BNO055_GYRO_REV_ID_ADDR              = 0x03
BNO055_SW_REV_ID_LSB_ADDR            = 0x04
BNO055_SW_REV_ID_MSB_ADDR            = 0x05
BNO055_BL_REV_ID_ADDR                = 0X06

# Accel data register
BNO055_ACCEL_DATA_X_LSB_ADDR         = 0X08
BNO055_ACCEL_DATA_X_MSB_ADDR         = 0X09
BNO055_ACCEL_DATA_Y_LSB_ADDR         = 0X0A
BNO055_ACCEL_DATA_Y_MSB_ADDR         = 0X0B
BNO055_ACCEL_DATA_Z_LSB_ADDR         = 0X0C
BNO055_ACCEL_DATA_Z_MSB_ADDR         = 0X0D

# Mag data register
BNO055_MAG_DATA_X_LSB_ADDR           = 0X0E
BNO055_MAG_DATA_X_MSB_ADDR           = 0X0F
BNO055_MAG_DATA_Y_LSB_ADDR           = 0X10
BNO055_MAG_DATA_Y_MSB_ADDR           = 0X11
BNO055_MAG_DATA_Z_LSB_ADDR           = 0X12
BNO055_MAG_DATA_Z_MSB_ADDR           = 0X13

# Gyro data registers
BNO055_GYRO_DATA_X_LSB_ADDR          = 0X14
BNO055_GYRO_DATA_X_MSB_ADDR          = 0X15
BNO055_GYRO_DATA_Y_LSB_ADDR          = 0X16
BNO055_GYRO_DATA_Y_MSB_ADDR          = 0X17
BNO055_GYRO_DATA_Z_LSB_ADDR          = 0X18
BNO055_GYRO_DATA_Z_MSB_ADDR          = 0X19

# Euler data registers
BNO055_EULER_H_LSB_ADDR              = 0X1A
BNO055_EULER_H_MSB_ADDR              = 0X1B
BNO055_EULER_R_LSB_ADDR              = 0X1C
BNO055_EULER_R_MSB_ADDR              = 0X1D
BNO055_EULER_P_LSB_ADDR              = 0X1E
BNO055_EULER_P_MSB_ADDR              = 0X1F

# Quaternion data registers
BNO055_QUATERNION_DATA_W_LSB_ADDR    = 0X20
BNO055_QUATERNION_DATA_W_MSB_ADDR    = 0X21
BNO055_QUATERNION_DATA_X_LSB_ADDR    = 0X22
BNO055_QUATERNION_DATA_X_MSB_ADDR    = 0X23
BNO055_QUATERNION_DATA_Y_LSB_ADDR    = 0X24
BNO055_QUATERNION_DATA_Y_MSB_ADDR    = 0X25
BNO055_QUATERNION_DATA_Z_LSB_ADDR    = 0X26
BNO055_QUATERNION_DATA_Z_MSB_ADDR    = 0X27

# Linear acceleration data registers
BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR  = 0X28
BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR  = 0X29
BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR  = 0X2A
BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR  = 0X2B
BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR  = 0X2C
BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR  = 0X2D

# Gravity data registers
BNO055_GRAVITY_DATA_X_LSB_ADDR       = 0X2E
BNO055_GRAVITY_DATA_X_MSB_ADDR       = 0X2F
BNO055_GRAVITY_DATA_Y_LSB_ADDR       = 0X30
BNO055_GRAVITY_DATA_Y_MSB_ADDR       = 0X31
BNO055_GRAVITY_DATA_Z_LSB_ADDR       = 0X32
BNO055_GRAVITY_DATA_Z_MSB_ADDR       = 0X33

# Temperature data register
BNO055_TEMP_ADDR                     = 0X34

# Status registers
BNO055_CALIB_STAT_ADDR               = 0X35
BNO055_SELFTEST_RESULT_ADDR          = 0X36
BNO055_INTR_STAT_ADDR                = 0X37

BNO055_SYS_CLK_STAT_ADDR             = 0X38
BNO055_SYS_STAT_ADDR                 = 0X39
BNO055_SYS_ERR_ADDR                  = 0X3A

# Unit selection register
BNO055_UNIT_SEL_ADDR                 = 0X3B
BNO055_DATA_SELECT_ADDR              = 0X3C

# Mode registers
BNO055_OPR_MODE_ADDR                 = 0X3D
BNO055_PWR_MODE_ADDR                 = 0X3E

BNO055_SYS_TRIGGER_ADDR              = 0X3F
BNO055_TEMP_SOURCE_ADDR              = 0X40

# Axis remap registers
BNO055_AXIS_MAP_CONFIG_ADDR          = 0X41
BNO055_AXIS_MAP_SIGN_ADDR            = 0X42

# Axis remap values
AXIS_REMAP_X                         = 0x00
AXIS_REMAP_Y                         = 0x01
AXIS_REMAP_Z                         = 0x02
AXIS_REMAP_POSITIVE                  = 0x00
AXIS_REMAP_NEGATIVE                  = 0x01

# SIC registers
BNO055_SIC_MATRIX_0_LSB_ADDR         = 0X43
BNO055_SIC_MATRIX_0_MSB_ADDR         = 0X44
BNO055_SIC_MATRIX_1_LSB_ADDR         = 0X45
BNO055_SIC_MATRIX_1_MSB_ADDR         = 0X46
BNO055_SIC_MATRIX_2_LSB_ADDR         = 0X47
BNO055_SIC_MATRIX_2_MSB_ADDR         = 0X48
BNO055_SIC_MATRIX_3_LSB_ADDR         = 0X49
BNO055_SIC_MATRIX_3_MSB_ADDR         = 0X4A
BNO055_SIC_MATRIX_4_LSB_ADDR         = 0X4B
BNO055_SIC_MATRIX_4_MSB_ADDR         = 0X4C
BNO055_SIC_MATRIX_5_LSB_ADDR         = 0X4D
BNO055_SIC_MATRIX_5_MSB_ADDR         = 0X4E
BNO055_SIC_MATRIX_6_LSB_ADDR         = 0X4F
BNO055_SIC_MATRIX_6_MSB_ADDR         = 0X50
BNO055_SIC_MATRIX_7_LSB_ADDR         = 0X51
BNO055_SIC_MATRIX_7_MSB_ADDR         = 0X52
BNO055_SIC_MATRIX_8_LSB_ADDR         = 0X53
BNO055_SIC_MATRIX_8_MSB_ADDR         = 0X54

# Accelerometer Offset registers
ACCEL_OFFSET_X_LSB_ADDR              = 0X55
ACCEL_OFFSET_X_MSB_ADDR              = 0X56
ACCEL_OFFSET_Y_LSB_ADDR              = 0X57
ACCEL_OFFSET_Y_MSB_ADDR              = 0X58
ACCEL_OFFSET_Z_LSB_ADDR              = 0X59
ACCEL_OFFSET_Z_MSB_ADDR              = 0X5A

# Magnetometer Offset registers
MAG_OFFSET_X_LSB_ADDR                = 0X5B
MAG_OFFSET_X_MSB_ADDR                = 0X5C
MAG_OFFSET_Y_LSB_ADDR                = 0X5D
MAG_OFFSET_Y_MSB_ADDR                = 0X5E
MAG_OFFSET_Z_LSB_ADDR                = 0X5F
MAG_OFFSET_Z_MSB_ADDR                = 0X60

# Gyroscope Offset register s
GYRO_OFFSET_X_LSB_ADDR               = 0X61
GYRO_OFFSET_X_MSB_ADDR               = 0X62
GYRO_OFFSET_Y_LSB_ADDR               = 0X63
GYRO_OFFSET_Y_MSB_ADDR               = 0X64
GYRO_OFFSET_Z_LSB_ADDR               = 0X65
GYRO_OFFSET_Z_MSB_ADDR               = 0X66

# Radius registers
ACCEL_RADIUS_LSB_ADDR                = 0X67
ACCEL_RADIUS_MSB_ADDR                = 0X68
MAG_RADIUS_LSB_ADDR                  = 0X69
MAG_RADIUS_MSB_ADDR                  = 0X6A

# Power modes
POWER_MODE_NORMAL                    = 0X00
POWER_MODE_LOWPOWER                  = 0X01
POWER_MODE_SUSPEND                   = 0X02

# Operation mode settings
OPERATION_MODE_CONFIG                = 0X00
OPERATION_MODE_ACCONLY               = 0X01
OPERATION_MODE_MAGONLY               = 0X02
OPERATION_MODE_GYRONLY               = 0X03
OPERATION_MODE_ACCMAG                = 0X04
OPERATION_MODE_ACCGYRO               = 0X05
OPERATION_MODE_MAGGYRO               = 0X06
OPERATION_MODE_AMG                   = 0X07
OPERATION_MODE_IMUPLUS               = 0X08
OPERATION_MODE_COMPASS               = 0X09
OPERATION_MODE_M4G                   = 0X0A
OPERATION_MODE_NDOF_FMC_OFF          = 0X0B
OPERATION_MODE_NDOF                  = 0X0C


class BNO055(object):
    _mode = None
    def __init__(self, address=BNO055_ADDRESS_A, i2c_interface=None, busnum=1,
                        mux_interface=None, mux_addr=None, mux_channel=None):
        self.address = address

        if(i2c_interface is None):
            self.i2c = i2cDevice(address=address, busnum=busnum, i2c_interface=i2c_interface)
        else: self.i2c = i2c_interface

        if(mux_channel is None): self.channel = 0
        else: self.channel = mux_channel

        if(mux_interface is not None):
            self.mux = mux_interface
            self.master = mux_interface.address
        elif((mux_addr is not None) and (mux_interface is None)):
            self.master = mux_addr
            self.mux = TCA9548A(address=mux_addr, busnum=busnum, i2c_interface=self.i2c)
        else:
            self.master = None
            self.mux = None

    def __del__(self):
        try:    self.close()
        except: pass

    def close(self):
        try:    self.i2c.close()
        except: pass

        try:    self.mux.close()
        except: pass

    def set_mode(self, mode):
        """Set operation mode for BNO055 sensor.  Mode should be a value from
        table 3-3 and 3-5 of the datasheet:
          http://www.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
        """
        if(self.mux is not None):
            if(self.mux.current_channel != self.channel):
                print("[INFO] BNO055::set_mode() --- Switching i2c multiplexer to channel %d" % (self.channel))
                self.mux.select_channel(self.channel)
        self.i2c.writeByte(BNO055_OPR_MODE_ADDR, mode & 0xFF, address=self.address)
        # Delay for 30 milliseconds (datsheet recommends 19ms, but a little more
        # can't hurt and the kernel is going to spend some unknown amount of time
        # too).
        time.sleep(0.03)

    def config_mode(self):
        """ Enter configuration mode. """
        self.set_mode(OPERATION_MODE_CONFIG)

    def operation_mode(self):
        """Enter operation mode to read sensor data."""
        self.set_mode(self._mode)

    def begin(self, mode=OPERATION_MODE_NDOF):
        """Initialize the BNO055 sensor.  Must be called once before any other
        BNO055 library functions.  Will return True if the BNO055 was
        successfully initialized, and False otherwise.
        """
        if(self.mux is not None):
            if(self.mux.current_channel != self.channel):
                print("[INFO] BNO055::begin() --- Switching i2c multiplexer to channel %d" % (self.channel))
                self.mux.select_channel(self.channel)

        # Save the desired normal operation mode.
        self._mode = mode
        # First send a thow-away command and ignore any response or I2C errors
        # just to make sure the BNO is in a good state and ready to accept
        # commands (this seems to be necessary after a hard power down).
        try:
            self.i2c.writeByte(BNO055_PAGE_ID_ADDR, 0, address=self.address)
        except IOError:
            # Swallow an IOError that might be raised by an I2C issue.  Only do
            # this for this very first command to help get the BNO and board's
            # I2C into a clear state ready to accept the next commands.
            pass
        # Make sure we're in config mode and on page 0.
        self.config_mode()
        self.i2c.writeByte(BNO055_PAGE_ID_ADDR, 0, address=self.address)
        # Check the chip ID
        bno_id = self.i2c.readByte(BNO055_CHIP_ID_ADDR, address=self.address)
        # print('Read chip ID: 0x{0:02X}'.format(bno_id))
        if bno_id != BNO055_ID:
            return False
        # Reset the device.
        # Else use the reset command.  Note that ack=False is sent because
        # the chip doesn't seem to ack a reset in serial mode (by design?).
        self.i2c.writeByte(BNO055_SYS_TRIGGER_ADDR, 0x20, address=self.address)
        # Wait 650ms after reset for chip to be ready (as suggested in datasheet).
        time.sleep(0.65)
        # Set to normal power mode.
        self.i2c.writeByte(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL, address=self.address)
        # Default to internal oscillator.
        self.i2c.writeByte(BNO055_SYS_TRIGGER_ADDR, 0x0, address=self.address)
        # Enter normal operation mode.
        self.operation_mode()
        return True

    def get_revision(self):
        """Return a tuple with revision information about the BNO055 chip.  Will
        return 5 values:
          - Software revision
          - Bootloader version
          - Accelerometer ID
          - Magnetometer ID
          - Gyro ID
        """
        if(self.mux is not None):
            if(self.mux.current_channel != self.channel):
                print("[INFO] BNO055::get_revision() --- Switching i2c multiplexer to channel %d" % (self.channel))
                self.mux.select_channel(self.channel)
        # Read revision values.
        accel = self.i2c.readByte(BNO055_ACCEL_REV_ID_ADDR, address=self.address)
        mag = self.i2c.readByte(BNO055_MAG_REV_ID_ADDR, address=self.address)
        gyro = self.i2c.readByte(BNO055_GYRO_REV_ID_ADDR, address=self.address)
        bl = self.i2c.readByte(BNO055_BL_REV_ID_ADDR, address=self.address)
        sw_lsb = self.i2c.readByte(BNO055_SW_REV_ID_LSB_ADDR, address=self.address)
        sw_msb = self.i2c.readByte(BNO055_SW_REV_ID_MSB_ADDR, address=self.address)
        sw = ((sw_msb << 8) | sw_lsb) & 0xFFFF
        # Return the results as a tuple of all 5 values.
        return (sw, bl, accel, mag, gyro)

    def set_external_crystal(self, external_crystal):
        """Set if an external crystal is being used by passing True, otherwise
        use the internal oscillator by passing False (the default behavior).
        """
        if(self.mux is not None):
            if(self.mux.current_channel != self.channel):
                print("[INFO] BNO055::set_external_crystal() --- Switching i2c multiplexer to channel %d" % (self.channel))
                self.mux.select_channel(self.channel)
        # Switch to configuration mode.
        self.config_mode()
        # Set the clock bit appropriately in the SYS_TRIGGER register.
        if external_crystal:
            self.i2c.writeByte(BNO055_SYS_TRIGGER_ADDR, 0x80, address=self.address)
        else:
            self.i2c.writeByte(BNO055_SYS_TRIGGER_ADDR, 0x00, address=self.address)
        # Go back to normal operation mode.
        self.operation_mode()

    def get_system_status(self, run_self_test=True):
        """Return a tuple with status information.  Three values will be returned:
          - System status register value with the following meaning:
              0 = Idle
              1 = System Error
              2 = Initializing Peripherals
              3 = System Initialization
              4 = Executing Self-Test
              5 = Sensor fusion algorithm running
              6 = System running without fusion algorithms
          - Self test result register value with the following meaning:
              Bit value: 1 = test passed, 0 = test failed
              Bit 0 = Accelerometer self test
              Bit 1 = Magnetometer self test
              Bit 2 = Gyroscope self test
              Bit 3 = MCU self test
              Value of 0x0F = all good!
          - System error register value with the following meaning:
              0 = No error
              1 = Peripheral initialization error
              2 = System initialization error
              3 = Self test result failed
              4 = Register map value out of range
              5 = Register map address out of range
              6 = Register map write error
              7 = BNO low power mode not available for selected operation mode
              8 = Accelerometer power mode not available
              9 = Fusion algorithm configuration error
             10 = Sensor configuration error

        If run_self_test is passed in as False then no self test is performed and
        None will be returned for the self test result.  Note that running a
        self test requires going into config mode which will stop the fusion
        engine from running.
        """
        if(self.mux is not None):
            if(self.mux.current_channel != self.channel):
                print("[INFO] BNO055::get_system_status() --- Switching i2c multiplexer to channel %d" % (self.channel))
                self.mux.select_channel(self.channel)
        self_test = None
        if run_self_test:
            # Switch to configuration mode if running self test.
            self.config_mode()
            # Perform a self test.
            sys_trigger = self.i2c.readByte(BNO055_SYS_TRIGGER_ADDR, address=self.address)
            self.i2c.writeByte(BNO055_SYS_TRIGGER_ADDR, sys_trigger | 0x1, address=self.address)
            # Wait for self test to finish.
            time.sleep(1.0)
            # Read test result.
            self_test = self.i2c.readByte(BNO055_SELFTEST_RESULT_ADDR, address=self.address)
            # Go back to operation mode.
            self.operation_mode()
        # Now read status and error registers.
        status = self.i2c.readByte(BNO055_SYS_STAT_ADDR, address=self.address)
        error = self.i2c.readByte(BNO055_SYS_ERR_ADDR, address=self.address)
        # Return the results as a tuple of all 3 values.
        return (status, self_test, error)

    def get_calibration_status(self):
        """Read the calibration status of the sensors and return a 4 tuple with
        calibration status as follows:
          - System, 3=fully calibrated, 0=not calibrated
          - Gyroscope, 3=fully calibrated, 0=not calibrated
          - Accelerometer, 3=fully calibrated, 0=not calibrated
          - Magnetometer, 3=fully calibrated, 0=not calibrated
        """
        if(self.mux is not None):
            if(self.mux.current_channel != self.channel):
                print("[INFO] BNO055::get_calibration_status() --- Switching i2c multiplexer to channel %d" % (self.channel))
                self.mux.select_channel(self.channel)
        # Return the calibration status register value.
        cal_status = self.i2c.readByte(BNO055_CALIB_STAT_ADDR, address=self.address)
        sys = (cal_status >> 6) & 0x03
        gyro = (cal_status >> 4) & 0x03
        accel = (cal_status >> 2) & 0x03
        mag = cal_status & 0x03
        # Return the results as a tuple of all 3 values.
        return (sys, gyro, accel, mag)

    def get_calibration(self):
        """Return the sensor's calibration data and return it as an array of
        22 bytes. Can be saved and then reloaded with the set_calibration function
        to quickly calibrate from a previously calculated set of calibration data.
        """
        if(self.mux is not None):
            if(self.mux.current_channel != self.channel):
                print("[INFO] BNO055::get_calibration() --- Switching i2c multiplexer to channel %d" % (self.channel))
                self.mux.select_channel(self.channel)
        # Switch to configuration mode, as mentioned in section 3.10.4 of datasheet.
        self.config_mode()
        # Read the 22 bytes of calibration data and convert it to a list (from
        # a bytearray) so it's more easily serialized should the caller want to
        # store it.
        cal_data = list(self.i2c.readBytes(ACCEL_OFFSET_X_LSB_ADDR, 22, address=self.address))
        # Go back to normal operation mode.
        self.operation_mode()
        return cal_data

    def set_calibration(self, data):
        """Set the sensor's calibration data using a list of 22 bytes that
        represent the sensor offsets and calibration data.  This data should be
        a value that was previously retrieved with get_calibration (and then
        perhaps persisted to disk or other location until needed again).
        """
        if(self.mux is not None):
            if(self.mux.current_channel != self.channel):
                print("[INFO] BNO055::set_calibration() --- Switching i2c multiplexer to channel %d" % (self.channel))
                self.mux.select_channel(self.channel)
        # Check that 22 bytes were passed in with calibration data.
        if data is None or len(data) != 22:
            raise ValueError('Expected a list of 22 bytes for calibration data.')
        # Switch to configuration mode, as mentioned in section 3.10.4 of datasheet.
        self.config_mode()
        # Set the 22 bytes of calibration data.
        self.i2c.writeBytes(ACCEL_OFFSET_X_LSB_ADDR, data, address=self.address)
        # Go back to normal operation mode.
        self.operation_mode()

    def get_axis_remap(self):
        """Return a tuple with the axis remap register values.  This will return
        6 values with the following meaning:
          - X axis remap (a value of AXIS_REMAP_X, AXIS_REMAP_Y, or AXIS_REMAP_Z.
                          which indicates that the physical X axis of the chip
                          is remapped to a different axis)
          - Y axis remap (see above)
          - Z axis remap (see above)
          - X axis sign (a value of AXIS_REMAP_POSITIVE or AXIS_REMAP_NEGATIVE
                         which indicates if the X axis values should be positive/
                         normal or negative/inverted.  The default is positive.)
          - Y axis sign (see above)
          - Z axis sign (see above)

        Note that by default the axis orientation of the BNO chip looks like
        the following (taken from section 3.4, page 24 of the datasheet).  Notice
        the dot in the corner that corresponds to the dot on the BNO chip:

                           | Z axis
                           |
                           |   / X axis
                       ____|__/____
          Y axis     / *   | /    /|
          _________ /______|/    //
                   /___________ //
                  |____________|/
        """
        if(self.mux is not None):
            if(self.mux.current_channel != self.channel):
                print("[INFO] BNO055::get_axis_remap() --- Switching i2c multiplexer to channel %d" % (self.channel))
                self.mux.select_channel(self.channel)
        # Get the axis remap register value.
        map_config = self.i2c.readByte(BNO055_AXIS_MAP_CONFIG_ADDR, address=self.address)
        z = (map_config >> 4) & 0x03
        y = (map_config >> 2) & 0x03
        x = map_config & 0x03
        # Get the axis remap sign register value.
        sign_config = self.i2c.readByte(BNO055_AXIS_MAP_SIGN_ADDR, address=self.address)
        x_sign = (sign_config >> 2) & 0x01
        y_sign = (sign_config >> 1) & 0x01
        z_sign = sign_config & 0x01
        # Return the results as a tuple of all 3 values.
        return (x, y, z, x_sign, y_sign, z_sign)

    def set_axis_remap(self, x, y, z,
                       x_sign=AXIS_REMAP_POSITIVE, y_sign=AXIS_REMAP_POSITIVE,
                       z_sign=AXIS_REMAP_POSITIVE):
        """Set axis remap for each axis.  The x, y, z parameter values should
        be set to one of AXIS_REMAP_X, AXIS_REMAP_Y, or AXIS_REMAP_Z and will
        change the BNO's axis to represent another axis.  Note that two axises
        cannot be mapped to the same axis, so the x, y, z params should be a
        unique combination of AXIS_REMAP_X, AXIS_REMAP_Y, AXIS_REMAP_Z values.

        The x_sign, y_sign, z_sign values represent if the axis should be positive
        or negative (inverted).

        See the get_axis_remap documentation for information on the orientation
        of the axises on the chip, and consult section 3.4 of the datasheet.
        """
        if(self.mux is not None):
            if(self.mux.current_channel != self.channel):
                print("[INFO] BNO055::set_axis_remap() --- Switching i2c multiplexer to channel %d" % (self.channel))
                self.mux.select_channel(self.channel)
        # Switch to configuration mode.
        self.config_mode()
        # Set the axis remap register value.
        map_config = 0x00
        map_config |= (z & 0x03) << 4
        map_config |= (y & 0x03) << 2
        map_config |= x & 0x03
        self.i2c.writeByte(BNO055_AXIS_MAP_CONFIG_ADDR, map_config, address=self.address)
        # Set the axis remap sign register value.
        sign_config = 0x00
        sign_config |= (x_sign & 0x01) << 2
        sign_config |= (y_sign & 0x01) << 1
        sign_config |= z_sign & 0x01
        self.i2c.writeByte(BNO055_AXIS_MAP_SIGN_ADDR, sign_config, address=self.address)
        # Go back to normal operation mode.
        self.operation_mode()

    def _read_vector(self, address, count=3):
        # Read count number of 16-bit signed values starting from the provided
        # address. Returns a tuple of the values that were read.
        data = self.i2c.readBytes(address, count*2, address=self.address)
        result = [0]*count
        for i in range(count):
            result[i] = ((data[i*2+1] << 8) | data[i*2]) & 0xFFFF
            if result[i] > 32767:
                result[i] -= 65536
        return result

    def read_euler(self):
        """Return the current absolute orientation as a tuple of heading, roll,
        and pitch euler angles in degrees.
        """
        if(self.mux is not None):
            if(self.mux.current_channel != self.channel):
                print("[INFO] BNO055::read_euler() --- Switching i2c multiplexer to channel %d" % (self.channel))
                self.mux.select_channel(self.channel)
        heading, roll, pitch = self._read_vector(BNO055_EULER_H_LSB_ADDR)
        return (heading/16.0, roll/16.0, pitch/16.0)

    def read_magnetometer(self):
        """Return the current magnetometer reading as a tuple of X, Y, Z values
        in micro-Teslas.
        """
        if(self.mux is not None):
            if(self.mux.current_channel != self.channel):
                print("[INFO] BNO055::read_magnetometer() --- Switching i2c multiplexer to channel %d" % (self.channel))
                self.mux.select_channel(self.channel)
        x, y, z = self._read_vector(BNO055_MAG_DATA_X_LSB_ADDR)
        return (x/16.0, y/16.0, z/16.0)

    def read_gyroscope(self):
        """Return the current gyroscope (angular velocity) reading as a tuple of
        X, Y, Z values in degrees per second.
        """
        if(self.mux is not None):
            if(self.mux.current_channel != self.channel):
                print("[INFO] BNO055::read_gyroscope() --- Switching i2c multiplexer to channel %d" % (self.channel))
                self.mux.select_channel(self.channel)
        x, y, z = self._read_vector(BNO055_GYRO_DATA_X_LSB_ADDR)
        return (x/900.0, y/900.0, z/900.0)

    def read_accelerometer(self):
        """Return the current accelerometer reading as a tuple of X, Y, Z values
        in meters/second^2.
        """
        if(self.mux is not None):
            if(self.mux.current_channel != self.channel):
                print("[INFO] BNO055::read_accelerometer() --- Switching i2c multiplexer to channel %d" % (self.channel))
                self.mux.select_channel(self.channel)
        x, y, z = self._read_vector(BNO055_ACCEL_DATA_X_LSB_ADDR)
        return (x/100.0, y/100.0, z/100.0)

    def read_linear_acceleration(self):
        """Return the current linear acceleration (acceleration from movement,
        not from gravity) reading as a tuple of X, Y, Z values in meters/second^2.
        """
        if(self.mux is not None):
            if(self.mux.current_channel != self.channel):
                print("[INFO] BNO055::read_linear_acceleration() --- Switching i2c multiplexer to channel %d" % (self.channel))
                self.mux.select_channel(self.channel)
        x, y, z = self._read_vector(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR)
        return (x/100.0, y/100.0, z/100.0)

    def read_gravity(self):
        """Return the current gravity acceleration reading as a tuple of X, Y, Z
        values in meters/second^2.
        """
        if(self.mux is not None):
            if(self.mux.current_channel != self.channel):
                print("[INFO] BNO055::read_gravity() --- Switching i2c multiplexer to channel %d" % (self.channel))
                self.mux.select_channel(self.channel)
        x, y, z = self._read_vector(BNO055_GRAVITY_DATA_X_LSB_ADDR)
        return (x/100.0, y/100.0, z/100.0)

    def read_quaternion(self):
        """Return the current orientation as a tuple of X, Y, Z, W quaternion
        values.
        """
        if(self.mux is not None):
            if(self.mux.current_channel != self.channel):
                print("[INFO] BNO055::read_quaternion() --- Switching i2c multiplexer to channel %d" % (self.channel))
                self.mux.select_channel(self.channel)
        w, x, y, z = self._read_vector(BNO055_QUATERNION_DATA_W_LSB_ADDR, 4)
        # Scale values, see 3.6.5.5 in the datasheet.
        scale = (1.0 / (1<<14))
        return (x*scale, y*scale, z*scale, w*scale)

    def read_temp(self):
        """Return the current temperature in Celsius."""
        if(self.mux is not None):
            if(self.mux.current_channel != self.channel):
                print("[INFO] BNO055::read_temp() --- Switching i2c multiplexer to channel %d" % (self.channel))
                self.mux.select_channel(self.channel)
        return self.i2c.readSignedByte(BNO055_TEMP_ADDR, address=self.address)

    def startup(self):
        if not self.begin():
            raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

        # Print system status and self test result.
        status, self_test, error = self.get_system_status()
        print('System status: {0}'.format(status))
        print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
        # Print out an error if system status is in error mode.
        if status == 0x01:
            print('System error: {0}'.format(error))
            print('See datasheet section 4.3.59 for the meaning.')

        # Print BNO055 software revision and other diagnostic data.
        sw, bl, accel, mag, gyro = self.get_revision()
        print('Software version:   {0}'.format(sw))
        print('Bootloader version: {0}'.format(bl))
        print('Accelerometer ID:   0x{0:02X}'.format(accel))
        print('Magnetometer ID:    0x{0:02X}'.format(mag))
        print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

        cal_data = self.get_calibration()
        xm, ym, zm, x_sign, y_sign, z_sign = self.get_axis_remap()
        print("Stored Calibration Data: %s" % (str(cal_data)))
        print("Axis Mappings:")
        print("\t X-Axis      - %s" % str(xm))
        print("\t X-Axis Sign - %s" % str(x_sign))
        print("\t Y-Axis      - %s" % str(ym))
        print("\t Y-Axis Sign - %s" % str(y_sign))
        print("\t Z-Axis      - %s" % str(zm))
        print("\t Z-Axis Sign - %s" % str(z_sign))



    def get_data(self, getAccel = True, getGyro = True, getMag = True,
                    getAngles = True, getQuats = True, getTemperature = True):

        data_dict = {
            "ax": 0.0, "ay": 0.0, "az": 0.0,
            "gx": 0.0, "gy": 0.0, "gz": 0.0,
            "mx": 0.0, "my": 0.0, "mz": 0.0,
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 0.0,
            "temperature": 0.0,
        }

        if(getAccel):
            ax, ay, az = self.read_accelerometer()
            data_dict["ax"] = ax
            data_dict["ay"] = ay
            data_dict["az"] = az
        if(getGyro):
            gx, gy, gz = self.read_gyroscope()
            data_dict["gx"] = gx
            data_dict["gy"] = gy
            data_dict["gz"] = gz
        if(getMag):
            mx, my, mz = self.read_magnetometer()
            data_dict["mx"] = mx
            data_dict["my"] = my
            data_dict["mz"] = mz
        if(getAngles):
            heading, roll, pitch = self.read_euler()
            data_dict["roll"] = roll
            data_dict["pitch"] = pitch
            data_dict["yaw"] = heading
        if(getQuats):
            qx,qy,qz,qw = self.read_quaternion()
            data_dict["qx"] = qx
            data_dict["qy"] = qy
            data_dict["qz"] = qz
            data_dict["qw"] = qw
        if(getTemperature):
            temperature = self.read_temp()
            data_dict["temperature"] = temperature

        return data_dict

if __name__ == "__main__" :
    import argparse, pprint
    pp = pprint.PrettyPrinter(indent=5)
    # Setup commandline argument(s) structures
    ap = argparse.ArgumentParser(description='I2C multiplixer scanner')
    ap.add_argument("--bus", "-b", type=int, default=1, metavar='BUS', help="Id of the i2c bus you want to scan")
    ap.add_argument("--channel", "-c", type=int, default=4, metavar='CHANNEL', help="i2c multiplexer channel corresponding to the target device")
    ap.add_argument("--mux_addr", "-M", type=int, default=0x70, metavar='ADDRESS', help="Address of i2c multiplexer")
    ap.add_argument("--use_mux", "-m", action="store_true", help="use i2c multiplexert")
    ap.add_argument("--verbose", "-v", action="store_true", help="Verbose output")
    # Store parsed arguments into array of variables
    args = vars(ap.parse_args())

    # Extract stored arguments array into individual variables for later usage in script
    bus = args["bus"]
    mux_channel = args["channel"]
    mux_addr = args["mux_addr"]
    use_mux = args["use_mux"]
    flag_verbose = args["verbose"]

    if(use_mux):
        mux = TCA9548A(address=mux_addr, busnum=bus, i2c_interface=None)
        imu = BNO055(mux_interface=mux,i2c_interface=mux.i2c,mux_channel=mux_channel)
    else: imu = BNO055(i2c_interface=None, busnum=bus)

    imu.startup()
    print(" ------------ ")
    idata = imu.get_data()
    print("Imu Data:")
    print(" ------------ ")
    pp.pprint(idata)
    print(" ------------ ")
    print('Reading BNO055 data, press Ctrl-C to quit...')
    raw_input("Please press [Enter] to begin...")
    while True:
        # Read the Euler angles for heading, roll, pitch (all in degrees).
        heading, roll, pitch = imu.read_euler()
        # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
        sys, gyro, accel, mag = imu.get_calibration_status()
        # Print everything out.
        print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
              heading, roll, pitch, sys, gyro, accel, mag))
        # Other values you can optionally read:
        # Orientation as a quaternion:
        #x,y,z,w = imu.read_quaterion()
        # Sensor temperature in degrees Celsius:
        #temp_c = imu.read_temp()
        # Magnetometer data (in micro-Teslas):
        #x,y,z = imu.read_magnetometer()
        # Gyroscope data (in degrees per second):
        #x,y,z = imu.read_gyroscope()
        # Accelerometer data (in meters per second squared):
        #x,y,z = imu.read_accelerometer()
        # Linear acceleration data (i.e. acceleration from movement, not gravity--
        # returned in meters per second squared):
        #x,y,z = imu.read_linear_acceleration()
        # Gravity acceleration data (i.e. acceleration just from gravity--returned
        # in meters per second squared):
        #x,y,z = imu.read_gravity()
        # Sleep for a second until the next reading.
        time.sleep(0.01)

    print("---------------")
