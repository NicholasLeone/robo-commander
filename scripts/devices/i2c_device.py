import sys
import time
import smbus
import struct

class i2cDevice(object):
    def __init__(self, address, busnum=1, i2c_interface=None):
        self._address = address
        # Use default smbus interface if none other is provided
        if i2c_interface is None: self.bus = smbus.SMBus(busnum)
        else: self.bus = i2c_interface
        print('Initialzed I2C Device Address [{1:#0X}] on Bus{0}.'.format(busnum, address))

    def writeRaw8(self, value):
        """ Write an 8-bit value on the bus (without register). """
        value = value & 0xFF
        self.bus.write_byte(self._address, value)
        # print("Wrote 0x%02X",value)

    def write8(self, register, value):
        """ Write an 8-bit value to the specified register. """
        value = value & 0xFF
        self.bus.write_byte_data(self._address, register, value)
        # print("Wrote 0x%02X to register 0x%02X",value, register)

    def write16(self, register, value):
        """ Write a 16-bit value to the specified register. """
        value = value & 0xFFFF
        self.bus.write_word_data(self._address, register, value)
        # print("Wrote 0x%04X to register pair 0x%02X, 0x%02X",value, register, register+1)

    def writeList(self, register, data):
        """ Write bytes to the specified register. """
        self.bus.write_i2c_block_data(self._address, register, data)
        self._logger.debug("Wrote to register 0x%02X: %s",register, data)

    def readList(self, register, length):
        """ Read a length number of bytes from the specified register.  Results
        will be returned as a bytearray. """
        results = self.bus.read_i2c_block_data(self._address, register, length)
        # print("Read the following from register 0x%02X: %s",register, results)
        return results

    def readRaw8(self):
        """Read an 8-bit value on the bus (without register)."""
        result = self.bus.read_byte(self._address) & 0xFF
        # print("Read 0x%02X",result)
        return result

    def readU8(self, register):
        """ Read an unsigned byte from the specified register. """
        result = self.bus.read_byte_data(self._address, register) & 0xFF
        # print("Read 0x%02X from register 0x%02X",result, register)
        return result

    def readS8(self, register):
        """ Read a signed byte from the specified register. """
        result = self.readU8(register)
        if result > 127: result -= 256
        return result

    def readU16(self, register, little_endian=True):
        """ Read an unsigned 16-bit value from the specified register, with the
        specified endianness (default little endian, or least significant byte
        first). """
        result = self.bus.read_word_data(self._address,register) & 0xFFFF
        # print("Read 0x%04X from register pair 0x%02X, 0x%02X", result, register, register+1)

        # Swap bytes if using big endian because read_word_data assumes little
        # endian on ARM (little endian) systems.
        if not little_endian: result = ((result << 8) & 0xFF00) + (result >> 8)
        return result

    def readS16(self, register, little_endian=True):
        """ Read a signed 16-bit value from the specified register, with the
        specified endianness (default little endian, or least significant byte
        first). """
        result = self.readU16(register, little_endian)
        if result > 32767: result -= 65536
        return result

    def readU16LE(self, register):
        """ Read an unsigned 16-bit value from the specified register, in little
        endian byte order. """
        return self.readU16(register, little_endian=True)

    def readU16BE(self, register):
        """ Read an unsigned 16-bit value from the specified register, in big
        endian byte order. """
        return self.readU16(register, little_endian=False)

    def readS16LE(self, register):
        """ Read a signed 16-bit value from the specified register, in little
        endian byte order. """
        return self.readS16(register, little_endian=True)

    def readS16BE(self, register):
        """ Read a signed 16-bit value from the specified register, in big
        endian byte order. """
        return self.readS16(register, little_endian=False)

    def writeBytes(self, address, data):
        self.writeList(address, data)

    def writeByte(self, address, value):
        self.write8(address, value)

    def readBytes(self, address, length):
        return bytearray(self.readList(address, length))

    def readByte(self, address):
        return self.readU8(address)

    def readSignedByte(self, address):
        data = self.readByte(address)
        if data > 127: return data - 256
        else: return data
