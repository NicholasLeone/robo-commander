#!/usr/bin/env python
import sys
import time
import smbus
import struct

class i2cDevice(object):
    def __init__(self, address, i2c_interface=None, busnum=1, verbose = False):
        self._address = address
        self.verbose = verbose
        # Use default smbus interface if none other is provided
        if i2c_interface is None: self.bus = smbus.SMBus(busnum)
        else: self.bus = i2c_interface
        if(self.verbose): print('Initialzed I2C Device Address [{1:#0X}] on Bus{0}.'.format(busnum, address))

    def __del__(self):
        try:    self.close()
        except: pass

    def close(self):
        self.bus.close()

    def set_verbose(self, verbose):
        self.verbose = verbose

    def writeRaw8(self, value, address = None):
        """ Write an 8-bit value on the bus (without register). """
        if(address is None): address = self._address
        value = value & 0xFF
        self.bus.write_byte(address, value)
        if(self.verbose): print("Wrote Value 0x%02X to address 0x%02X" % (value,address) )

    def write8(self, register, value, address = None):
        """ Write an 8-bit value to the specified register. """
        if(address is None): address = self._address
        value = value & 0xFF
        self.bus.write_byte_data(address, register, value)
        if(self.verbose): print("Wrote 0x%02X to register 0x%02X at address 0x%02X" % (value, register,address) )

    def write16(self, register, value, address = None):
        """ Write a 16-bit value to the specified register. """
        if(address is None): address = self._address
        value = value & 0xFFFF
        self.bus.write_word_data(address, register, value)
        if(self.verbose): print("Wrote 0x%04X to register pair 0x%02X, 0x%02X at address 0x%02X" % (value, register, register+1,address) )

    def writeList(self, register, data, address = None):
        """ Write bytes to the specified register. """
        if(address is None): address = self._address
        self.bus.write_i2c_block_data(address, register, data)
        if(self.verbose): print("Wrote to register 0x%02X at address 0x%02X: %s" % (register, address, str(data)) )

    def readList(self, register, length, address = None):
        """ Read a length number of bytes from the specified register.  Results
        will be returned as a bytearray. """
        if(address is None): address = self._address
        results = self.bus.read_i2c_block_data(address, register, length)
        if(self.verbose): print("Read the following from register 0x%02X at address 0x%02X: %s" % (register, address, str(results)) )
        return results

    def readRaw8(self, address = None):
        """Read an 8-bit value on the bus (without register)."""
        if(address is None): address = self._address
        result = self.bus.read_byte(address) & 0xFF
        if(self.verbose): print("Read 0x%02X from address 0x%02X" % (result, address))
        return result

    def readU8(self, register, address = None):
        """ Read an unsigned byte from the specified register. """
        if(address is None): address = self._address
        result = self.bus.read_byte_data(address, register) & 0xFF
        if(self.verbose): print("Read 0x%02X from register 0x%02X at address 0x%02X" % (result, register, address) )
        return result

    def readS8(self, register, address = None):
        """ Read a signed byte from the specified register. """
        if(address is None): address = self._address
        result = self.readU8(register, address=address)
        if result > 127: result -= 256
        return result

    def readU16(self, register, address = None, little_endian=True):
        """ Read an unsigned 16-bit value from the specified register, with the
        specified endianness (default little endian, or least significant byte
        first). """
        if(address is None): address = self._address
        result = self.bus.read_word_data(address,register) & 0xFFFF
        if(self.verbose): print("Read 0x%04X from register pair 0x%02X, 0x%02X from address 0x%02X" % (result, register, register+1, address))

        # Swap bytes if using big endian because read_word_data assumes little
        # endian on ARM (little endian) systems.
        if not little_endian: result = ((result << 8) & 0xFF00) + (result >> 8)
        return result

    def readS16(self, register, address = None, little_endian=True):
        """ Read a signed 16-bit value from the specified register, with the
        specified endianness (default little endian, or least significant byte
        first). """
        if(address is None): address = self._address
        result = self.readU16(register, address=address, little_endian=little_endian)
        if result > 32767: result -= 65536
        return result

    def readU16LE(self, register, address = None):
        """ Read an unsigned 16-bit value from the specified register, in little
        endian byte order. """
        if(address is None): address = self._address
        return self.readU16(register, address=address, little_endian=True)

    def readU16BE(self, register, address = None):
        """ Read an unsigned 16-bit value from the specified register, in big
        endian byte order. """
        if(address is None): address = self._address
        return self.readU16(register, address=address, little_endian=False)

    def readS16LE(self, register, address = None):
        """ Read a signed 16-bit value from the specified register, in little
        endian byte order. """
        if(address is None): address = self._address
        return self.readS16(register, address=address, little_endian=True)

    def readS16BE(self, register, address = None):
        """ Read a signed 16-bit value from the specified register, in big
        endian byte order. """
        if(address is None): address = self._address
        return self.readS16(register, address=address, little_endian=False)

    def writeBytes(self, register, data, address = None):
        self.writeList(register, data, address=address)

    def writeByte(self, register, value, address = None):
        self.write8(register, value, address=address)

    def readBytes(self, register, length, address = None):
        return bytearray(self.readList(register, length, address=address))

    def readByte(self, register, address = None):
        return self.readU8(register, address=address)

    def readSignedByte(self, register, address = None):
        data = self.readByte(register, address=address)
        if data > 127: return data - 256
        else: return data
