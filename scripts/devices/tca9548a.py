#!/usr/bin/env python
from i2c_device import i2cDevice

class TCA9548A(object):
    def __init__(self, address=0x70, busnum=1, i2c_interface=None):
        self.address = address
        self.busnum = busnum
        self.current_channel = 0
        if(i2c_interface is None):
            self.i2c = i2cDevice(address=address, busnum=busnum, i2c_interface=i2c_interface)
        else: self.i2c = i2c_interface

    def __del__(self):
        print("Killing TCA9548A...")
        try:    self.close()
        except: pass

    def close(self):
        self.i2c.close()

    def select_channel(self,channel,address=None):
        if(address is None): address = self.address
        try:
            self.i2c.writeRaw8(2**channel, address)
            self.current_channel = channel
            return 1
        except: return 0

    def scan_bus(self, target, address=None, delay=None, verbose=False, debug=False):
        if(address is None): address = self.address
        # Search through all 8 channels
        for ch in range(8):
            self.select_channel(ch,address)
            # Search through all supported address id's
            for i in range(128):
                if(i == address): continue
                data = 0
                try:
                    self.i2c.writeRaw8(0,i)
                    if(i == target): print("Found Target [0x%02X] at address 0x%02X on Channel %d"% (i,address,ch))
                    else:
                        if(verbose): print("Found device [0x%02X] at address 0x%02X on Channel %d" % (i,address,ch))
                except:
                    if(debug): print("Could not write to address %d on Bus %d" % (i,ch))
                    pass
                if(delay is not None): time.sleep(delay)


if __name__ == "__main__" :
    mux = TCA9548A(address=0x70, busnum=1, i2c_interface=None)
    mux.scan_bus(0x28,verbose=True)
    print("---------------")
