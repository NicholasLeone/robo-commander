from i2c_device import i2cDevice

class TCA9548A(object):
    def __init__(self, address=0x70, busnum=1, i2c_interface=None):
        self.address = address
        self.busnum = busnum
        self.i2c = i2cDevice(address=address, busnum=busnum, i2c_interface=i2c_interface)

    def __del__(self):
        print("Killing TCA9548A...")

    def select_channel(self,channel,address=None):
        if(address is None): address = self.address
        try:
            self.i2c.bus.write_byte(address,2**channel)
            return 1
        except: return 0

    def scan_bus(self, target, address=None, delay=None, verbose=False, debug=False):
        if(address is None): address = self.address
        # Search through all 8 channels
        for ch in range(8):
            self.select_channel(address, ch)
            # Search through all supported address id's
            for i in range(128):
                if(i == address): continue
                data = 0
                try:
                    self.i2c.bus.write_byte(i,0)
                    if(i == target): print("Found Target [0x%02X] at address 0x%02X on Channel %d"% (i,address,ch))
                    else:
                        if(verbose): print("Found device [0x%02X] at address 0x%02X on Channel %d" % (i,address,ch))
                except:
                    if(debug): print("Could not write to address %d on Bus %d" % (i,ch))
                    pass
                if(delay is not None): time.sleep(delay)


if __name__ == "__main__" :
    mux = TCA9548A()
    mux.scan_bus(0x28)
    print("---------------")
