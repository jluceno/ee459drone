import smbus

class atmega:
  def __init__(self):
    self.addr = 0x24
    self.bus = smbus.SMbus(1)
  
  def get_throttle(self, channel):
    if channel == 1:
      self.bus.write_byte_data(self.addr, 0, 0)
      return self.bus.read_byte_data(self.addr, 0)
    elif channel == 2:
      self.bus.write_byte_data(self.addr, 0, 1)
      return self.bus.read_byte_data(self.addr, 1)
    elif channel == 3:
      self.bus.write_byte_data(self.addr, 0, 2)
      return self.bus.read_byte_data(self.addr, 2)
    elif channel == 4:
      self.bus.write_byte_data(self.addr, 0, 3)
      return self.bus.read_byte_data(self.addr, 3)
    else:
      return -1
    