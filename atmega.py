import smbus
import time

time_waiting = 0.05

class atmega:
  def __init__(self):
    self.addr = 0x18
    self.bus = smbus.SMBus(1)
  
  def get_throttle(self, channel):
    if channel == 1:
      self.bus.write_byte_data(self.addr, 0, 0)
      time.sleep(time_waiting)
      return self.bus.read_byte_data(self.addr, 0)
    elif channel == 2:
      self.bus.write_byte_data(self.addr, 0, 1)
      time.sleep(time_waiting)
      return self.bus.read_byte_data(self.addr, 1)
    elif channel == 3:
      self.bus.write_byte_data(self.addr, 0, 2)
      time.sleep(time_waiting)
      return self.bus.read_byte_data(self.addr, 2)
    elif channel == 4:
      self.bus.write_byte_data(self.addr, 0, 3)
      time.sleep(time_waiting)
      return self.bus.read_byte_data(self.addr, 3)
    else:
      return -1
    