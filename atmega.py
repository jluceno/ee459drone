import smbus
import time

time_waiting = 0.05

class atmega:
  def __init__(self):
    self.addr = 0x18
    self.bus = smbus.SMBus(1)
  
  def get_throttle(self, channel):
    if channel == 1:
      self.bus.write_byte(self.addr, 0)
      time.sleep(time_waiting)
    elif channel == 2:
      self.bus.write_byte(self.addr, 1)
      time.sleep(time_waiting)
    elif channel == 3:
      self.bus.write_byte(self.addr, 2)
      time.sleep(time_waiting)
    elif channel == 4:
      self.bus.write_byte(self.addr, 3)
      time.sleep(time_waiting)
    elif channel == 5:
      self.bus.write_byte(self.addr, 4)
      time.sleep(time_waiting)
      return 
    else:
      return -1
    return self.bus.read_byte(self.addr)
    