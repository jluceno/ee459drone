import smbus
import time



class atmega:
  def __init__(self):
    self.addr = 0x18
    self.bus = smbus.SMBus(1)

    self.time_waiting = 0.01
    self.prev_ch1 = 0
    self.prev_ch2 = 0
    self.prev_ch3 = 0
    self.prev_ch4 = 0
    self.prev_voltage = 0
  
  ## Gets input from the controller and battery info. Returns a percentage.

  ## For sticks:
  ## 0 = stick is at the minimum value
  ## 50 = stick is in a neutral position
  ## 100 = stick is at the maximum value

  ## For battery:
  ## 0 = battery is at 9.00V. DO NOT REACH THIS POINT, IT MAY DAMAGE THE BATTERY
  ## 50 = battery is at ~11.0V. stop using drone at this charge
  ## 100 = battery is at 12.45V. fully charged

  ## args:
  ## channel = 1,2,3,4 get channels 1,2,3 or 4.
  ## channel = 5, get the battery level
  def get_data(self, channel):
    if channel == 1:
      try:
        self.bus.write_byte(self.addr, 0)
        time.sleep(self.time_waiting)
        self.prev_ch1 = self.bus.read_byte(self.addr)
      except IOError:
        print("ATmega CH1 I/O error")
      return self.prev_ch1
    elif channel == 2:
      try:
        self.bus.write_byte(self.addr, 1)
        time.sleep(self.time_waiting)
        self.prev_ch2 = self.bus.read_byte(self.addr)
      except IOError:
        print("ATmega CH2 I/O error")
      return self.prev_ch2
    elif channel == 3:
      try:
        self.bus.write_byte(self.addr, 2)
        time.sleep(self.time_waiting)
        self.prev_ch3 = self.bus.read_byte(self.addr)
      except IOError:
        print("ATmega CH3 I/O error")
      return self.prev_ch3
    elif channel == 4:
      try:
        self.bus.write_byte(self.addr, 3)
        time.sleep(self.time_waiting)
        self.prev_ch4 = self.bus.read_byte(self.addr)
      except IOError:
        print("ATmega CH4 I/O error")
      return self.prev_ch4
    elif channel == 5:
      try:
        self.bus.write_byte(self.addr, 4)
        time.sleep(self.time_waiting)
        self.prev_voltage = self.bus.read_byte(self.addr)
      except IOError:
        print("ATmega Voltage I/O error")
      return self.prev_voltage
    else:
      return -1
    