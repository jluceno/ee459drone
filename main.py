import bmp388
import time
import atmega

bmp388 = bmp388.DFRobot_BMP388_I2C()
atmega = atmega.atmega()

## TODO
##def calibrateIMU():

## TODO
##def setupHat():

## TODO
##def calibrateESC():

def calibrateBMP(sea_level):
  bmp388.readCalibratedAltitude(sea_level)


def setup():
  ## Wait 10 seconds for the user to calibrate the ATmega
  time.sleep(10)
  
  ## Setup bmp388
  time.sleep(0.5)
  calibrateBMP(0)

## Begin program ===============================================================

setup()

## Main loop
while True:
  ## For later:
  ## Read the sensor values
  ## Calculate PID values

  ## For now:
  ## Get user input
  roll = 50 - atmega.get_throttle(1)
  pitch = 50 - atmega.get_throttle(2)
  throttle = atmega.get_throttle(3)
  yaw = 50 - atmega.get_throttle(4)

  ## Motor mixing
  ## Not sure about yaw (just a guess)
  motor0 = throttle + roll - pitch + yaw
  motor1 = throttle - roll - pitch - yaw
  motor2 = throttle - roll + pitch + yaw
  motor3 = throttle + roll + pitch - yaw

  ## Output values to ESCs


  ## Loop again


