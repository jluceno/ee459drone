import bmp388
import time
import atmega

bmp388, atmega

## TODO
##def calibrateIMU():

## TODO
##def calibrateESC():

def calibrateBMP(sea_level):
  bmp388.readCalibratedAltitude(sea_level)


def setup():
  ## Wait 10 seconds for the user to calibrate the ATmega
  time.sleep(10)
  atmega = atmega()

  ## Setup bmp388
  bmp388 = bmp388.DFRobot_BMP388_I2C()
  time.sleep(0.5)
  calibrateBMP(0)

## Main loop
while True:
  ## For later:
  ## Read the sensor values
  ## Calculate PID values

  ## For now:
  ## Get user input
  roll = atmega.get_throttle(1)
  pitch = atmega.get_throttle(2)
  throttle = atmega.get_throttle(3)
  yaw = atmega.get_throttle(4)

  
  ## Output values to ESCs
  

main()
