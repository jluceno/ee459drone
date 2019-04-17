import bmp388
import time
import atmega
import hat_motors

bmp388 = bmp388.DFRobot_BMP388_I2C()
time.sleep(0.5)
atmega = atmega.atmega()
servohat = hat_motors.hatservo(500, 0, 1, 2, 3)

## TODO
##def calibrateIMU():


def setupHat():
  ## Set the input low -> high -> low
  servohat.motorset(servohat.motor1, 0x7fff)
  servohat.motorset(servohat.motor2, 0x7fff)
  servohat.motorset(servohat.motor3, 0x7fff)
  servohat.motorset(servohat.motor4, 0x7fff)
  time.sleep(0.5)
  servohat.motorset(servohat.motor1, 0xffff)
  servohat.motorset(servohat.motor2, 0xffff)
  servohat.motorset(servohat.motor3, 0xffff)
  servohat.motorset(servohat.motor4, 0xffff)
  time.sleep(0.5)
  servohat.motorset(servohat.motor1, 0x7fff)
  servohat.motorset(servohat.motor2, 0x7fff)
  servohat.motorset(servohat.motor3, 0x7fff)
  servohat.motorset(servohat.motor4, 0x7fff)


def calibrateBMP(sea_level):
  bmp388.readCalibratedAltitude(sea_level)


def setup():
  ## Wait 10 seconds for the user to calibrate the ATmega
  time.sleep(2)
  
  ## Setup bmp388
  calibrateBMP(512)

  ## Setup hat
  setupHat()

## Begin program ===============================================================

setup()

## Main loop
while True:
  ## For later:
  ## Read the sensor values
  ## Calculate PID values

  ## For now:
  ## Get user input
  roll = 50 - atmega.get_data(1)
  time.sleep(0.05)
  pitch = 50 - atmega.get_data(2)
  time.sleep(0.05)
  throttle = atmega.get_data(3)
  time.sleep(0.05)
  yaw = 50 - atmega.get_data(4)
  time.sleep(0.05)

  print(roll, pitch, throttle, yaw)

  ## Motor mixing
  ## Not sure about yaw (just a guess)
  '''
  motor0 = throttle + roll - pitch + yaw
  motor1 = throttle - roll - pitch - yaw
  motor2 = throttle - roll + pitch + yaw
  motor3 = throttle + roll + pitch - yaw
  '''
  motor1 = throttle - pitch + roll - yaw #ESC 1, front-left: CCW
  motor2 = throttle - pitch - roll  + yaw #ESC 2, front-right: CW
  motor3 = throttle + pitch - roll - yaw #ESC 3, rear-right: CCW
  motor4 = throttle + pitch + roll + yaw #ESC 4, rear-left: CW

  ## Output values to ESCs

  ## Does not use motor mixing.
  ## Just outputs throttle input directly to ESCs
  ## Duty cycle can be set to 0x0000 to 0xffff
  ## ESCs values: 0x7fff = 0%, 0xffff 100%
  duty_cycle = int(throttle/100 * 0x7fff)+0x7fff

  if (duty_cycle > 0xffff):
    duty_cycle = 0xffff

  servohat.motorset(servohat.motor1, duty_cycle)
  servohat.motorset(servohat.motor2, duty_cycle)
  servohat.motorset(servohat.motor3, duty_cycle)
  servohat.motorset(servohat.motor4, duty_cycle)

  ## Loop again


