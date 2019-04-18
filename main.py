import bmp388
import time
import atmega
import hat_motors
import RPi.GPIO as GPIO

bmp388 = bmp388.DFRobot_BMP388_I2C()
time.sleep(3)
atmega = atmega.atmega()
servohat = hat_motors.hatservo(500, 0, 1, 2, 3)

max_value = 150

## TODO
##def calibrateIMU():

def calibrateBMP(sea_level):
  bmp388.readCalibratedAltitude(sea_level)

def setupLEDs():
  GPIO.setmode(GPIO.BCM)
  GPIO.setwarnings(False)
  GPIO.setup(4,GPIO.OUT)
  GPIO.setup(17,GPIO.OUT)
  GPIO.setup(18,GPIO.OUT)
  GPIO.output(4,GPIO.LOW)
  GPIO.output(17,GPIO.LOW)
  GPIO.output(18,GPIO.LOW)


def setup():
  ## Wait 10 seconds for the user to calibrate the ATmega
  time.sleep(2)
  
  ## Setup bmp388
  calibrateBMP(512)

  ## Setup LEDs
  setupLEDs()

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
  pitch = 50 - atmega.get_data(2)
  throttle = atmega.get_data(3)
  yaw = 50 - atmega.get_data(4)

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

  ## Just outputs throttle input directly to ESCs
  ## Duty cycle can be set to 0x0000 to 0xffff
  ## ESCs values: 0x7fff = 0%, 0xffff 100%
  duty_cycle1 = int(motor1/max_value * 0x7fff)+0x7fff
  duty_cycle2 = int(motor2/max_value * 0x7fff)+0x7fff
  duty_cycle3 = int(motor3/max_value * 0x7fff)+0x7fff
  duty_cycle4 = int(motor4/max_value * 0x7fff)+0x7fff

  print(duty_cycle1, duty_cycle2, duty_cycle3, duty_cycle4)

  servohat.motorset(servohat.motor1, min(0xffff, max(0x7fff, duty_cycle1)))
  servohat.motorset(servohat.motor2, min(0xffff, max(0x7fff, duty_cycle2)))
  servohat.motorset(servohat.motor3, min(0xffff, max(0x7fff, duty_cycle3)))
  servohat.motorset(servohat.motor4, min(0xffff, max(0x7fff, duty_cycle4)))

  ## Output battery status
  battery_perc = int(atmega.get_data(5))
  
  if battery_perc > 75:
    GPIO.output(4,GPIO.HIGH)
    GPIO.output(17,GPIO.HIGH)
    GPIO.output(18,GPIO.HIGH)
  elif battery_perc > 50:
    GPIO.output(4,GPIO.HIGH)
    GPIO.output(17,GPIO.HIGH)
    GPIO.output(18,GPIO.LOW)
  elif battery_perc > 0: 
    GPIO.output(4,GPIO.HIGH)
    GPIO.output(17,GPIO.LOW)
    GPIO.output(18,GPIO.LOW)


  ## Loop again


