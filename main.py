import bmp388
import time
import atmega
import hat_motors
import board
import busio
import adafruit_lsm9ds1
import RPi.GPIO as GPIO
import sys
import math
import PID

print("Starting PI drone ...")

## BMP setup
print("BMP setup ...")
bmp388 = bmp388.DFRobot_BMP388_I2C()
time.sleep(3)

## ATmega setup
print("ATmega setup ...")
atmega = atmega.atmega()

## IMU setup
print("IMU setup ...")
i2c_imu = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c_imu)

## Servo hat setup
print("HAT setup ...")
servohat = hat_motors.hatservo(500, 0, 1, 2, 3)

## PID setup
pid_roll = PID.PID(0.2, 0, 0)
pid_pitch = PID.PID(0.2, 0, 0)
pid_yaw = PID.PID(0.2, 0, 0)

## Globals
roll_baseline = 0
pitch_baseline = 0
yaw_baseline = 0

## Max angles a user can input
roll_max = 10
pitch_max = 10
yaw_max = 10

## Flags
## Change this if you want to use motor mixing or PID control
is_PID_control = False

## Params
## Used to determine the max throttle in motor mixing control
max_value = 150

def calibrateIMU(numLoops):
  ax_avg = 0
  ay_avg = 0
  az_avg = 0
  mx_avg = 0
  my_avg = 0
  mz_avg = 0
  for i in range(0,numLoops):
    # read values 
    accel_x, accel_y, accel_z = sensor.acceleration
    mag_x, mag_y, mag_z = sensor.magnetic
    # gyro_x, gyro_y, gyro_z = sensor.gyro
    # add to averages
    ax_avg += accel_x
    ay_avg += accel_y
    az_avg += accel_z
    mx_avg += mag_x
    my_avg += mag_y
    mz_avg += mag_z
    # gx_avg += gyro_x
    # gy_avg += gyro_y
    # gz_avg += gyro_z
  ax_avg /= numLoops
  ay_avg /= numLoops
  az_avg /= numLoops
  mx_avg /= numLoops
  my_avg /= numLoops
  mz_avg /= numLoops
  # gx_avg /= numLoops
  # gy_avg /= numLoops
  # gz_avg /= numLoops
  # for debugging purposes -- 
  print(ax_avg)
  print(ay_avg)
  print(az_avg)
  print(mx_avg)
  print(my_avg)
  print(mz_avg)
  # print(gx_avg)
  # print(gy_avg)
  # print(gz_avg)
  accelerations = []
  accelerations.append(ax_avg)
  accelerations.append(ay_avg)
  accelerations.append(az_avg)
  mags = []
  mags.append(mx_avg)
  mags.append(my_avg)
  mags.append(mz_avg)
  retval = []
  retval.append(accelerations)
  retval.append(mags)
  return retval

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
  ## Setup bmp388
  print("Calibrating BMP ...")
  calibrateBMP(512)
  

  ## Setup LEDs
  print("Setting up LEDs ...")
  setupLEDs()


## Begin program ===============================================================

setup()

## Setup IMU
print("Calibrating IMU ...")
imu_vals = calibrateIMU(5000)

print(imu_vals)

ax_calibration = imu_vals[0][0]
ay_calibration = imu_vals[0][1]
az_calibration = imu_vals[0][2]

print("ax_calibration: ", str(ax_calibration))
print("ay_calibration: ", str(ay_calibration))
print("az_calibration: ", str(az_calibration))

mx_calibration = imu_vals[1][0]
my_calibration = imu_vals[1][1]
mz_calibration = imu_vals[1][2]

print("mx_calibration: ", str(mx_calibration))
print("my_calibration: ", str(my_calibration))
print("mz_calibration: ", str(mz_calibration))

## Main loop
while True:
  ## Read the sensor values ====================================================

  # Read in the IMU values
  accel_x, accel_y, accel_z = sensor.acceleration
  mag_x, mag_y, mag_z = sensor.magnetic

  accel_x -= ax_calibration
  accel_y -= ay_calibration
  accel_z -= az_calibration
  mag_x -= mx_calibration
  mag_y -= my_calibration
  mag_z -= mz_calibration
  '''
  print("ax: ", accel_x)
  print("ay: ", accel_y)
  print("az: ", accel_z)
  print("gx: ", gyro_x)
  print("gy: ", gyro_y)
  print("gz: ", gyro_z)
  '''
  imu_pitch = 180 * math.atan2(accel_x, math.sqrt(accel_y*accel_y + accel_z*accel_z))/math.pi
  imu_roll = 180 * math.atan2(accel_y, math.sqrt(accel_x*accel_x + accel_z*accel_z))/math.pi
  imu_yaw = 180 * math.atan2(accel_z, math.sqrt(accel_x*accel_x + accel_z* accel_z))/math.pi
  
  print("imu_pitch: ", imu_pitch)
  print("imu_roll: ", imu_roll)
  print("imu_yaw: ", imu_yaw)

  ## Read the BMP values
  pres = bmp388.readPressure() 
  temp = bmp388.readTemperature()
  altitude = bmp388.readAltitude()

  print("Temperature: %s C" %temp)
  print("Pressure : %s Pa" %pres)
  print("Altitude :%s m" %altitude)

  ## Get user input 
  roll = 50 - atmega.get_data(1)
  pitch = 50 - atmega.get_data(2)
  throttle = atmega.get_data(3)
  yaw = 50 - atmega.get_data(4)

  ## Calculate PID values or motor mix =========================================
  
  if is_PID_control:
    ## PID control
    pid_roll.SetPoint = (roll_max * roll/50)
    pid_pitch.SetPoint = (pitch_max * pitch/50)
    ##pid_yaw.SetPoint = yaw_baseline + (yaw_max * yaw/50)

    pid_roll.update(imu_roll)
    pid_pitch.update(imu_pitch)
    ##pid_yaw.update(None)

    motor1 = pid_pitch.output + pid_roll.output - yaw #ESC 1, front-left: CCW
    motor2 = pid_pitch.output - pid_roll.output  + yaw #ESC 2, front-right: CW
    motor3 = pid_pitch.output - pid_roll.output - yaw #ESC 3, rear-right: CCW
    motor4 = pid_pitch.output + pid_roll.output + yaw #ESC 4, rear-left: CW
  
    duty_cycle1 = int((throttle/100 * 0x7fff) + motor1)+0x7fff
    duty_cycle2 = int((throttle/100 * 0x7fff) + motor2)+0x7fff
    duty_cycle3 = int((throttle/100 * 0x7fff) + motor3)+0x7fff
    duty_cycle4 = int((throttle/100 * 0x7fff) + motor4)+0x7fff
  else:
    ## Motor mixing
    motor1 = throttle - pitch + roll - yaw #ESC 1, front-left: CCW
    motor2 = throttle - pitch - roll  + yaw #ESC 2, front-right: CW
    motor3 = throttle + pitch - roll - yaw #ESC 3, rear-right: CCW
    motor4 = throttle + pitch + roll + yaw #ESC 4, rear-left: CW
    
    duty_cycle1 = int(motor1/max_value * 0x7fff)+0x7fff
    duty_cycle2 = int(motor2/max_value * 0x7fff)+0x7fff
    duty_cycle3 = int(motor3/max_value * 0x7fff)+0x7fff
    duty_cycle4 = int(motor4/max_value * 0x7fff)+0x7fff

  ## Output battery status
  battery_perc = int(atmega.get_data(5)) - 154
  print("Battery: ", battery_perc)
  
  if battery_perc > 45:
    GPIO.output(4,GPIO.HIGH)
    GPIO.output(17,GPIO.HIGH)
    GPIO.output(18,GPIO.HIGH)
  elif battery_perc > 30:
    GPIO.output(4,GPIO.HIGH)
    GPIO.output(17,GPIO.HIGH)
    GPIO.output(18,GPIO.LOW)
  elif battery_perc < 10: 
    GPIO.output(4,GPIO.HIGH)
    GPIO.output(17,GPIO.LOW)
    GPIO.output(18,GPIO.LOW)

  print("Duty cycles: ", duty_cycle1, duty_cycle2, duty_cycle3, duty_cycle4)

  ## Output values to ESCs
  ## Duty cycle can be set to 0x0000 to 0xffff
  ## ESCs values: 0x7fff = 0%, 0xffff 100%
  servohat.motorset(servohat.motor1, min(0xffff, max(0x7fff, duty_cycle1)))
  servohat.motorset(servohat.motor2, min(0xffff, max(0x7fff, duty_cycle2)))
  servohat.motorset(servohat.motor3, min(0xffff, max(0x7fff, duty_cycle3)))
  servohat.motorset(servohat.motor4, min(0xffff, max(0x7fff, duty_cycle4)))

  ## Loop again


