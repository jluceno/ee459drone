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

## SET THESE ===================================================================

## Max angles a user can set as the goal
roll_max = 10
pitch_max = 10
yaw_max = 1

## PID setup
pid_roll = PID.PID(4, 1, 0)
pid_pitch = PID.PID(4, 1, 0)
pid_yaw = PID.PID(4, 0.02, 0)
last_input = "4 1 0"
## Flags
## Change this if you want to use motor mixing or PID control
is_PID_control = True

## Params
## Used to determine the max throttle in motor mixing control
max_value = 150

## Set sensitivity of the sticks. Only affects motor mixing.
throttle_sensitivity = 1
roll_sensitivity = 0.5
pitch_sensitivity = 0.5
yaw_sensitivity = 1

## What is the default angle?.
roll_baseline = 0
pitch_baseline = 0
yaw_baseline = 0

## filter stuff
dt = 0.08

## Current sea level pressure
sealevel = 1015

## END SET THESE ===============================================================

print("Starting PI drone ...")

## BMP setup
print("BMP setup ...")
bmp388_set = False
bmp388_dev = None

while not bmp388_set:
  try:
    bmp388_dev = bmp388.DFRobot_BMP388_I2C()
    bmp388_set = True
    time.sleep(3)
  except IOError:
    print("Retrying BMP setup ...")

## ATmega setup
print("ATmega setup ...")
atmega = atmega.atmega()

## IMU setup
print("IMU setup ...")
i2c_imu = None
sensor = None
imu_set = False

while not imu_set:
  try:
    i2c_imu = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c_imu)
    imu_set = True
  except IOError:
    print("Retrying IMU setup ...")


## Servo hat setup
print("HAT setup ...")
servohat = None
hat_set = False

while not hat_set:
  try:
    servohat = hat_motors.hatservo(500, 0, 1, 2, 3)
    hat_set = True
  except IOError:
    print("Retrying HAT setup ...")


## Globals
pie = math.pi
declination = -11.93 # for IMU Yaw calculation
prev_pres = 0
prev_altitude = 0
prev_temp = 0

## FUNCTIONS ===================================================================

def calibrateIMU(numLoops):
  ax_avg = 0
  ay_avg = 0
  az_avg = 0
  mx_avg = 0
  my_avg = 0
  mz_avg = 0
  gx_avg = 0
  gy_avg = 0
  gz_avg = 0
  for i in range(0,numLoops):
    # read values 
    accel_x, accel_y, accel_z = sensor.acceleration
    mag_x, mag_y, mag_z = sensor.magnetic
    gyro_x, gyro_y, gyro_z = sensor.gyro
    # add to averages
    ax_avg += accel_x
    ay_avg += accel_y
    az_avg += accel_z
    mx_avg += mag_x
    my_avg += mag_y
    mz_avg += mag_z
    gx_avg += gyro_x
    gy_avg += gyro_y
    gz_avg += gyro_z
  ax_avg /= numLoops
  ay_avg /= numLoops
  az_avg /= numLoops
  mx_avg /= numLoops
  my_avg /= numLoops
  mz_avg /= numLoops
  gx_avg /= numLoops
  gy_avg /= numLoops
  gz_avg /= numLoops
  # for debugging purposes -- 
  print(ax_avg)
  print(ay_avg)
  print(az_avg)
  print(mx_avg)
  print(my_avg)
  print(mz_avg)
  print(gx_avg)
  print(gy_avg)
  print(gz_avg)
  accelerations = []
  accelerations.append(ax_avg)
  accelerations.append(ay_avg)
  accelerations.append(az_avg)
  mags = []
  mags.append(mx_avg)
  mags.append(my_avg)
  mags.append(mz_avg)
  gyros = []
  gyros.append(gx_avg)
  gyros.append(gy_avg)
  gyros.append(gz_avg)
  retval = []
  retval.append(accelerations)
  retval.append(mags)
  retval.append(gyros)
  return retval

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
  ## Setup LEDs
  print("LED setup ...")
  setupLEDs()

## END FUNCTIONS ===============================================================
## Begin program ===============================================================
try:
  setup()
  
  # ## Setup IMU
  # print("Calibrating IMU ...")
  # imu_vals = None
  # imu_calibrated = False

  # while not imu_calibrated:
  #   try:
  #     imu_vals = calibrateIMU(20000)
  #     imu_calibrated = True
  #   except IOError:
  #     print("Retrying IMU calibration ...")

  ax_calibration = -0.3509821501756853
  ay_calibration = -0.10137417272018709
  az_calibration = -0.004894602377033763 - 9.297254987947468

  # print("ax_calibration: ", str(ax_calibration))
  # print("ay_calibration: ", str(ay_calibration))
  # print("az_calibration: ", str(az_calibration))

  mx_calibration = -0.5304085779999891
  my_calibration = 0.34171776800001596
  mz_calibration = -0.2928921099999922

  # print("mx_calibration: ", str(mx_calibration))
  # print("my_calibration: ", str(my_calibration))
  # print("mz_calibration: ", str(mz_calibration))

  gx_calibration = 0.0663263125000002
  gy_calibration = 1.7302792499999993
  gz_calibration = 1.178446500000009

  # print("gx_calibration: ", str(gx_calibration))
  # print("gy_calibration: ", str(gy_calibration))
  # print("gz_calibration: ", str(gz_calibration))

  accel_x, accel_y, accel_z = sensor.acceleration
  mag_x, mag_y, mag_z = sensor.magnetic
  gyro_x, gyro_y, gyro_z = sensor.gyro

  accel_x -= ax_calibration
  accel_y -= ay_calibration
  accel_z -= az_calibration
  mag_x -= mx_calibration
  mag_y -= my_calibration
  mag_z -= mz_calibration
  gyro_x -= gx_calibration
  gyro_y -= gy_calibration
  gyro_z -= gz_calibration

  imu_pitch = -180 * math.atan2(accel_x, math.sqrt(accel_y*accel_y + accel_z*accel_z))/ pie
  imu_roll = -180 * math.atan2(accel_y, math.sqrt(accel_x*accel_x + accel_z*accel_z))/ pie
  imu_yaw = 0
  imu_pitch_last = imu_pitch
  imu_roll_last = imu_roll

  ## Main loop
  while True:
    time_init = time.time()
    ## Read the sensor values ====================================================

    # Read in the IMU values
    accel_x, accel_y, accel_z = sensor.acceleration
    mag_x, mag_y, mag_z = sensor.magnetic
    gyro_x, gyro_y, gyro_z = sensor.gyro

    accel_x -= ax_calibration
    accel_y -= ay_calibration
    accel_z -= az_calibration
    mag_x -= mx_calibration
    mag_y -= my_calibration
    mag_z -= mz_calibration
    gyro_x -= gx_calibration
    gyro_y -= gy_calibration
    gyro_z -= gz_calibration

    # print("ax: ", accel_x)
    # print("ay: ", accel_y)
    # print("az: ", accel_z)
    # print("gx: ", gyro_x)
    # print("gy: ", gyro_y)
    # print("gz: ", gyro_z)
    # vals = calc_Roll_Pitch_Yaw(accel_x, accel_y, accel_z, mag_x, mag_y, mag_z)

    # imu_pitch = vals[0]
    # imu_roll = vals[1]
    # imu_yaw = vals[2]

    imu_pitch = -180 * math.atan2(accel_x, math.sqrt(accel_y*accel_y + accel_z*accel_z))/ pie
    imu_roll = -180 * math.atan2(accel_y, math.sqrt(accel_x*accel_x + accel_z*accel_z))/ pie
    imu_yaw = gyro_z

    print("pre filter----------- ")
    print("roll: ", imu_roll)
    print("pitch: ", imu_pitch)

    imu_pitch = (0.99 * (imu_pitch_last +(dt *gyro_y))) + (0.01 * imu_pitch)
    imu_roll = (0.99 * (imu_roll_last + (dt*gyro_x))) + (0.01 * imu_roll)

    # imu_pitch *= -1
    # imu_roll *= -1

    imu_pitch_last = imu_pitch
    imu_roll_last = imu_roll

    print("post filter----------- ")
    print("roll: ", imu_roll)
    print("gyro_x: ", gyro_x)
    print("pitch: ", imu_pitch)
    print("gyro_y: ", gyro_y)
    # print("imu_pitch: ", imu_pitch)
    # print("imu_roll: ", imu_roll)
    # print("imu_yaw: ", imu_yaw)

    ## Read the BMP values
    pres = 0
    temp = 0
    altitude = 0
    try:
      pres = bmp388_dev.readPressure()
      temp = bmp388_dev.readTemperature()
      altitude = bmp388_dev.readCalibratedAltitude(sealevel)
      prev_pres = pres
      prev_temp = temp
      prev_altitude = altitude
    except IOError:
      pres = prev_pres
      temp = prev_temp
      altitude = prev_altitude

    # print("Temperature: %s C" %temp)
    # print("Pressure : %s Pa" %pres)
    # print("Altitude :%s m" %altitude)

    ## Get user input 
    roll = 50 - atmega.get_data(1)
    pitch = 50 - atmega.get_data(2)
    throttle = atmega.get_data(3)
    yaw = 50 - atmega.get_data(4)

    if (abs(roll) < 2):
      roll = 0
    if (abs(pitch) < 2):
      pitch = 0
    if (abs(yaw) < 2):
      yaw = 0

    ## Calculate PID values or motor mix =========================================

    if is_PID_control:
      ## PID control
      pid_roll.SetPoint = (roll_max * roll/50)
      pid_pitch.SetPoint = (pitch_max * pitch/50)
      pid_yaw.SetPoint = (yaw_max * yaw/50)

      pid_roll.update(imu_roll)
      pid_pitch.update(imu_pitch)
      pid_yaw.update(gyro_z)

      motor1 = -pid_pitch.output + pid_roll.output - pid_yaw.output #ESC 1, front-left: CCW
      motor2 = -pid_pitch.output - pid_roll.output  + pid_yaw.output #ESC 2, front-right: CW
      motor3 = pid_pitch.output - pid_roll.output - pid_yaw.output #ESC 3, rear-right: CCW
      motor4 = pid_pitch.output + pid_roll.output + pid_yaw.output #ESC 4, rear-left: CW

      throttle *= throttle_sensitivity

      #print(motor1, motor2, motor3, motor4)

      duty_cycle1 = int(((throttle)/100 * 0x7fff) + motor1)+0x7fff
      duty_cycle2 = int(((throttle)/100 * 0x7fff) + motor2)+0x7fff
      duty_cycle3 = int(((throttle)/100 * 0x7fff) + motor3)+0x7fff
      duty_cycle4 = int(((throttle)/100 * 0x7fff) + motor4)+0x7fff
    else:
      ## Motor mixing
      throttle *= throttle_sensitivity
      roll *= roll_sensitivity
      pitch *= pitch_sensitivity
      yaw *= yaw_sensitivity
        
      motor1 = throttle - pitch + roll - yaw #ESC 1, front-left: CCW
      motor2 = throttle - pitch - roll  + yaw #ESC 2, front-right: CW
      motor3 = throttle + pitch - roll - yaw #ESC 3, rear-right: CCW
      motor4 = throttle + pitch + roll + yaw #ESC 4, rear-left: CW
      
      duty_cycle1 = int(motor1/max_value * 0x7fff)+0x7fff
      duty_cycle2 = int(motor2/max_value * 0x7fff)+0x7fff
      duty_cycle3 = int(motor3/max_value * 0x7fff)+0x7fff
      duty_cycle4 = int(motor4/max_value * 0x7fff)+0x7fff

    ## Output battery status
    battery_perc = (int(atmega.get_data(5)) - 154)/(189-154) * 100
    #print("Battery: ", battery_perc)

    if battery_perc > 50:
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

    if throttle < 3:
      duty_cycle1 = 0x7fff
      duty_cycle2 = 0x7fff
      duty_cycle3 = 0x7fff
      duty_cycle4 = 0x7fff
      servohat.motorset(servohat.motor1, min(0xffff, max(0x7fff, duty_cycle1)))
      servohat.motorset(servohat.motor2, min(0xffff, max(0x7fff, duty_cycle2)))
      servohat.motorset(servohat.motor3, min(0xffff, max(0x7fff, duty_cycle3)))
      servohat.motorset(servohat.motor4, min(0xffff, max(0x7fff, duty_cycle4)))
      usr_input = "n"
      while usr_input == "n":
        time.sleep(2)

        ## gains args: <roll/pitch p> <roll/pitch i> <roll/pitch d> <yaw p> <yaw i> <yaw d>
        print("last inputted values: ")
        print(last_input)
        usr_input = input("Set roll / pitch PID values: P I D ")
        last_input = usr_input
        gains = usr_input.split(" ")
        pid_roll.setKp(float(gains[0]))
        pid_roll.setKi(float(gains[1]))
        pid_roll.setKd(float(gains[2]))
        pid_pitch.setKp(float(gains[0]))
        pid_pitch.setKi(float(gains[1]))
        pid_pitch.setKd(float(gains[2]))
        # pid_yaw.setKp(float(gains[3]))
        # pid_yaw.setKi(float(gains[4]))
        # pid_yaw.setKd(float(gains[5]))

    #print("Duty cycles: ", round((duty_cycle1 - 0x7fff)/0xffff * 100), round((duty_cycle2 - 0x7fff)/0xffff * 100),
      #round((duty_cycle3 - 0x7fff)/0xffff * 100), round((duty_cycle4 - 0x7fff)/0xffff * 100))

    ## Output values to ESCs
    ## Duty cycle can be set to 0x0000 to 0xffff
    ## ESCs values: 0x7fff = 0%, 0xffff 100%
    servohat.motorset(servohat.motor1, min(0xffff, max(0x7fff, duty_cycle1)))
    servohat.motorset(servohat.motor2, min(0xffff, max(0x7fff, duty_cycle2)))
    servohat.motorset(servohat.motor3, min(0xffff, max(0x7fff, duty_cycle3)))
    servohat.motorset(servohat.motor4, min(0xffff, max(0x7fff, duty_cycle4)))

    time_f = time.time()
    time_d = time_f - time_init
    print("Time difference: ", time_d)

    ## Loop again
except KeyboardInterrupt:
  print("Motors off!")
  time.sleep(1)
  servohat.motorset(servohat.motor1, 0)
  servohat.motorset(servohat.motor2, 0)
  servohat.motorset(servohat.motor3, 0)
  servohat.motorset(servohat.motor4, 0)
