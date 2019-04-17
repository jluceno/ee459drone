import bmp388
import time
import atmega
import board
import busio
import adafruit_lsm9ds1
from picamera import PiCamera
import sys
import math
import PID

# objects of classes here
bmp388 = bmp388.DFRobot_BMP388_I2C()
atmega = atmega.atmega()
# I2C setup for the IMU
i2c_imu = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c_imu)

# camera instances & camera setup
camera = PiCamera() 
camera.rotation = 180
camera.annotate_text = "EE 459 Capstone Project"


#########################################################################
## calibrateIMU(): 
## Purpose: calibrate the IMU by taking a ton of readings then averaging them
## NOTE: ommitted testing of magnetometer because we dont need it
## input: number of loops to take the average from-- increasing numLoops increases accuracy. 
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


## end calibrateIMU function
#########################################################################

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
  #calibrateBMP(0)
  



## Begin program ===============================================================

setup()

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


accel_x, accel_y, accel_z = sensor.acceleration
mag_x, mag_y, mag_z = sensor.magnetic

accel_x -= ax_calibration
accel_y -= ay_calibration
accel_z -= az_calibration
mag_x -= mx_calibration
mag_y -= my_calibration
mag_z -= mz_calibration

imu_pitch = 180 * math.atan2(accel_x, math.sqrt(accel_y*accel_y + accel_z*accel_z))/math.pi;
imu_roll = 180 * math.atan2(accel_y, math.sqrt(accel_x*accel_x + accel_z*accel_z))/math.pi;

print("imu_pitch: ", imu_pitch)
print("imu_roll: ", imu_roll)

# determine if we are in starting position
roll = 50 - atmega.get_throttle(1)
pitch = 50 - atmega.get_throttle(2)
throttle = atmega.get_throttle(3)
yaw = 50 - atmega.get_throttle(4)

# if #TODO: find starting conditions PWM acceptable ranges


# start the video recording
time.sleep(2)

iterator = 0

## TODO: implement analog reading of battery voltage
## Main loop
while True:

  
  usr_in = input("capture an image? [y/n]")
  if(usr_in == "y"):
    print("capturing image")
    iterator += 1
    img_file_name = "/home/pi/ee459drone/media/image" + str(iterator) + ".jpg"
    camera.capture(img_file_name)
    print("done")
  
  ## For later:
  ## Read the sensor values
  
  time.sleep(2)
  
  # Read in the IMU values
  accel_x, accel_y, accel_z = sensor.acceleration
  gyro_x, gyro_y, gyro_z = sensor.gyro

  accel_x -= ax_calibration
  accel_y -= ay_calibration
  accel_z -= az_calibration
  gyro_x -= gx_calibration
  gyro_y -= gy_calibration
  gyro_z -= gz_calibration

  print("ax: ", accel_x)
  print("ay: ", accel_y)
  print("az: ", accel_z)
  print("gx: ", gyro_x)
  print("gy: ", gyro_y)
  print("gz: ", gyro_z)

  ## Calculate PID values

  ## For now:
  ## Get user input
  
  roll = 50 - atmega.get_throttle(1)
  pitch = 50 - atmega.get_throttle(2)
  throttle = atmega.get_throttle(3)
  yaw = 50 - atmega.get_throttle(4)

  print("roll: ", roll)
  print("pitch: ", pitch)
  print("throttle: ", throttle)
  print("yaw:", yaw)

  ## Motor mixing
  ## Not sure about yaw (just a guess)
  motor0 = throttle + roll - pitch + yaw
  motor1 = throttle - roll - pitch - yaw
  motor2 = throttle - roll + pitch + yaw
  motor3 = throttle + roll + pitch - yaw

  print("m0: ", motor0)
  print("m1: ", motor1)
  print("m2: ", motor2)
  print("m3: ", motor3)

  print("end of loop")

  ## Output values to ESCs

  ## Loop again