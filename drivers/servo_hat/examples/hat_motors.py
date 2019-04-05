# necessary libraries 
import board
import busio 
import adafruit_pca9685

def setup(freq, m1, m2, m3, m4):
	#Create the I2C bus interface
	i2c = busio.I2C(board.SCL, board.SDA)

	#Create a simple PCA9685 class instance
	hat = adafruit_pca9685.PCA9685(i2c)

	#set board's pwm frequency (LED tutorial default = 60)
	hat.frequency = freq

	#set different channels for different motors 
	motor1 = hat.channels[m1]
	motor2 = hat.channels[m2]
	motor3 = hat.channels[m3]
	motor4 = hat.channels[m4]

def motorset(motor, val):
	motor.duty_cycle = val

def motoroff(motor):
	motor.duty_cycle = 0

def motorinc(motor, val):
	#gradually increase by 1 to value
	for i in range(val):
		motor.duty_cycle = i

def motordec(motor, val):
	#gradually decrease by 1 to value
	for i in range(val, 0, -1):
		motor.duty_cycle = i
