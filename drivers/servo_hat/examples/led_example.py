import board
import busio 
#Import PCA9685 module 
import adafruit_pca9685

#Create the I2C bus interface
i2c = busio.I2C(board.SCL, board.SDA)

#Create a simple PCA9685 class instance
hat = adafruit_pca9685.PCA9685(i2c)

#set board's pwm frequency
hat.frequency = 60

#set different channels for different outputs 
led_channel = hat.channels[0]

#duty cycle 16-bit value - this means 100%
led_channel.duty_cycle = 0xffff

#This is 0%
led_channel.duty_cycle = 0

#in-between value
led_channel.duty_cycle = 1000

#Incrase brightness:
for i in range(0xffff):
	led_channel.duty_cycle = i

#Decrease brightness:
for i in range(0xffff, 0, -1):
	led_channel.duty_cycle = i

