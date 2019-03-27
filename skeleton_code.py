# Sketelon Code

#import all the libraries
import RPi.GPIO as GPIO 	# needed for the interrupt service routine
import wiringpi as wpi
import serial
ser = serial.Serial ("/dev/ttyAMA0") # this will open the serial port for the device


'''
Setup
-----------------------------
Run Clean up function
	* possibly omit this if it is causing trouble

Initialize all the registers

wiringpi.wiringPiSetup()

initialize all the pins
	* example-- wpi.pinMode(6, 1)    -- sets pin 6 to output
	
Turn on an LED to indicate that the setup processing is occuring
	* pinX has already been declared as output
	* wpi.digitalWrite(pinX, 1)

calibrate the imu
	* good example in Brooking's flight code

wait until receiver is active and the throttle is set to the lowest position 
and yaw set to far left


Load the battery voltage -- received from the ATMega


When all the setup is done, turn off 

'''

'''
Main Loop
-------------------------------
Start the loop timer
	* If the loop time is longer or shorter than 4000us the angle calculation is off
	* this is very important if we use Brooking's code

generate IMU PID input values
	* see Brooking's code 

calculate PID

read the battery voltage, this is needed for the motors to compensate for low battery

Turn on an LED if the Battery is too low
	* say 3 minutes of flight time left on battery to safely land

If battery is dangerously low, slowly lower the drone to the ground
	* this will need to be a separate function that takes control of the drone
	* lowers slowly to altitude of 2M
	* lowers even slower till it touches the ground
	* turns off the motor and starts beeping

Create the pulses for the ESC's
	* https://youtu.be/fqEkVcqxtU8 
	* send via I2C to the servo shield telling it what PWM values to send to the ESC's

wait till the timer hits 4000us by doing something useful during this time-- tbd


'''

'''
INTERUPT SERVICE ROUTINE
The ATMega will send a pin high causing as interrupt on the Raspberry Pi

This will tell the Raspberry Pi to read the SPI Rx Port 
and update the input channel values

On the ATMega's end, it needs to read the new receiver channel values 
and determine if they changed. If they have changed, send em to the Pi
And signal that you have sent new data by making an I/O pin go high 
this will trigger this routine




'''

