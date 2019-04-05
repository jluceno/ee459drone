# Sketelon Code

#import all the libraries
import RPi.GPIO as GPIO 	# needed for the interrupt service routine


ser = serial.Serial ("/dev/ttyAMA0") # this will open the serial port for the device

# declare global variables
LED_PIN = #TODO: declare the LED Pin number

pid_p_gain_roll = 1.3               	#Gain setting for the roll P-controller
pid_i_gain_roll = 0.04              	#Gain setting for the roll I-controller
pid_d_gain_roll = 18.0              	#Gain setting for the roll D-controller
pid_max_roll = 400                   	#Maximum output of the PID-controller (+/-)

pid_p_gain_pitch = pid_p_gain_roll  	#Gain setting for the pitch P-controller.
pid_i_gain_pitch = pid_i_gain_roll  	#Gain setting for the pitch I-controller.
pid_d_gain_pitch = pid_d_gain_roll  	#Gain setting for the pitch D-controller.
pid_max_pitch = pid_max_roll          	#Maximum output of the PID-controller (+/-)

pid_p_gain_yaw = 4.0                	#Gain setting for the pitch P-controller. //4.0
pid_i_gain_yaw = 0.02               	#Gain setting for the pitch I-controller. //0.02
pid_d_gain_yaw = 0.0                	#Gain setting for the pitch D-controller.
pid_max_yaw = 400                     	#Maximum output of the PID-controller (+/-)

auto_level = True 						# True means auto-level is on

last_channel_1, last_channel_2, last_channel_3, last_channel_4
receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4
counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter
esc_1, esc_2, esc_3, esc_4
throttle, battery_voltage
cal_int, start, gyro_address
receiver_input
temperature
acc_axis
gyro_axis
roll_level_adjust, pitch_level_adjust

acc_x, acc_y, acc_z, acc_total_vector
timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer
timer_1, timer_2, timer_3, timer_4, current_time
loop_timer
gyro_pitch, gyro_roll, gyro_yaw
gyro_axis_cal
pid_error_temp
pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error
pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error
pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error
angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll
gyro_angles_set
# import all the libraries
# declare instances of everything we need
# initialize all serial communications 
# initalize all the pins as input / output




'''
Setup
-----------------------------
Run Clean up function
	* possibly omit this if it is causing trouble

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

def calibrateIMU():

def calibrateESC():

def calibrateBMP():

# calibrate anything else?


def setup():
	
	# turn on an LED to indicate that we are in setup mode
	wpi.digitalWrite(LED_PIN, 1)

	# Run the clean up function
		# omit this if it is causing trouble

	# calibrate the stuff, good example in Brooking's code

	# read the I2C data from the ATMEGA 


	while ((receiver_input_channel_3 < 990) or (receiver_input_channel_3 > 1020) || (receiver_input_channel_4 < 1400)):
		# convert the receiver signals to 1000us to 2000us standard
	

	# if we get to this point in this function we know that everything went smoothely
	wpi.digitalWrite(LED_PIN, 0) # turn off the LED
	return True



def main():
	# start the loop timer
	# increment loop number
	loop_counter += 1

	# generate IMU PID values

	# calculate PID

	if (loop_counter == 1000):
		# read the battery voltage 
			# I2C read from the AtMega
		if battery_voltage < battery_limit:

		loop_counter = 0

	# 




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

