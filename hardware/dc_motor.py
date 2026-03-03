import time
import RPi.GPIO as GPIO

# Define GPIO pins for motor control
Motor_A_EN = 4           			# Enable pin for Motor A
Motor_B_EN = 17          			# Enable pin for Motor B
Motor_A_Pin1 = 26        			# Control pin 1 for Motor A
Motor_A_Pin2 = 21        			# Control pin 2 for Motor A
Motor_B_Pin1 = 27        			# Control pin 1 for Motor B
Motor_B_Pin2 = 18        			# Control pin 2 for Motor B

# Define motor direction constants
Dir_forward = 0          			# Forward direction
Dir_backward = 1         			# Backward direction

# Global variables for PWM objects
pwm_A = 0
pwm_B = 0

# Function to stop both motors
def motorStop():
    # Set all control pins and enable pins to LOW to stop the motors
    GPIO.output(Motor_A_Pin1, GPIO.LOW)
    GPIO.output(Motor_A_Pin2, GPIO.LOW)
    GPIO.output(Motor_B_Pin1, GPIO.LOW)
    GPIO.output(Motor_B_Pin2, GPIO.LOW)
    GPIO.output(Motor_A_EN, GPIO.LOW)
    GPIO.output(Motor_B_EN, GPIO.LOW)

# Function to set up GPIO pins and initialize motors
def setup():
    global pwm_A, pwm_B
    GPIO.setwarnings(False)            		# Disable GPIO warnings
    GPIO.setmode(GPIO.BCM)             		# Use BCM pin numbering
    # Set up GPIO pins as outputs
    GPIO.setup(Motor_A_EN, GPIO.OUT)
    GPIO.setup(Motor_B_EN, GPIO.OUT)
    GPIO.setup(Motor_A_Pin1, GPIO.OUT)
    GPIO.setup(Motor_A_Pin2, GPIO.OUT)
    GPIO.setup(Motor_B_Pin1, GPIO.OUT)
    GPIO.setup(Motor_B_Pin2, GPIO.OUT)

    motorStop()                        		# Stop motors initially

    try:
        # Initialize PWM for Motor A and Motor B with a frequency of 1 kHz
        pwm_A = GPIO.PWM(Motor_A_EN, 1000)
        pwm_B = GPIO.PWM(Motor_B_EN, 1000)
    except:
        pass

# Function to control the left motor (Motor B)
def motor_left(status, direction, speed):
    if status == 0:  				# Stop the motor
        GPIO.output(Motor_B_Pin1, GPIO.LOW)
        GPIO.output(Motor_B_Pin2, GPIO.LOW)
        GPIO.output(Motor_B_EN, GPIO.LOW)
    else:
        if direction == Dir_forward:  		# Set motor to spin forward
            GPIO.output(Motor_B_Pin1, GPIO.HIGH)
            GPIO.output(Motor_B_Pin2, GPIO.LOW)
            pwm_B.start(100)          		# Start PWM with full duty cycle
            pwm_B.ChangeDutyCycle(speed)  	# Adjust speed with duty cycle
        elif direction == Dir_backward:  	# Set motor to spin backward
            GPIO.output(Motor_B_Pin1, GPIO.LOW)
            GPIO.output(Motor_B_Pin2, GPIO.HIGH)
            pwm_B.start(0)            		# Start PWM with 0 duty cycle
            pwm_B.ChangeDutyCycle(speed)  	# Adjust speed with duty cycle

# Function to control the right motor (Motor A)
def motor_right(status, direction, speed):
    if status == 0:  				# Stop the motor
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.LOW)
        GPIO.output(Motor_A_EN, GPIO.LOW)
    else:
        if direction == Dir_forward:  		# Set motor to spin forward
            GPIO.output(Motor_A_Pin1, GPIO.HIGH)
            GPIO.output(Motor_A_Pin2, GPIO.LOW)
            pwm_A.start(100)          		# Start PWM with full duty cycle
            pwm_A.ChangeDutyCycle(speed)  	# Adjust speed with duty cycle
        elif direction == Dir_backward:  	# Set motor to spin backward
            GPIO.output(Motor_A_Pin1, GPIO.LOW)
            GPIO.output(Motor_A_Pin2, GPIO.HIGH)
            pwm_A.start(0)            		# Start PWM with 0 duty cycle
            pwm_A.ChangeDutyCycle(speed)  	# Adjust speed with duty cycle
