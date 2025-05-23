# Import libraries
import RPi.GPIO as GPIO
import time
from time import sleep
from Bluetin_Echo import Echo
from rpi_ws281x import * 
import Adafruit_PCA9685 
from robotLight import RobotLight
import argparse

# GPIO Mode (BOARD / BCM) > from the Adeept Robot Manual
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Define the pins (Found from the original code of Servo by the robot)
    # SERVO
SERVO = 0 
    # Ultrasonic
TRIG = 11
ECHO = 8
    # Motor
Motor_A_EN    = 4
Motor_B_EN    = 17
Motor_A_Pin1  = 26
Motor_A_Pin2  = 21
Motor_B_Pin1  = 27
Motor_B_Pin2  = 18
    # LED
LED_COUNT = 3 # Number of LED pixels.
LED_PIN = 12 # GPIO pin connected to the pixels (18 uses PWM!).
#LED_PIN = 10 # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
LED_CHANNEL = 0 # set to '1' for GPIOs 13, 19, 41, 45 or 53
LED_DMA = 10 # DMA channel to use for generating signal (try 10)
    # Line tracking 
line_pin_right = 19
line_pin_middle = 16
line_pin_left = 20


# GPIO Pins Setup
    # Ultrasonic
GPIO.setup(TRIG,GPIO.OUT) 
GPIO.setup(ECHO,GPIO.IN)  
    # Motor
GPIO.setup(Motor_A_EN, GPIO.OUT)
GPIO.setup(Motor_B_EN, GPIO.OUT)
GPIO.setup(Motor_A_Pin1, GPIO.OUT)
GPIO.setup(Motor_A_Pin2, GPIO.OUT)
GPIO.setup(Motor_B_Pin1, GPIO.OUT)
GPIO.setup(Motor_B_Pin2, GPIO.OUT)
    # Line tracking
GPIO.setup(line_pin_right,GPIO.IN)
GPIO.setup(line_pin_middle,GPIO.IN)
GPIO.setup(line_pin_left,GPIO.IN)

# Define Constants and Objects
    # Ultrasonic 
speed_of_sound = 315

    # Motor
Dir_forward   = 1
Dir_backward  = 0
left_forward  = 1
left_backward = 0
right_forward = 0
right_backward= 1
pwm_A = GPIO.PWM(Motor_A_EN, 1000)
pwm_B = GPIO.PWM(Motor_B_EN, 1000)

    # Lights 
RL = RobotLight()
RL.start()


    # LED strip configuration:
LED_FREQ_HZ = 800000 # LED signal frequency in hertz (usually 800khz)
LED_BRIGHTNESS = 255 # Set to 0 for darkest and 255 for brightest
LED_INVERT = False # True to invert the signal (when using NPN transistor level shift)

    # Servo
look_direction = 1 # change this form 1 to 0 to reverse servos
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50) #  This servo is controlled by a 50Hz PWM signal
servo_max = 500
servo_min = 100
servo_org = 300

    # Maze Solver
WALL_DISTANCE = 15    # Desired distance from the wall (in cm)
TURN_TIME = 0.7         # Time to turn 90 degrees (tune this)
FORWARD_SPEED = 100   # Speed for forward movement
REACHED_END = False   # Determine whther endpoing is achieved or not


# Define the required functions 
    # Ultrasonic - Object detection 
def measureDistanceLib():
    echo = Echo(TRIG, ECHO, speed_of_sound)
    samples = 5
    distance = echo.read('cm', samples)
    print(distance, 'cm')
    return distance

def measureDistance():
    # Send a pulse to the sensor
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    # Measure the time taken for the echo to return
    while GPIO.input(ECHO) == 0:
       #print("echo")
        pulse_start = time.time()
    while GPIO.input(ECHO) == 1:
       #print("echo11")
        pulse_end = time.time()
    #Time elapsed
    pulse_duration = pulse_end - pulse_start
    # Calculate the distance
    distance = (pulse_duration * 34300)/2
    distance = round(distance, 2)
    return distance

    # Servo: pwm.set_pwm(port_num, deviation_value, PWM_duty_cycle)
def servo_to_max():
    pwm.set_pwm(SERVO, 0, servo_min)
    time.sleep(1)
    pwm.set_pwm(SERVO, 0, servo_max)
    time.sleep(1)

def servo_to_min():
    pwm.set_pwm(SERVO, 0, servo_max)
    time.sleep(1)
    pwm.set_pwm(SERVO, 0, servo_min)
    time.sleep(1)

def look(angle):
    pwm.set_pwm(SERVO, 0, angle)
    time.sleep(0.4)
    return measureDistance()

    # Motor
def motorStop():
	GPIO.output(Motor_A_Pin1, GPIO.LOW)
	GPIO.output(Motor_A_Pin2, GPIO.LOW)
	GPIO.output(Motor_B_Pin1, GPIO.LOW)
	GPIO.output(Motor_B_Pin2, GPIO.LOW)
	GPIO.output(Motor_A_EN, GPIO.LOW)
	GPIO.output(Motor_B_EN, GPIO.LOW)

def motor_left(status, direction, speed):# Motor 2 positive and negative rotation
	if status == 0: # stop
		GPIO.output(Motor_B_Pin1, GPIO.LOW)
		GPIO.output(Motor_B_Pin2, GPIO.LOW)
		GPIO.output(Motor_B_EN, GPIO.LOW)
	else:
		if direction == Dir_backward:
			GPIO.output(Motor_B_Pin1, GPIO.HIGH)
			GPIO.output(Motor_B_Pin2, GPIO.LOW)
			pwm_B.start(100)
			pwm_B.ChangeDutyCycle(speed)
		elif direction == Dir_forward:
			GPIO.output(Motor_B_Pin1, GPIO.LOW)
			GPIO.output(Motor_B_Pin2, GPIO.HIGH)
			pwm_B.start(0)
			pwm_B.ChangeDutyCycle(speed)


def motor_right(status, direction, speed): # Motor 1 positive and negative rotation
	if status == 0: # stop
		GPIO.output(Motor_A_Pin1, GPIO.LOW)
		GPIO.output(Motor_A_Pin2, GPIO.LOW)
		GPIO.output(Motor_A_EN, GPIO.LOW)
	else:
		if direction == Dir_forward:#
			GPIO.output(Motor_A_Pin1, GPIO.HIGH)
			GPIO.output(Motor_A_Pin2, GPIO.LOW)
			pwm_A.start(100)
			pwm_A.ChangeDutyCycle(speed)
		elif direction == Dir_backward:
			GPIO.output(Motor_A_Pin1, GPIO.LOW)
			GPIO.output(Motor_A_Pin2, GPIO.HIGH)
			pwm_A.start(0)
			pwm_A.ChangeDutyCycle(speed)
	return direction


def move(speed, direction, turn, radius=0.6):   # 0 < radius <= 1  
	#speed = 100
	if direction == 'forward':
		if turn == 'right':
			motor_left(0, left_backward, int(speed*radius))
			motor_right(1, right_forward, speed)
		elif turn == 'left':
			motor_left(1, left_forward, speed)
			motor_right(0, right_backward, int(speed*radius))
		else:
			motor_left(1, left_forward, speed)
			motor_right(1, right_forward, speed)
	elif direction == 'backward':
		if turn == 'right':
			motor_left(0, left_forward, int(speed*radius))
			motor_right(1, right_backward, speed)
		elif turn == 'left':
			motor_left(1, left_backward, speed)
			motor_right(0, right_forward, int(speed*radius))
		else:
			motor_left(1, left_backward, speed)
			motor_right(1, right_backward, speed)
	elif direction == 'no':
		if turn == 'right':
			motor_left(1, left_backward, speed)
			motor_right(1, right_forward, speed)
		elif turn == 'left':
			motor_left(1, left_forward, speed)
			motor_right(1, right_backward, speed)
		else:
			motorStop()
	else:
		pass

def destroy():
	look(servo_org)
	time.sleep(0.2)
	motorStop()
	GPIO.cleanup()             # Release resource
	
    # Line tracking
def reachedEnd():
    status_right = GPIO.input(line_pin_right)  
    status_middle = GPIO.input(line_pin_middle)  
    status_left = GPIO.input(line_pin_left)  
    if (status_right == 1 or status_left == 1 or status_middle == 1 ):
        REACHED_END = True
    return REACHED_END
	
    # LED 
class LED:
    def __init__(self):
        self.LED_COUNT      = 16      # Number of LED pixels.
        self.LED_PIN        = 12      # GPIO pin connected to the pixels (18 uses PWM!).
        self.LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
        self.LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
        self.LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
        self.LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
        self.LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53
        parser = argparse.ArgumentParser()
        parser.add_argument('-c', '--clear', action='store_true', help='clear the display on exit')
        args = parser.parse_args()

        # Create NeoPixel object with appropriate configuration.
        self.strip = Adafruit_NeoPixel(self.LED_COUNT, self.LED_PIN, self.LED_FREQ_HZ, self.LED_DMA, self.LED_INVERT, self.LED_BRIGHTNESS, self.LED_CHANNEL)
        # Intialize the library (must be called once before other functions).
        self.strip.begin()

    # Define functions which animate LEDs in various ways.
    def colorWipe(self, color, wait_ms=0):
        """Wipe color across display a pixel at a time."""
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
            self.strip.show()
            time.sleep(wait_ms/1000.0)


# Maze solver using right-hand following algorithm
while True:
    try:
		
        front = look(servo_org)
        print("front")
        print(front)
        print("more than 2, forward")
        move(50, 'forward', '')
        time.sleep(0.2)
        if front < 12:
            motorStop()
            print("less than 2, stop")
            right = look(servo_max)
            time.sleep(0.1)
            print("right")
            print(right)
            if right > 12:
                print("right more than 2, turn")
                look(servo_org)
                move(100, 'no', 'right')
                time.sleep(0.4)
                motorStop()
            else:
                move(100, 'no', 'left')
                time.sleep(0.4)
                motorStop()
				
        time.sleep(0.2)
    except KeyboardInterrupt:
        destroy()
        break










