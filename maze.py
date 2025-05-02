# Import libraries
import RPi.GPIO as GPIO
import time
from time import sleep
from Bluetin_Echo import Echo
from rpi_ws281x import * 
import Adafruit_PCA9685 

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
ENA = 4
ENB = 17
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


# GPIO Pins Setup
    # Ultrasonic
GPIO.setup(TRIG,GPIO.OUT) 
GPIO.setup(ECHO,GPIO.IN)  
    # Motor
GPIO.setup(ENA,GPIO.OUT)
GPIO.setup(ENB,GPIO.OUT)
GPIO.setup(IN1,GPIO.OUT)
GPIO.setup(IN2,GPIO.OUT)
GPIO.setup(IN3,GPIO.OUT)
GPIO.setup(IN4,GPIO.OUT)
    # 

# Define Constants and Objects
    # Ultrasonic 
speed_of_sound = 315

    # Motor
pwm_A=GPIO.PWM(ENA,1000) # p is an object that gives a neat way to control the motor, 1000 Hz Freq.
pwm_B=GPIO.PWM(ENB,1000) # p is an object that gives a neat way to control the motor, 1000 Hz Freq.

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

# Define the required functions 
    # Ultrasonic - Object detection 
def measureDistance():
    echo = Echo(TRIG, ECHO, speed_of_sound)
    samples = 5
    distance = echo.read('cm', samples)
    print(distance, 'cm')
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

    # Motor
def motorStop(): 
	GPIO.output(Motor_A_Pin1, GPIO.LOW)
	GPIO.output(Motor_A_Pin2, GPIO.LOW)
	GPIO.output(Motor_B_Pin1, GPIO.LOW)
	GPIO.output(Motor_B_Pin2, GPIO.LOW)
	GPIO.output(ENA, GPIO.LOW)
	GPIO.output(ENB, GPIO.LOW)



     


# Define the LED Class
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










