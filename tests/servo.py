import RPi.GPIO as GPIO
import time


SERVO_PIN=21

GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

p = GPIO.PWM(SERVO_PIN, 50) # GPIO 17 for PWM with 50Hz
p.start(2.5) 

def __angle(deg):
    return  deg / 18 + 2

while True:

    p.ChangeDutyCycle(__angle(180))
    time.sleep(1)

    p.ChangeDutyCycle(__angle(120))
    time.sleep(1)

