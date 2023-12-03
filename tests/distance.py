import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

TRIG = 13  # GPIO pin for the trigger
ECHO = 6  # GPIO pin for the echo

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

GPIO.output(TRIG, False)
print("Waiting for sensor to settle...")
time.sleep(2)

try:
    while True:
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()

        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # Speed of sound = 34300 cm/s

        distance = round(distance, 2)
        print(f"Distance: {distance} cm")
        time.sleep(1)

except KeyboardInterrupt:
    GPIO.cleanup()
