import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

# init list with pin numbers

pinList = [20, 21]

# loop through pins and set mode and state to 'low'

for i in pinList: 
  GPIO.setup(i, GPIO.OUT) 
  GPIO.output(i, GPIO.LOW)

# time to sleep between operations in the main loop

SleepTimeL = 2

# main loop
for i in range(100):
    try:
        print("ONE",i%2)
        GPIO.output(20,   i%2 )
        GPIO.output(21, i%2)
        time.sleep(SleepTimeL);  

        # End program cleanly with keyboard
    except KeyboardInterrupt:
        print(" Quit")

        # Reset GPIO settings
        GPIO.cleanup()