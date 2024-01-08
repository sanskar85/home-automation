import RPi.GPIO as GPIO
import time
import config
from threading import Thread


LED_1=config.LED_1
LED_2=config.LED_2
LED_3=config.LED_3

GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_1, GPIO.OUT)
GPIO.setup(LED_2, GPIO.OUT)
GPIO.setup(LED_3, GPIO.OUT)

class Switch:
    def __new__(cls):
        if not hasattr(cls, "instance"):
            cls.instance = super().__new__(cls)
            cls.instance.__init()
        return cls.instance

    @staticmethod
    def get_instance():
        return Switch()

    def __init(self):
        GPIO.output(LED_1, GPIO.LOW)
        GPIO.output(LED_2, GPIO.LOW)
        GPIO.output(LED_3, GPIO.LOW)
        
    def switch_1(self,option=False):      
        print(option, GPIO.HIGH if option else GPIO.LOW)
        GPIO.output(LED_1, GPIO.HIGH if option else GPIO.LOW)
        
    def switch_2(self,option=False):      
        GPIO.output(LED_2, GPIO.HIGH if option else GPIO.LOW)
        
    def switch_3(self,option=False):      
        GPIO.output(LED_3, GPIO.HIGH if option else GPIO.LOW)
        
        