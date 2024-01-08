import RPi.GPIO as GPIO
import time
import config
from threading import Thread


LOCK_PIN=config.LOCK_PIN
TRIG=config.TRIG
ECHO=config.ECHO
DOOR_DISTANCE = config.DOOR_DISTANCE

GPIO.setmode(GPIO.BCM)
GPIO.setup(LOCK_PIN, GPIO.OUT)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

class Door:
    def __new__(cls):
        if not hasattr(cls, "instance"):
            cls.instance = super().__new__(cls)
            cls.instance.__init()
        return cls.instance

    @staticmethod
    def get_instance():
        return Door()

    def __init(self):
        GPIO.output(LOCK_PIN, GPIO.LOW)
        self.curr_state = 'CLOSE'
        
    def start_door_listner(self,on_door_close=lambda x: None):
        self.door_handler_thread = Thread(target=self.__check_close, args=(on_door_close,))
        self.door_handler_thread.daemon = True
        self.door_handler_thread.start()  
        
        
        
    def open(self):
        if self.curr_state == 'OPEN':
            return
        self.curr_state = 'OPEN'        
        GPIO.output(LOCK_PIN, GPIO.HIGH)
        time.sleep(5)
        self.close()


    def close(self):
        if self.curr_state == 'CLOSE':
            return
        self.curr_state = 'CLOSE'
        GPIO.output(LOCK_PIN, GPIO.LOW)
        
    def __check_close(self,on_door_close=lambda x: None):
        while True:
            if self.curr_state == 'OPEN':
                time.sleep(5)
            dist = Door.get_door_distance()
            if dist < DOOR_DISTANCE :
                on_door_close()
                self.close()
            time.sleep(1)  # Delay between readings
            
    
    @staticmethod
    def get_door_distance():
        # Set trigger to LOW for a short time to ensure a clean signal
        GPIO.output(TRIG, False)
        time.sleep(0.02)  # 20 microseconds for initialization

        # Send a 10us pulse to trigger
        GPIO.output(TRIG, True)
        time.sleep(0.00001)  # 10 microseconds
        GPIO.output(TRIG, False)

        # Wait for the echo pin to go high
        start_time = time.time()
        while GPIO.input(ECHO) == 0:
            start_time = time.time()

        # Wait for the echo pin to go low again
        stop_time = time.time()
        while GPIO.input(ECHO) == 1:
            stop_time = time.time()

        # Calculate the distance based on the time difference
        elapsed_time = stop_time - start_time
        distance = (elapsed_time * 34300) / 2  # Speed of sound = 343 meters per second (in air)

        return distance

   