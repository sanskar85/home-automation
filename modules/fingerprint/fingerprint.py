import time
import adafruit_fingerprint
import serial
from threading import Thread, Event
import sys
import config

sys.path.append(config.BASE_FOLDER)
from modules import Display,Message
from  utils.images import get_person_name




class Fingerprint:
    def __new__(cls):
        if not hasattr(cls, "instance"):
            cls.instance = super().__new__(cls)
            cls.instance.__init()
        return cls.instance

    @staticmethod
    def get_instance(on_detected=lambda x: None):
        return Fingerprint(on_detected)

    def __init(self,on_detected=lambda x: None):
        self.uart = serial.Serial("/dev/ttyS0", baudrate=57600, timeout=1)
        self.finger = adafruit_fingerprint.Adafruit_Fingerprint(self.uart)
        if self.finger.read_templates() != adafruit_fingerprint.OK:
            raise RuntimeError("Failed to read templates")
        self.scanning = Event()
        self.on_detected = on_detected
        
    
    def __scan_fingerprint(self)->int:
        while self.scanning.is_set():
            if self.finger.get_image() != adafruit_fingerprint.OK:
                continue
            if self.finger.image_2_tz(1) != adafruit_fingerprint.OK:
                continue
            if self.finger.finger_search() == adafruit_fingerprint.OK:
                name = get_person_name(self.finger.finger_id)
                if name != "Unknown":
                    self.on_detected(name)
                    time.sleep(3)
        

    def start_scanning(self):
            self.scanning.set()  # Set the scanning event
            self.scanning_thread = Thread(target=self.__scan_fingerprint)
            self.scanning_thread.start()

    def stop_scanning(self):
        self.scanning.clear()  # Clear the scanning event
        self.scanning_thread.join()  # Wait for the thread to finish

    def start_enroll(self, location):
        self.stop_scanning()  # Stop scanning before enrollment
        self.enroll_finger(location)
        self.start_scanning()  # Resume scanning after enrollment

    def enroll_finger(self, location):
        for fingerimg in range(1, 3):
            if fingerimg == 1:
                Display.get_instance().add_message(Message("Place","finger","to","enroll",delay=0.5,persist=True))
            else:
                Display.get_instance().add_message(Message("Place","same","finger to","enroll",delay=0.5,persist=True))

            while True:
                i = self.finger.get_image()
                if i == adafruit_fingerprint.OK:
                    Display.get_instance().add_message(Message("Biometrics","Recorded",delay=0.5,persist=True))
                    break
                if i == adafruit_fingerprint.NOFINGER:
                    continue
                Display.get_instance().add_message(Message("Biometrics","enrollment","failed",delay=3))

            Display.get_instance().add_message(Message("Templating","biometrics",delay=0.5,persist=True))
            i = self.finger.image_2_tz(fingerimg)
            if i != adafruit_fingerprint.OK:
                if i == adafruit_fingerprint.IMAGEMESS:
                    Display.get_instance().add_message(Message("Messy","images",delay=3))
                elif i == adafruit_fingerprint.FEATUREFAIL:
                    Display.get_instance().add_message(Message("Biometric"," not","clear",delay=3))
                else:
                    Display.get_instance().add_message(Message("Failed","to","create","template",delay=3))
                    
                return
            if fingerimg == 1:
                Display.get_instance().add_message(Message("Remove","finger",delay=0.5,persist=True))
                time.sleep(1)
                while i != adafruit_fingerprint.NOFINGER:
                    i = self.finger.get_image()

        Display.get_instance().add_message(Message("Creating","model...",delay=0.5,persist=True))
        i = self.finger.create_model()
        if i != adafruit_fingerprint.OK:
            if i == adafruit_fingerprint.ENROLLMISMATCH:
                Display.get_instance().add_message(Message("Prints","did","not","match",delay=3))
            else:
                Display.get_instance().add_message(Message("Error","creating","model",delay=3))
            return

        i = self.finger.store_model(location)
        if i != adafruit_fingerprint.OK:
            if i == adafruit_fingerprint.BADLOCATION:
                print("Bad storage location")
            elif i == adafruit_fingerprint.FLASHERR:
                print("Flash storage error")
            else:
                print("Error storing model")
                Display.get_instance().add_message(Message("Error","creating","model",delay=3))
            return

        Display.get_instance().add_message(Message("Enrolled",delay=4))
        return

    def delete_fingerprint(self, location):
        if self.finger.delete_model(location) == adafruit_fingerprint.OK:
            Display.get_instance().add_message(Message("Deleted!!!",delay=2.5))
        else:
            Display.get_instance().add_message(Message("Failed","to", "delete", "biometrics",delay=3))
