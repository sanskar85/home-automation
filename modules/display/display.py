from dataclasses import dataclass
import subprocess
import adafruit_ssd1306
import board
from PIL import Image, ImageDraw, ImageFont
import time
import adafruit_dht
import psutil
import config
import queue
from threading import Thread

WIDTH = 128
HEIGHT = 64
BORDER = 5



@dataclass
class Message():
    line1: str = ""
    line2: str = ""
    line3: str = ""
    line4: str = ""
    delay: int = 1
    persist:bool = False


# Raspberry Pi pin configuration: GPIO 2 (SDA) and GPIO 3 (SCL)
class Display:
    def __new__(cls):
        if not hasattr(cls, "instance"):
            cls.instance = super().__new__(cls)
            cls.instance.__init()
        return cls.instance

    @staticmethod
    def get_instance():
        return Display()

    def __init(self):
        i2c = board.I2C()
        self.disp = adafruit_ssd1306.SSD1306_I2C(
            WIDTH, HEIGHT, i2c, addr=0x3C, reset=None
        )
        self.image = Image.new("1", (self.disp.width, self.disp.height))
        self.draw = ImageDraw.Draw(self.image)
        self.draw.rectangle(
            (0, 0, self.disp.width, self.disp.height), outline=255, fill=255
        )
        self.font = ImageFont.truetype("modules/display/PixelOperator.ttf", config.FONT_SIZE)
        
        
        for proc in psutil.process_iter():
            if proc.name() == 'libgpiod_pulsein' or proc.name() == 'libgpiod_pulsei':
                proc.kill()
        self.dht11 = adafruit_dht.DHT11(config.DHT11)
        
        self.message_queue = queue.Queue() 

        self.message_handler_thread = Thread(target=self.message_handler)
        self.message_handler_thread.daemon = True
        self.message_handler_thread.start()
        
    def add_message(self, message: Message):
        self.message_queue.put(message)
        
        
    
    def __clear(self):
        self.draw.rectangle(
            (0, 0, self.disp.width, self.disp.height), outline=0, fill=0
        )
        
    def __printDefault(self):
        dht = self.__getTemp()
        if dht == None:
            dht = (0,0)
        connected_wifi = self.__get_connected_wifi()
        msg = Message(f"Temperature :- {dht[0]}'C", f"Humidity :- {dht[1]}%","Wifi Status:", connected_wifi,1)
        self.__print(msg)
        
    def message_handler(self):
        prev_persist = False
        while True:
            try:
                message = self.message_queue.get(timeout=1)  # Wait for a message with a timeout
                prev_persist = message.persist
                self.__print(message)
            except queue.Empty:
                if prev_persist == False:
                    self.__printDefault() 
                    prev_persist = True
        
        
    def __print(self, message:Message):
        self.__clear()
        self.draw.text((0, 0), message.line1, font=self.font, fill=255)
        self.draw.text((0, 16),message.line2, font=self.font, fill=255)
        self.draw.text((0, 32),message.line3, font=self.font, fill=255)
        self.draw.text((0, 48),message.line4, font=self.font, fill=255)

        self.disp.image(self.image)
        self.disp.show()
        time.sleep(message.delay)
    
    def __getTemp(self):
            try:
                temp = self.dht11.temperature
                humidity = self.dht11.humidity
                return (temp,humidity)
            except RuntimeError as error:
                return None
            except Exception as error:
                return None

    @staticmethod
    def __get_connected_wifi():
        try:
            result = subprocess.check_output(["iwgetid", "-r"])
            return result.decode("utf-8").strip()
        except subprocess.CalledProcessError:
            return "Not Connected"


