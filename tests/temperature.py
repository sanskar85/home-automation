
import adafruit_dht


dht11 = adafruit_dht.DHT11(23)


def getTemp():
        try:
            temp = dht11.temperature
            humidity = dht11.humidity
            return (temp,humidity)
        except RuntimeError as error:
            return None
        except Exception as error:
            return None
        
print(getTemp())