
import sys
import config

sys.path.append(config.BASE_FOLDER)
from modules import  Door


def open():
    Door.get_instance().open()

def close():
   Door.get_instance().close()
