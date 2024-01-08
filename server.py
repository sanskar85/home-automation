import threading
import uvicorn
from fastapi import FastAPI

import config
from api import routers
from modules.face_recognizer import recognizer
from modules import Display,Message,Fingerprint,Door

app = FastAPI()

app.include_router(routers.router)



def on_face_detected(name:str):
    Display.get_instance().add_message(Message("Recognized", name,"Opening Gate",delay=2))
    Door.get_instance().open()

def on_door_close():
    Display.get_instance().add_message(Message("Door","Closed"))

def encodings_callback(state, value):
    if state == "ENCODINGS":
        if value == True:
            Display.get_instance().add_message(Message("Recognizer", "Started",delay=1.5))
            config.RECOGNIZER_STATE = "RECOGNIZER_STARTED"
        else:
            Display.get_instance().add_message(Message("Loading", "Recognizer",delay=1.5))
            config.RECOGNIZER_STATE = "LOADING_RECOGNIZER"
        return
    
    


def start_server():
    Display.get_instance()
    Fingerprint.get_instance(on_face_detected).start_scanning()
    Door.get_instance().start_door_listner(on_door_close)
    
    threading.Thread(target=recognizer.startRecognizer, args=(encodings_callback,on_face_detected)).start()
    uvicorn.run(app, host="0.0.0.0", port=8282)


if __name__ == "__main__":
    start_server()
