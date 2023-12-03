# import requi9red module

import cv2
import face_recognition
from .library import getKnownEncodings

known_names, known_encodings = None, None
video_capture = None


# Raspberry Pi pin configuration: GPIO 2 (SDA) and GPIO 3 (SCL)
class Recognizer:
    def __new__(cls):
        if not hasattr(cls, "instance"):
            cls.instance = super().__new__(cls)
            cls.instance.__init()
        return cls.instance

    @staticmethod
    def get_instance():
        return Recognizer()

    def __init(self):
        self.video_capture = cv2.VideoCapture(0)
        self.known_names = None
        self.known_encodings = None

    def read_encodings(self, cb=lambda x,y: None):
        if self.known_names != None:
            return cb("ENCODINGS", True)
        cb("ENCODINGS", False)
        known_names, known_encodings = getKnownEncodings()
        cb("ENCODINGS", True)
        self.known_names = known_names
        self.known_encodings = known_encodings

    def get_camera(self):
        return self.video_capture

    def startCamera(self, on_face_detected=lambda x: None):
        self.read_encodings()

        while True:
            success, frame = self.video_capture.read()
            if not success:
                break

            is_recognized, name = self.recognize(frame)
            if is_recognized:
                on_face_detected(name)

    def recognize(self, frame):
        face_locations = face_recognition.face_locations(frame)
        face_encodings = face_recognition.face_encodings(frame, face_locations)

        for face_encoding in face_encodings:
            matches = face_recognition.compare_faces(
                self.known_encodings, face_encoding, tolerance=0.5
            )
            if True in matches:
                matched_index = matches.index(True)
                return (True, self.known_names[matched_index])

        return (False, "Unknown")

    def stopCamera(self):
        self.video_capture.release()
        cv2.destroyAllWindows()


def startRecognizer(encodings_cb=lambda x,y: None,on_face_detected=lambda x,y: None):
    rec = Recognizer.get_instance()
    rec.read_encodings(encodings_cb)
    rec.startCamera(on_face_detected)

