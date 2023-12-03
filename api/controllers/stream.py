import cv2
from fastapi.responses import StreamingResponse
import sys

import config

sys.path.append(config.BASE_FOLDER)
from modules import  Recognizer


def generate_frames():
    cap = Recognizer.get_instance().get_camera()

    while True:
        success, frame = cap.read()
        if not success:
            break
        _, buffer = cv2.imencode(".jpg", frame)  # Encode frame as JPEG
        yield (
            b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n"
        )


def video_feed():
    return StreamingResponse(
        generate_frames(), media_type="multipart/x-mixed-replace;boundary=frame"
    )
