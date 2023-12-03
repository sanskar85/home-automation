from fastapi import APIRouter, File, Form, HTTPException, UploadFile
from fastapi.responses import JSONResponse

router = APIRouter()
import config

from .controllers import stream,gate,fingerprint
from .handler import images


@router.get("/status")
async def route_video_feed():
    return {"success": True, "recognizor_status": config.RECOGNIZER_STATE}


@router.get("/live-feed")
async def route_video_feed():
    return stream.video_feed()


@router.post("/upload-images")
async def upload_file(file: UploadFile = File(...), name: str = Form(...)):
    if name is None:
        raise HTTPException(
            status_code=400, detail="Name field is missing in form data"
        )

    status, error = images.upload_person_image(file, name)

    if status:
        return {"success": True}
    else:
        return {"success": False, "message": f"File upload failed: {str(error)}"}


@router.post("/enroll-fingerprint/")
async def enroll_fingerprint(name: str):
    print(name)
    known_persons = images.list_dirs()
    try:
        index = known_persons.index(name)
        status = fingerprint.enroll(index)
        return {
            "success": "status",
        }
    except:
        return {
            "success": False,
        }

@router.post("/open-gate")
async def open_gate():
    gate.open()
    return {"success": True}

@router.post("/close-gate")
async def close_gate():
    gate.close()
    return {"success": True}
    
