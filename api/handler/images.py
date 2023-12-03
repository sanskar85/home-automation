import os
import uuid

from fastapi import UploadFile

UPLOAD_FOLDER = "persons"


def upload_person_image(file: UploadFile, person_name):
    try:
        random_id = f"{str(uuid.uuid4())}.{file.filename.split('.')[-1]}"
        upload_folder = generate_upload_folder(person_name)
        file_path = os.path.join(upload_folder, random_id)

        with open(file_path, "wb") as f:
            f.write(file.file.read())
            
        if os.path.exists("data.pkl"):
            os.remove("data.pkl")

        return (True, None)
    except Exception as e:
        return (False, e)


def generate_upload_folder(name):
    upload_folder = os.path.join(UPLOAD_FOLDER, name)
    os.makedirs(upload_folder, exist_ok=True)
    return upload_folder


def list_dirs():
    subfolders = [f.name for f in os.scandir(UPLOAD_FOLDER) if f.is_dir()]
    return subfolders
