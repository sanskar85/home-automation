import os
import pickle

import face_recognition

UPLOADS_FOLDER = "persons"


def getSavedImages():
    file_list = []
    for root, directories, files in os.walk(UPLOADS_FOLDER):
        for filename in files:
            file_path = os.path.join(root, filename)
            person_name = os.path.basename(root)
            file_list.append((person_name, file_path))
    return file_list


def getKnownEncodings():
    known_encodings = None
    known_names = None
    saved_persons = getSavedImages()
    try:
        with open("data.pkl", "rb") as file:
            known_encodings = pickle.load(file)
            known_names = pickle.load(file)
    except FileNotFoundError:
        pass
    except:
        pass

    if known_encodings is None:
        known_encodings = []
        known_names = []
        for person_name, image_path in saved_persons:
            known_image = face_recognition.load_image_file(image_path)
            encodings = face_recognition.face_encodings(known_image)
            if len(encodings) == 0:
                os.remove(image_path)
                continue
            encoding = encodings[0]
            known_encodings.append(encoding)
            known_names.append(person_name)


    with open("data.pkl", "wb") as file:
        pickle.dump(known_encodings, file)
        pickle.dump(known_names, file)

    return known_names, known_encodings
