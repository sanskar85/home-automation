import os
UPLOAD_FOLDER = "persons"



def list_uploaded_persons():
    person_images_list = []
    person_names = list_dirs()
    for person_name in person_names:
        for image in person_images(person_name):
            person_images_list.append((person_name, image))

    return person_images_list

def get_person_name(index:int):
    person_names = list_dirs()
    return -1 if len(person_names) >= index else person_images[index]


def person_images(person_name):
    folder_path = generate_upload_folder(person_name)
    files_in_folder = [
        f
        for f in os.listdir(folder_path)
        if os.path.isfile(os.path.join(folder_path, f))
    ]
    return files_in_folder


def generate_upload_folder(name):
    upload_folder = os.path.join(UPLOAD_FOLDER, name)
    os.makedirs(upload_folder, exist_ok=True)
    return upload_folder


def list_dirs():
    subfolders = [f.name for f in os.scandir(UPLOAD_FOLDER) if f.is_dir()]
    return subfolders
