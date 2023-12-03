
import sys

sys.path.append('scripts')
from modules import Fingerprint

def enroll(index:int):
    Fingerprint.get_instance().start_enroll(index)
