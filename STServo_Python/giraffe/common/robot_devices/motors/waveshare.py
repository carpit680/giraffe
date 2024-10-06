import os
import time
import sys, tty, termios

parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)
from STservo_sdk import *