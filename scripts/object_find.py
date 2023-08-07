"""
Script to estimate horizontal distance from the magnet to the object
"""

#!/usr/bin/env python

import rospy
from roscv_modules import vision_module
from roscv.msg import VisionResult
from std_msgs import Float64

# Import the API.
from local_iq_gnc.py_gnc_functions import *
# To print colours (optional).
from local_iq_gnc.PrintColours import *

drone = gnc_api()

def main():
    pass