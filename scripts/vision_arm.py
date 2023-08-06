"""
Simple script that disarms the drone upon detecting an object
"""

#!/usr/bin/env python

import rospy
from roscv_modules import vision_module
from roscv.msg import VisionResult

# Import the API.
from ..src.local_iq_gnc.py_gnc_functions import *
# To print colours (optional).
from ..src.local_iq_gnc.PrintColours import *

global drone

def main():
    # Initialize ROS node
    rospy.init_node("vision_arm")

    # Create an object for the navigation API
    drone = gnc_api()
    
    # Wait for FCU connection
    drone.wait4connect()
    # Wait for mode switch (to guided)
    drone.wait4start()

    # Local reference frame
    drone.initialize_local_frame()

    # Arm command
    drone.arm()

    # Subscribe to vision_result topic
    rospy.Subscriber("vision_result", VisionResult, disarming_function)

def disarming_function(data):
    if data.data:
        rospy.loginfo("OBJECCT DETECTED")

        drone.disarm()