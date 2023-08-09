"""
Simple script that disarms the drone upon detecting an object
"""

#!/usr/bin/env python

import rospy
from roscv_modules import vision_module
from roscv.msg import VisionResult

# Import the API.
from local_iq_gnc.py_gnc_functions import *
# To print colours (optional).
from local_iq_gnc.PrintColours import *

def setup():
    # Initialize ROS node
    rospy.init_node("vision_arm")

    # Create an object for the navigation API
    global drone
    drone = gnc_api()
    
    # Wait for FCU connection
    drone.wait4connect()
    # Wait for mode switch (to guided)
    drone.wait4start()

    # Local reference frame
    drone.initialize_local_frame()

    # Start the loop
    main()

def main():
    # Subscribe to vision_result topic
    rospy.Subscriber("vision_result", VisionResult, disarming_function)

    rospy.spin()

def disarming_function(data):
    # Verify FCU connection
    connected = drone.wait4connect()
    # Verify mode switch (to guided)
    guided = drone.wait4start()

    if (connected==0) and (guided==0):
        if data.detected and not drone.current_state_g.armed:
            rospy.loginfo("OBJECT DETECTED")
            drone.arm()
        elif not data.detected and drone.current_state_g.armed:
            rospy.loginfo("OBJECT GONE")
            drone.disarm()
    else:
        rospy.loginfo("GUIDED STOPPED")

if __name__ == '__main__':
    setup()