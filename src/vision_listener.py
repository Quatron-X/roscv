"""
Listener node for computer vision, GCS
"""

#!/usr/bin/env python

import rospy, cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

bridge = CvBridge()

def callback(data):
    # log to screen, node log, and /rosout
    if data.data:
        rospy.loginfo("frame received")
    
        try:
            # frame = bridge.imgmsg_to_cv2(data, "bgr8")
            frame = bridge.imgmsg_to_cv2(data, "passthrough") # debug
            cv2.imshow("Received Frame", frame)
            cv2.waitKey(1)
        except CvBridgeError as err:
            rospy.loginfo(f"CV_BRIDGE ERROR: {err}")
    else:
        rospy.loginfo("frame not received")

def main():
    rospy.loginfo("started...")

    # anonymous=True to append unique numbers to the end of node's name so multiple listeners can be run
    rospy.init_node('listener', anonymous=True)

    rospy.loginfo("subscribing...")

    # rospy subscriber
    rospy.Subscriber("vision", Image, callback) # calls callback() passing Image msg from message vision

    # spin() keeps this python running until the node is stopped
    rospy.spin()

if __name__ == "__main__":
    main()
