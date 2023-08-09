"""
Pure camera streaming node through ROS.
Will try to use threading and lossy image compression with cv.imencode
"""

#!/usr/bin/env python

import rospy, cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# Instantiate CvBridge for converting ROS Image messages to OpenCV2
bridge = CvBridge()

def main():
    vid = cv2.VideoCapture(0)
    vid.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
    vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]

    # initialize node
    rospy.init_node("vision")
    # ros publisher
    frame_pub = rospy.Publisher("vision", Image, queue_size=10)
    # publish frequency in Hz
    rate = rospy.Rate(10) # 10 Hz
    
    while not rospy.is_shutdown() and vid.isOpened():
        ret, frame = vid.read()
        _, frame = cv2.imencode('.jpg', frame, encode_param) # compressing frame
        try:
            # frame_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            frame_msg = bridge.cv2_to_imgmsg(frame, "passthrough") # debug
            frame_msg.header.stamp = rospy.Time().now()

            # log into screen, node log, and /rosout
            rospy.loginfo("frame processed")

            # publish message to topic vision
            frame_pub.publish(frame_msg)
            rospy.loginfo("frame sent")

        except CvBridgeError as err:
            rospy.loginfo(f"CV_BRIDGE ERROR: {err}")

        rate.sleep()

    return

if __name__ == "__main__":
    main()