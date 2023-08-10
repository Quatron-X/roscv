"""
Pure camera streaming node through ROS.
Will try to use threading and lossy image compression with cv.imencode
Ended up converting to gray and encoding mono8
"""

#!/usr/bin/env python

import rospy, cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

# Instantiate CvBridge for converting ROS Image messages to OpenCV2
bridge = CvBridge()

def main():
    vid = cv2.VideoCapture(0)
    vid.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    # encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]

    # initialize node
    rospy.init_node("vision")
    # ros publisher
    # frame_pub = rospy.Publisher("vision", Image, queue_size=10)
    frame_pub = rospy.Publisher("vision", CompressedImage, queue_size=10)
    # publish frequency in Hz
    rate = rospy.Rate(30) # 10 Hz
    
    while not rospy.is_shutdown() and vid.isOpened():
        ret, frame = vid.read()
        # _, frame = cv2.imencode('.jpg', frame, encode_param) # compressing frame
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        try:
            # frame_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            # frame_msg = bridge.cv2_to_imgmsg(frame, "mono8") # debug
            frame_msg = bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg') # debug
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