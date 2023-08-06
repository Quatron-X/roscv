"""
Computer vision node, raspikrti
"""

#!/usr/bin/env python

import rospy, cv2
from roscv_modules import vision_module
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from roscv.msg import VisionResult

# vision module
vm = vision_module.VisionModule()

# Instantiate CvBridge for converting ROS Image messages to OpenCV2
bridge = CvBridge()

def main():
    vid = cv2.VideoCapture(0)
    vid.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    # initialize node
    rospy.init_node("vision")
    # ros publisher
    frame_pub = rospy.Publisher("vision", Image, queue_size=10)
    result_pub = rospy.Publisher("vision_result", VisionResult, queue_size=10)
    # publish frequency in Hz
    rate = rospy.Rate(10) # 10 Hz
    
    while not rospy.is_shutdown() and vid.isOpened():
        ret, frame = vid.read()
        # if ret: # debug
        #     cv2.imshow("frame test", frame) # debug
        #     cv2.waitKey(1) # debug
        try:
            result, mask_result, detected = vm.detect_color(frame) # sends detected bool through msg

            frame_msg = bridge.cv2_to_imgmsg(result, "bgr8")
            # frame_msg = bridge.cv2_to_imgmsg(frame, "passthrough") # debug
            frame_msg.header.stamp = rospy.Time().now()

            # pass the bool value detected into the VisionResult message
            result_msg = VisionResult(detected)

            # log into screen, node log, and /rosout
            rospy.loginfo("frame processed")

            # publish message to topic vision
            frame_pub.publish(frame_msg)
            result_pub.publish(result_msg)
            rospy.loginfo("frame sent")

        except CvBridgeError as err:
            rospy.loginfo(f"CV_BRIDGE ERROR: {err}")

        rate.sleep()

    return

if __name__ == "__main__":
    main()