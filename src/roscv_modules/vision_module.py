"""
Computer Vision functions to process videos
Copied from Code/Vision/vision.py to be run inside the RPi
"""

import cv2
import numpy as np
from PIL import Image

# cap = cv2.VideoCapture(0)


#lower_limit = [79, 137, 244]
#upper_limit = [108, 195, 252]
class VisionModule:
    def __init__(self):
        self.color = [94, 166, 248] # piring oren tenant 4
        # color = [102, 158, 139] # piring ijo tenant 4
        # color = [0, 255, 0]

    def get_limits(self, color):
        self.c = np.uint8([[self.color]]) # BGR values into an np object
        self.hsv_c = cv2.cvtColor(self.c, cv2.COLOR_BGR2HSV)

        self.lower_limit = self.hsv_c[0][0][0] - 10, 124, 124 # lower limit in HSV
        self.upper_limit = self.hsv_c[0][0][0] + 10, 255, 255 # upper limit in HSV

        self.lower_limit = np.array(self.lower_limit, dtype=np.uint8)
        self.upper_limit = np.array(self.upper_limit, dtype=np.uint8)

        return self.lower_limit, self.upper_limit

    def detect_color(self, frame): # takes frame, returns boxed result
        self.hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        self.lower_limit, self.upper_limit = self.get_limits(self.color)
        self.mask = cv2.blur(cv2.inRange(self.hsv_frame, self.lower_limit, self.upper_limit), (5,5))

        self.pil_mask = Image.fromarray(self.mask)
        self.bbox = self.pil_mask.getbbox()    

        detected = False

        if self.bbox is not None:
            self.x1, self.y1, self.x2, self.y2 = self.bbox
            if (abs(self.x2-self.x1) >= 5) and (abs(self.y2-self.y1) >= 5):
                frame = cv2.rectangle(frame, (self.x1, self.y1), (self.x2, self.y2), (0, 255, 0), 5)
                detected = True
            else:
                detected = False


        return frame, self.mask, detected
