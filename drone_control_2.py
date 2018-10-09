#!/usr/bin/env python
import rospy
# ros image control robot
import sys
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
# import the necessary packages
from imutils import paths
import numpy as np
import imutils
import cv2


def find_marker(image):
	# convert the image to grayscale, blur it, and detect edges
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray, (5, 5), 0)
	edged = cv2.Canny(gray, 35, 125)

	# find the contours in the edged image and keep the largest one;
	# we'll assume that this is our piece of paper in the image
	cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	cnts = cnts[0] if imutils.is_cv2() else cnts[1]
	c = max(cnts, key = cv2.contourArea)
	# compute the bounding box of the of the paper region and return it
	return cv2.minAreaRect(c)

def callback(ros_data):
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    marker = find_marker(image_np)
    # draw a bounding box around the image and display it
    box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
    box = np.int0(box)
    cv2.drawContours(image_np, [box], -1, (0, 255, 0), 2)
    cv2.imshow("image", image_np)
    cv2.waitKey(1)


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('drone_camera_control', anonymous=True)
    subscriber = rospy.Subscriber('/ardrone/front/image_raw/compressed', CompressedImage, callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down Drone Control module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
