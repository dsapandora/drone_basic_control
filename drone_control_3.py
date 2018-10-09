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

def distance_to_camera(knownWidth, focalLength, perWidth):
	# compute and return the distance from the maker to the camera
	return (knownWidth * focalLength) / perWidth

# initialize the known distance from the camera to the object, which
# in this case is 24 inches
KNOWN_DISTANCE = 24.0

# initialize the known object width, which in this case, the piece of
# paper is 12 inches wide
KNOWN_WIDTH = 11.0
focalLength = 0.0
first_round = True
drone_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


def callback(ros_data):
    global first_round, focalLength
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    marker = find_marker(image_np)
    if first_round:
        focalLength = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH
        first_round = False
    inches = distance_to_camera(KNOWN_WIDTH, focalLength, marker[1][0])
    # draw a bounding box around the image and display it
    box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
    box = np.int0(box)
    cv2.drawContours(image_np, [box], -1, (0, 255, 0), 2)
    cv2.putText(image_np, "%.2fft" % (inches / 12), (image_np.shape[1] - 200, image_np.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 3)
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
