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

def callback(ros_data):
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
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
