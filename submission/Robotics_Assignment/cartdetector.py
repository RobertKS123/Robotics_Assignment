#!/usr/bin/env python
#James Thackeray-2228361
#Robert Schwarz-2136369
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError #wiki.ros.org gave help with this
import cv2

def cart_callback(img_msg):
    pub = rospy.Publisher('/witsdetector', String, queue_size=10)
    try:
        # read in the image from ros to convert to a opencv image
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # convert image received to hsv
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # get the green of the image
    green = np.uint8([[[5, 5, 255]]])
    hsvGreen = cv2.cvtColor(green, cv2.COLOR_BGR2HSV)
    # create a mask to isolate the green in the cart
    mask = cv2.inRange(hsv, (60, 255, 100), (100, 255,255))

    # if the mask does not contain the pixel range of that colour green then mask will be 0
    if (np.all((mask == 0))):
        # no utility cart in view
        pub.publish("No")
        print("No")
    else:
        pub.publish("Yes")
        print("Yes")


rospy.init_node('cartdetector', anonymous=True)
bridge = CvBridge()
global sub_once
sub_once = rospy.Subscriber("/camera/rgb/image_raw", Image, cart_callback)

while not rospy.is_shutdown():
    rospy.spin()
    break
