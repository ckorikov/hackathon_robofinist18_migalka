#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
import cv2

import numpy as np
from numpy import array
import matplotlib.pyplot as plt
import random

STOP = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
bridge = CvBridge()

def process_scan(msg):
    currDist = msg.ranges
    n = 50
    newList = [currDist[i:i + n] for i in range(0, len(currDist), n)]
    nArray = array(newList)

    a11 = nArray.reshape(50, 100)
    plt.imshow(a11, cmap='hot')
    plt.colorbar()
    plt.show()

def image_callback(msg):
    #print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg
        cv2.imwrite('camera_image.jpeg', cv2_img)

print("Start!")
rospy.init_node('migalka_core') # Initialise a ROS node
publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
subscriber = rospy.Subscriber('/scan', LaserScan, process_scan)

rate = rospy.Rate(2)

to_publish = Twist()
to_publish.linear.x = 0.0
to_publish.angular.z = 0.0
while not rospy.is_shutdown():
    publisher.publish(to_publish)
    rate.sleep()