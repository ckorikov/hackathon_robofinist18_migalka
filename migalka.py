#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# from sensor_msgs.msg import LaserScan

STOP = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
bridge = CvBridge()

# def process_scan(msg):
#     print(msg)

def image_callback(msg):
    print("Received an image!")
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
# publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
# subscriber = rospy.Subscriber('/scan', LaserScan, process_scan)

rate = rospy.Rate(2)

to_publish = Twist()
to_publish.linear.x = -0.3
to_publish.angular.z = 0.2
while not rospy.is_shutdown():
    publisher.publish(to_publish)
    rate.sleep()