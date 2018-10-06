#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import LaserScan

STOP = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))

def process_scan(msg):
    print(msg)

print("Start!")
rospy.init_node('migalka_core') # Initialise a ROS node
publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
subscriber = rospy.Subscriber('/scan', LaserScan, process_scan)

rate = rospy.Rate(2)

to_publish = Twist()
to_publish.linear.x = -0.3
to_publish.angular.z = 0.2
while not rospy.is_shutdown():
    publisher.publish(to_publish)
    rate.sleep()