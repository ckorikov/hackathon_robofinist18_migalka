#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
import math
import itertools as it

import telepot
from telepot.loop import MessageLoop

from numpy import dot,array,empty_like
from matplotlib.path import Path

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from enum import Enum


def make_path(x1, y1, x2, y2):
    return Path([[x1, y1], [x1, y2], [x2, y2], [x2, y1]])


def perp(a):
    b = empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b


def seg_intersect(a1, a2, b1, b2):
    da = a2 - a1
    db = b2 - b1
    dp = a1 - b1
    dap = perp(da)
    denom = dot(dap, db)
    num = dot(dap, dp)

    x3 = ((num / denom.astype(float)) * db + b1)[0]
    y3 = ((num / denom.astype(float)) * db + b1)[1]
    p1 = make_path(a1[0], a1[1], a2[0], a2[1])
    p2 = make_path(b1[0], b1[1], b2[0], b2[1])
    if p1.contains_point([x3, y3]) and p2.contains_point([x3, y3]):
        return x3, y3
    else:
        return False


def getcoord(rho, theta):
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho
    x1 = int(x0 + 1000 * (-b))
    y1 = int(y0 + 1000 * (a))
    x2 = int(x0 - 1000 * (-b))
    y2 = int(y0 - 1000 * (a))
    return x1, y1, x2, y2


class State(Enum):
    STOP = 0
    DRIVE = 1
    RIGHT_LANE = 2
    LEFT_LANE = 3
    SEARCH = 4


class MigalkaBot:
    def __init__(self):
        self.state = State.STOP

        # ROS init
        rospy.init_node('migalka_core')
        self.controleler = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        rospy.Subscriber("/camera/rgb/image_raw", Image, self._image_callback)
        self.rate = rospy.Rate(2)

        # Movement controller
        self.mov = Twist()
        self._set_v(0.0)
        self._set_a(0.0)
        self.cnt = 0

        # Image controller
        self.bridge = CvBridge()

        # Telegram init
        self.telegram = telepot.Bot("698222967:AAEOczhELvvlMjdzWfJBoGfGiyfVmkD-PI8")
        MessageLoop(self.telegram, self._tgm_handler).run_as_thread()

    def to(self, state):
        print "From %s to state %s" % (self.state, state)
        if self.state == State.STOP:
            if state == State.DRIVE:
                self._set_v(0.1)
                self.state = state
            else:
                print "Unknown state transition"
        elif self.state == state.DRIVE:
            if state == State.STOP:
                self._set_v(0.0)
                self._set_a(0.0)
                self.state = state
            elif state == State.SEARCH:
                self._set_v(0.0)
                val = 0.2
                if abs(self.mov.angular.z) > 0.0010:
                    val = self.mov.angular.z/abs(self.mov.angular.z)*0.2
                self._set_a(val)
                self.state = state
            else:
                print "Unknown state transition"
        elif self.state == state.SEARCH:
            if state == State.DRIVE:
                self._set_v(0.1)
                self._set_a(0.0)
                self.state = state
            else:
                print "Unknown state transition"
        else:
            print "Unknown state"

    def _image_callback(self, msg):
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width, channels = cv2_img.shape
        except CvBridgeError, e:
            print(e)
        else:
            gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
            imgb = cv2.GaussianBlur(gray, (3, 3), 0)
            imgb[0:int(height*0.6),0:width] = 0
            x,th = cv2.threshold(imgb,200,255, cv2.THRESH_BINARY)
            edges = cv2.Canny(th, 50, 400, apertureSize=3)
            lines = cv2.HoughLines(edges, 1, np.pi / 180, 100)
            if lines is not None and len(lines) > 0:
                linesCorr = lines
            #     linesCorr = []
            #     for line in lines:
            #         for rho, theta in line:
            #             # if abs(theta - math.pi / 2) < math.pi / 24: continue
            #             linesCorr.append((rho, theta))
            #         print len(linesCorr)
            #         for t in linesCorr:
            #             comb = list(it.combinations(range(0, len(linesCorr)), 2))
            #             # print comb
            #             for c in comb:
            #                 d1 = linesCorr[c[0]]
            #                 d2 = linesCorr[c[1]]
            #                 if abs(d1[1] - d2[1]) < math.pi / 8:
            #                     del linesCorr[c[0]]
            #                     break

                for l in linesCorr:
                    x1, y1, x2, y2 = getcoord(l[0], l[1])
                    cv2.line(edges, (x1, y1), (x2, y2), (255, 0, 0), 5)
                cv2.imshow("frame", edges)
                cv2.waitKey()

                if self.state == State.DRIVE:
                    if len(linesCorr) < 2:
                        self.cnt = self.cnt + 1
                    elif self.cnt > 10:
                        self.cnt = 0
                        self.to(State.SEARCH)
                    elif len(linesCorr) >= 2:
                        d1 = linesCorr[0]
                        x1, y1, x2, y2 = getcoord(d1[0], d1[1])
                        d2 = linesCorr[1]
                        x3, y3, x4, y4 = getcoord(d2[0], d2[1])
                        p1 = array([x1, y1])
                        p2 = array([x2, y2])
                        p3 = array([x3, y3])
                        p4 = array([x4, y4])
                        x, y = seg_intersect(p1, p2, p3, p4)
                        # print x, width / 2
                        if x > width / 2:
                            self._set_a(-0.01)
                        elif x < width / 2:
                            self._set_a(0.01)
                        else:
                            self._set_a(0.0)
                elif self.state == State.SEARCH:
                    if len(linesCorr) >= 2:
                        self.to(State.DRIVE)
                elif self.state == State.RIGHT_LANE:
                    if len(linesCorr) >= 2:
                        d1 = linesCorr[0]
                        x1, y1, x2, y2 = getcoord(d1[0], d1[1])
                        d2 = linesCorr[1]
                        x3, y3, x4, y4 = getcoord(d2[0], d2[1])
                        p1 = array([x1, y1])
                        p2 = array([x2, y2])
                        p3 = array([x3, y3])
                        p4 = array([x4, y4])
                        x, y = seg_intersect(p1, p2, p3, p4)

    def _tgm_handler(self, msg):
        chat_id = msg['chat']['id']
        command = msg['text']

        if command == '/start':
            self.to(State.DRIVE)
            self.telegram.sendMessage(chat_id, "ok")
        elif command == '/stop':
            self.to(State.STOP)
            self.telegram.sendMessage(chat_id, "ok")

    def _set_v(self, v):
        # print "Set speed: %f" % (v)
        self.mov.linear.x = v

    def _set_a(self, a):
        # print "Set angle: %f" % (a)
        self.mov.angular.z = a


bot = MigalkaBot()

while not rospy.is_shutdown():
    bot.controleler.publish(bot.mov)
    bot.rate.sleep()