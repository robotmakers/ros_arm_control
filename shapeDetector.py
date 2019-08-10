#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
import numpy as np
import math
import collections

MAX_SHAPES_DEQUEUE_SIZE = 20

pub = rospy.Publisher('robotmaker/shape_arm', String, queue_size=10)
rospy.init_node('detector', anonymous=True)
rate = rospy.Rate(1000) # 10hz

def setLabel(image, str, contour):
        (text_width, text_height), baseline = cv.getTextSize(str, cv.FONT_HERSHEY_SIMPLEX, 2, 3)
        x, y, width, height = cv.boundingRect(contour)
        pt_x = x + int((width - text_width) / 2)
        pt_y = y + int((height + text_height) / 2)
#       cv.rectangle(image, (pt_x, pt_y+baseline), (pt_x+text_width, pt_y-text_height), (200,200,200), cv.FILLED)
        cv.putText(image, "press 'q' to exit on this frame", (20, 20), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1, 8)
        cv.putText(image, str, (pt_x, pt_y), cv.FONT_HERSHEY_SIMPLEX, 5, (0, 0, 255), 6, 8)

def getContours():
        camera = PiCamera()
        camera.resolution = (640, 480)
        camera.framerate = 30
        rawCapture = PiRGBArray(camera, size=(640, 480))
        time.sleep(0.1)
        detectedShapes = collections.deque(MAX_SHAPES_DEQUEUE_SIZE * [0], MAX_SHAPES_DEQUEUE_SIZE)
        previousSize = 0
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                img_color = frame.array
                img_gray = cv.cvtColor(img_color, cv.COLOR_BGR2GRAY)

                ret, img_binary = cv.threshold(img_gray, 120, 255, cv.THRESH_BINARY_INV|cv.THRESH_OTSU)
                cv.imshow("Frame2", img_binary)
                key1 = cv.waitKey(1) & 0xFF
                rawCapture.truncate(0)
                if key1 == ord("q"):
                        break

                img_contours, contours, hierarchy = cv.findContours(img_binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
                img_contours = cv.drawContours(img_color, contours, -1, (0, 255, 0), 3);

                for cnt in contours:
                        size = len(cnt)
                        epsilon = 0.005 * cv.arcLength(cnt, True)
                        approx = cv.approxPolyDP(cnt, epsilon, True)
                        size = len(approx)

                        cv.line(img_color, tuple(approx[0][0]), tuple(approx[size-1][0]), (0, 255, 0), 3)
                        for k in range(size-1):
                                cv.line(img_color, tuple(approx[k][0]), tuple(approx[k+1][0]), (0, 255, 0), 3)

                        if cv.isContourConvex(approx):
                                detectedShapes.append(size)
                                samples = [ item for item, count in collections.Counter(detectedShapes).items() if count > 1]
                                if len(samples) == 1 and previousSize == size:
                                        sendRosMsg(size)

                                previousSize = size
                                setLabel(img_contours, str(size), cnt)
                cv.imshow("Frame4", img_contours)
                key3 = cv.waitKey(1) & 0xFF
                rawCapture.truncate(0)
                if key3 == ord("q"):
                        break

                time.sleep(0.3)


def sendRosMsg(size):
        if not rospy.is_shutdown():
                send_str = "s:%s" % str(size)
                rospy.loginfo(send_str)
                pub.publish(send_str)
                rate.sleep()

if __name__ == '__main__':
        try:
                getContours()
        except rospy.ROSInterruptException:
                pass
