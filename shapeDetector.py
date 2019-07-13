#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
import numpy as np
import math

pub = rospy.Publisher('robotmaker/shape_arm', String, queue_size=10)
rospy.init_node('detector', anonymous=True)
rate = rospy.Rate(100) # 10hz

def setLabel(image, str, contour):
        (text_width, text_height), baseline = cv.getTextSize(str, cv.FONT_HERSHEY_SIMPLEX, 0.7, 1)
        x,y,width,height = cv.boundingRect(contour)
        pt_x = x+int((width-text_width)/2)
        pt_y = y+int((height + text_height)/2)
        cv.rectangle(image, (pt_x, pt_y+baseline), (pt_x+text_width, pt_y-text_height), (200,200,200), cv.FILLED)
        cv.putText(image, str, (pt_x, pt_y), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 1, 8)

def getContours():
        camera = PiCamera()
        camera.resolution = (640, 480)
        camera.framerate = 30
        rawCapture = PiRGBArray(camera, size=(640, 480))
        time.sleep(0.1)
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                img_color = frame.array
                img_gray = cv.cvtColor(img_color, cv.COLOR_BGR2GRAY)
                cv.imshow("Frame", img_gray)
                key = cv.waitKey(1) & 0xFF
                rawCapture.truncate(0)
                if key == ord("q"):
                        break

                ret, img_binary = cv.threshold(img_gray, 127, 255, cv.THRESH_BINARY_INV|cv.THRESH_OTSU)
                _, contours, hierarchy = cv.findContours(img_binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

                for cnt in contours:
                        size = len(cnt)
#                       print(size)
                        epsilon = 0.005 * cv.arcLength(cnt, True)
                        approx = cv.approxPolyDP(cnt, epsilon, True)

                        size = len(approx)
#                       print(size)

                        cv.line(img_color, tuple(approx[0][0]), tuple(approx[size-1][0]), (0, 255, 0), 3)
                        for k in range(size-1):
                                cv.line(img_color, tuple(approx[k][0]), tuple(approx[k+1][0]), (0, 255, 0), 3)

                        if cv.isContourConvex(approx):
                                print(size)
                                shapeDetector(size)
                        else:
                                setLabel(img_color, str(size), cnt)


def shapeDetector(size):
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
