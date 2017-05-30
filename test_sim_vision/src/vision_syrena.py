#!/usr/bin/env python
import cv2
import numpy as np
import rospkg
import rospy
import math
from sensor_msgs.msg import CompressedImage
from test_sim_srv_msg.msg import vision_data
from test_sim_srv_msg.srv import path_sim

lower_orange = np.array([30,30,152])
upper_orange = np.array([90,100,255])
lower_red = np.array([0,120,100])
upper_red = np.array([40,200,255])
img = None
contours = None
orange = None
closing = None

def Oreintation(moment):
    tmp = pow((moment['mu20'] - moment['mu02']), 2)
    tmp = math.sqrt(tmp + (4 * pow(moment['mu11'], 2)))

    k = moment['mu02'] - moment['mu20']

    l = 2 * moment['mu11']

    rad_maj = math.atan2(k + tmp, l)
    rad_min = math.atan2(k - tmp, l)
    return rad_maj, rad_min

def find_path():
    global contours,img

    for c in contours:
        rect = (x,y),(ww,hh),angle = cv2.minAreaRect(c)
        # print("rect:  ",rect)
        # area = ww*hh
        # print("area: ")
        # print(ww)
        # if area <= 20 or area >= 1000 or ww <= 10:
        #     continue
        real_area = cv2.contourArea(c)

        if real_area <= 2000 or real_area >= 3000:
            continue
        print(real_area)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(img, [box], -1,(0,0,0),1,8)


def findColors():
    global contours,img,orange, closing
    kernel = np.ones((5,5),np.uint8)

    while not rospy.is_shutdown():
        while img is None:
            print("img None")
        im = img.copy()
        cv2.imshow('from_cam',img)
        # blur = cv2.blur(im, (5,5))
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        red = cv2.inRange(hsv, lower_red, upper_red)
        opening = cv2.morphologyEx(red, cv2.MORPH_OPEN, kernel)
        erosion = cv2.erode(red, kernel, iterations = 1)
        dilation = cv2.dilate( red, kernel, iterations = 1)
        closing = cv2.morphologyEx(red, cv2.MORPH_CLOSE, kernel)

        imgray = cv2.cvtColor(hsv,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(red,127,255,0)
        _ , contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,
                                                        cv2.CHAIN_APPROX_NONE)
        result = cv2.drawContours(im, contours, -1, (0,255,255), 3)
        # find_path()
        cv2.imshow('result',result)
        for c in contours:
            M = cv2.moments(c)
            rect = (x,y),(ww,hh),angle = cv2.minAreaRect(c)
            area = ww*hh
            if area <= 10 or ww <= 2 :
                continue
            real_area = cv2.contourArea(c)
            ratio_area = real_area/area
            ratio_scale = hh/ww

            angle = 90-Oreintation(M)[0]*180/math.pi

            print("Angle: ", angle)
            if not M['m00'] == 0.0:
                cx = float(M['m10']/M['m00']) - 400
                cy = 300 - float(M['m01']/M['m00'])
                print('cx: ',cx)
                print('cy: ',cy)
        break

     	cv2.waitKey(30)

def callback(msg):
    global img,wait,hsv,orange

    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.imdecode(arr, 1)


if __name__ == '__main__':
    rospy.init_node('syrena_gazebo')
    topic = '/syrena/bottom_cam/image_raw/compressed'
    rospy.Subscriber(topic, CompressedImage,callback)
    findColors()
