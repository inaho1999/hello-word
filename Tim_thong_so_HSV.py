# coding=utf-8
import cv2
import qi
import numpy as np
import almath
import math
import motion
import sys
import vision_definitions
import argparse
from pynput import keyboard


def none(x):
    pass


def getImange():
    source = cv2.VideoCapture("video-1644677362.mp4")
    frame = source.get(cv2.CAP_PROP_FRAME_COUNT)
    winnamed = "redball"
    # cv2.namedWindow(winnamed)
    # cv2.createTrackbar('hmin', winnamed, 154, 179, none)
    # cv2.createTrackbar('hmax', winnamed, 179, 179, none)
    # cv2.createTrackbar('smin', winnamed, 88, 255, none)
    # cv2.createTrackbar('smax', winnamed, 255, 255, none)
    # cv2.createTrackbar('vmin', winnamed, 94, 255, none)
    # cv2.createTrackbar('vmax', winnamed, 255, 255, none)
    while True:
        # get image
        hasframe, image_top = source.read()
        if hasframe == False:
            print 'cannot capture.'
            break

        hsv = cv2.cvtColor(image_top, cv2.COLOR_BGR2HSV)
        # hmin = cv2.getTrackbarPos('hmin', winnamed)
        # hmax = cv2.getTrackbarPos('hmax', winnamed)
        # smin = cv2.getTrackbarPos('smin', winnamed)
        # smax = cv2.getTrackbarPos('smax', winnamed)
        # vmin = cv2.getTrackbarPos('vmin', winnamed)
        # vmax = cv2.getTrackbarPos('vmax', winnamed)
        # # camera bottom (154, 88, 94), (179, 255, 255)
        # mask = cv2.inRange(hsv, (hmin, smin, vmin), (hmax, smax, vmax))
        mask = cv2.inRange(hsv, (154, 88, 94), (179, 255, 255))
        contours, hier = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            cv2.circle(image_top, (int(x),int(y)), int(radius), (220, 0, 255), 1)
        cv2.imshow(winnamed, mask)
        if cv2.waitKey(1) == 27:
            break
        if source.get(cv2.CAP_PROP_POS_FRAMES) == frame:
            source.set(cv2.CAP_PROP_POS_FRAMES, 0)


def turnaround():
    pass


if __name__ == '__main__':

    data = []
    getImange()
