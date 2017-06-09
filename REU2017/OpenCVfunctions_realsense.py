#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Using the edge detection demo edge.py from OpenCV combined with image capture
from PyRealSense demo show_cv2.py to do canny edge detection on the RealSense RGB image
'''

# Python 2/3 compatibility
from __future__ import print_function

import logging
logging.basicConfig(level=logging.INFO)

import time
import numpy as np
import cv2
import pyrealsense as pyrs

# turn on the RealSense R200
pyrs.start()
dev = pyrs.Device()

# this is carry over from example code. def is a function definition
# pass moves the code along without actually executing anything
# this statement doesn't really do anything as currently written, but the nothing
# function is call in "createTrackbar" below
def nothing(*arg):
    pass

# create an Opencv window where the image with canny edge detection will eventually show up
cv2.namedWindow('edge')
cv2.namedWindow('edge2')

#create a trackbar named "thrs1" in window "edge". Default position is 2000, range is [0 5000]
cv2.createTrackbar('thrs1', 'edge', 2000, 5000, nothing)
cv2.createTrackbar('thrs2', 'edge', 4000, 5000, nothing)

#cap = video.create_capture(fn)

while True:
    #acquire a frame from the RealSense camera
    dev.wait_for_frame()
    #flag,img = dev.colour
    #we want to use the RGB stream
    img = dev.colour
    #convert image a grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #get threshold values from trackbar position
    thrs1 = cv2.getTrackbarPos('thrs1', 'edge')
    thrs2 = cv2.getTrackbarPos('thrs2', 'edge')
    #identify edges using the Canny edge detection in OpenCV
    blurred= cv2.blur(gray,(5,5))
    edge = cv2.Canny(blurred, thrs1, thrs2, apertureSize=5)
    vis = img.copy()
    vis = np.uint8(vis/2.)
    vis[edge != 0] = (0, 255, 0)
    gray2=gray.copy()
    gray2[edge != 0] = (0)
    print(edge.shape)
    cd = np.concatenate((gray,blurred), axis=1)
    cd2 = np.concatenate((edge,gray2), axis=1)
    cv2.imshow('edge', cd)
    cv2.imshow('edge2',cd2)
    cv2.imshow('',vis)



    ## CvVideoWriter * outStream = cvCreateVideoWriter(outFile, CV_FOURCC('M', 'J', 'P', 'G'), 2,
    ##                                                   cvSize((int)(srcImg->width * percent / 100), (int)(
    ##    srcImg->height * percent / 100)), true );
    ## cvWriteFrame(outStream, srcImgRes)		//CvVideoWriter* outStream = cvCreateVideoWriter(outFile, CV_FOURCC('M','J','P','G'), 2, cvSize((int)(srcImg->width*percent/100), (int)(srcImg->height*percent/100)), true );
    ##cvWriteFrame(outStream, srcImgRes)

    ch = cv2.waitKey(5)
    if ch == 27:
        break


cv2.destroyAllWindows()

