#!/usr/bin/env python
# -*- coding: utf-8 -*-
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

import logging
logging.basicConfig(level=logging.INFO)

import time
import numpy as np
import cv2
import pyrealsense as pyrs
import atexit

# From Adafruit MotorHat example code
# create a default object, no changes to I2C address or frequency
mh= Adafruit_MotorHAT(addr=0x60)

# auto disable motors on shutdown
def turnOffMotors():
        mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

atexit.register(turnOffMotors)

# set which motorblock is being used
myMotor = mh.getMotor(1)

# start the pyrealsense library for communicating with the Intel RealSense R200
pyrs.start()
dev = pyrs.Device()

# initialize a counter
cnt = 0
a = 0
# Get current time
last = time.time()

# image acquisition parameters
smoothing = 0.9;
fps_smooth = 30

# set the speed to start, from 0 (off) to 255 (max speed)
myMotor.setSpeed(150)
myMotor.run(Adafruit_MotorHAT.FORWARD)
# turn off motor
myMotor.run(Adafruit_MotorHAT.RELEASE)

while (cnt<500):
    print cnt

    cnt += 1

    if cnt < 250:
        print "Forward"
        myMotor.setSpeed(200)
        myMotor.run(Adafruit_MotorHAT.FORWARD)


    if cnt >= 250 :
        a += 1
        if a <= 200:
            i = 200 - a
            print i
            if i != 0:
                print "slowing"
                myMotor.setSpeed(i)
            if i == 0:
                print "stopped"
                myMotor.run(Adafruit_MotorHAT.RELEASE)


    # The % is the modulo operator in python- it returns the remainder of cnt/10
    if (cnt % 10) == 0:
        now = time.time()
        dt = now - last
        fps = 10/dt
        fps_smooth = (fps_smooth * smoothing) + (fps * (1.0-smoothing))
        last = now

    dev.wait_for_frame()
    c = dev.colour
    d = dev.depth * dev.depth_scale * 1000
    d = cv2.applyColorMap(d.astype(np.uint8), cv2.COLORMAP_RAINBOW)

    cd = np.concatenate((c,d), axis=1)

    cv2.putText(cd, str(fps_smooth)[:4], (0,50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,0))

    cv2.imshow('', cd)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
