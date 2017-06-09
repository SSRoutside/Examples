#!/usr/bin/env python
# -*- coding: utf-8 -*-
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
from flask import Flask, render_template, Response
from pythonds.basic.stack import Stack
from scipy import ndimage
from scipy.ndimage import morphology as scmorph
from collections import deque

import logging
import time
import numpy as np
import cv2
import pyrealsense as pyrs
import atexit
import thread


# set up logging level
logging.basicConfig(level=logging.INFO)

# create a default motor object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x60)

# auto disable motors on shutdown
def turnOffMotors():
        mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

atexit.register(turnOffMotors)

# For proper motor control, should use a diff-drive model to convert
# velocity commands into wheel control
# should set up a dictionary or new class to handle motor attributes, like speed


# -- INITIALISING

# global variables or anything that needs to be initialised outside a function goes here

# image display stack
imQueue = deque()

# set filter coefficients (currently first order filter)
c0 = 1
c1 = 0.5
thresh = 20

# initialise constants used in navigation loop
yaw_error = 0
est_dist = 100

# motor state variables set to zero
cs_m1 = 0
cs_m2 = 0
cs_m3 = 0
cs_m4 = 0

# other variables to pass between threads:
cX, cY = [0,0]
mean_disp = 0


# -- MOTOR CONTROL FUNCTIONS

def simple_lowpass(v_current, v_des):
    # low pass filter to mitigate discontinuities in motor commands
    v_new = int(round(c0*v_current + c1*(v_des - v_current)))
    # watch out for negative numbers...
    return v_new


def motorControl(vdLeft, vdRight):
    # basic differential control function for the four wheel motors
    global cs_m1, cs_m2, cs_m3, cs_m4

    vd_vec = [vdRight, vdRight, vdLeft, vdLeft]
    motorspeeds = [cs_m1, cs_m2, cs_m3, cs_m4]

    # lowpass filtering may be overkill, these motors seem pretty robust
    for wi in range(1, 5):
        # check speed difference 
        if abs(vd_vec[wi-1]-motorspeeds[wi-1]) < thresh:
            v_new = vd_vec[wi-1]

        else:
            v_new = simple_lowpass(motorspeeds[wi - 1], vd_vec[wi-1])

        mot = mh.getMotor(wi)

        if v_new < 0:
            mot.setSpeed(-1 * v_new)
            mot.run(Adafruit_MotorHAT.BACKWARD)
        else:
            mot.setSpeed(v_new)
            mot.run(Adafruit_MotorHAT.FORWARD)

        # update current motor speed
        motorspeeds[wi - 1] = v_new


# fancier manoeuvres can be developed by decoupling front and rear motors

# -- VISUAL NAVIGATION FUNCTIONS

def depthmap_seg_nav(d_im_col):
    # Segments the depth map and identifies objects based on contour information
    # Could be used for (eg) wall following, object detection, goal detection
    
    # declare shared variables
    global cX, cY
    global mean_disp
    global yaw_error

    # take colorised depth image and return a heading direction
    # filter out 'background' (aka depth out of range).
    d_im_gray = cv2.cvtColor(d_im_col, cv2.COLOR_BGR2GRAY)
    d_thresh = cv2.adaptiveThreshold(d_im_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 3, 5)
    # set background to zero
    d_thresh[d_im_gray < 5] = 0

    # erode to remove white noise
    kernel = np.ones((2, 2), np.uint8)
    opening = cv2.morphologyEx(d_thresh, cv2.MORPH_OPEN, kernel, iterations=2)

    # need to isolate regions, so distance transform to ensure they're separated
    dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)
    # threshold back to binary
    _, sure_fg = cv2.threshold(dist_transform, 0.12 * dist_transform.max(), 255, 0)
    sure_fg = np.uint8(sure_fg)

    _, contours, hierarchy = cv2.findContours(sure_fg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # if intensity too subtle, could use hsv as base image, compare hue?

    mean = 0
    found_direction = 0
    for cntr in contours:

        if 1500 < cv2.contourArea(cntr):
            cv2.drawContours(d_im_col, [cntr], 0, (0, 255, 0), 2)
            mask = np.zeros(d_im_gray.shape, np.uint8)
            cv2.drawContours(mask, [cntr], 0, (255, 255, 255), -1)
            mean_new = cv2.mean(d_im_gray, mask)

            if mean_new > mean:
                mean = mean_new
                # this is rather bodgy! should assign before loop instead of using flag
                cntr_direction = cntr
                found_direction = 1

    # we can attempt to use segmentation for navigation, for example
    if found_direction:        
        # if we have an updated direction - should usually get SOMETHING
        # get centroid
        M = cv2.moments(cntr_direction)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        mean_disp = mean[0]

        # calculate vector direction, assuming centre of x axis is 0 heading
        # call depthmap_seg_nav in outer control loop to enable navigation via image segmentation
        
        yaw_error = cX - (640/2) # change this if using smaller depth frame

    return True

def depthmap_flow_nav(d_im):
    # calculates a crude depth flow field and identifies a likely clear path 
    # by template matching to a gaussian distance function
    global cX, cY
    global yaw_error
    global est_dist

    # create nxn zeros and appropriate kernel
    kernlen = 321
    dirac_im = np.zeros((kernlen, kernlen))
    # set element at the middle to one, a dirac delta
    dirac_im[kernlen//2, kernlen//2] = 1
    
    # gaussian-smooth the dirac, resulting in a gaussian filter:
    gauss_template = cv2.GaussianBlur(dirac_im, (kernlen, kernlen), 0)
    # normalise
    max_g = max(gauss_template.max(axis=1))
    gauss_display = np.array(255*gauss_template/max_g, dtype=np.uint8)

    # filter the distance output to remove discontinuities and approximate a flow field
    d_im_filt = scmorph.grey_closing(d_im, size=(7, 7))
    blur = cv2.GaussianBlur(d_im_filt, (71, 71), 0)

    # we may want to restrict our analysis to a central 'band' of the image
    # can use a mask in the template match for this
    blur = np.array(blur, dtype=np.uint8)

    # Cross correlate a gaussian peaked function of size < image:
    template_match = cv2.matchTemplate(blur, gauss_display, cv2.TM_CCORR_NORMED)

    template_match = cv2.normalize(template_match, 0, 1, cv2.NORM_MINMAX)

    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(template_match)
    # max_loc gives top left of template match
    cX = max_loc[0] + kernlen // 2
    cY = max_loc[1] + kernlen // 2
    
    # distance: examine distance at max_loc to estimate whether we have hit a wall? This doesn't work very well!
    vis_cent = blur[(cX-8):(cX+8), (cY-8):(cY+8)]
    vis_cent = vis_cent.astype('float64')
    vis_cent[vis_cent < 5 ] = np.nan
    est_dist = np.nanmean(vis_cent) 

    yaw_error = cX - 640/2 # change this for different depth image size


# -- WEB STREAMING FUNCTIONS

# Option: try using Server-Sent Events to publish to a web port - the browser should keep the stream 'open' and listen for updates
# or use gen functions and a push/pop queue

# set up flask app for streaming to browser
app = Flask(__name__)

@app.route('/')
def index():
    return render_template('./index.html')

# try queuing
def gen():
    # streaming over wifi slows things down massively. There may be a more efficient encoding/sending mechanism
    # in the meantime, outputting only once per second to reduce lag
    global imQueue
    imcount = 0
    imint = 30

    while True:
        try:
            imcount += 1
            imshow = imQueue.popleft()
            # only show every fifth image (trying to reduce lag)
            if imcount> imint: 
                ret, frame = cv2.imencode('.jpg', imshow)
                # this is pretty slow over wifi
                jpeg_encode = frame.tobytes()
                yield(b'--frame\r\n'
                      b'Content-Type: image/jpeg\r\n\r\n' + jpeg_encode + b'\r\n\r\n')
                imcount = 1

        except:
            pass



@app.route('/video_feed')
def video_feed():
    return Response(gen(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def threaded_stream():
    app.run(host='0.0.0.0', port=5000,  debug=False, threaded=True)

def shutdown_server(environ):
    # graceful shutdown for external streaming *currently not used*
    if not 'werkzeug.server.shutdown' in environ:
        raise RuntimeError('Not running the development server')
    
    environ['werkzeug.server.shutdown']()


# -- MAIN CONTROL LOOP

if __name__ == '__main__':
    # wee bit of overkill in our main definition here

    # Enabling remote view will stream the sensor output to a web server
    # (address is ip of robot, check before enabling, usually 10.251.209.161)

    # Pros: can view in browser at ip.address.here:5000
    # Cons: MASSIVELY slows down performance, for some reason. Need to fix this

    global imQueue

    REMOTE_VIEW = True

    vt = 300  # desired forward speed - nice and steady
    v0 = 0  # starting speed, use if we put in a derivative function
    kp = 1
    kd = 0.3 # totally random derivative gain
    wallcount = 0
    timeout = 0
    yaw_e_prev = 0
    dt = 0.03*5

    motorrunning = True

    frameint = 5
    framecount = 0

    if REMOTE_VIEW:
        thread.start_new_thread(threaded_stream, ())


    pyrs.start()
    py_dev = pyrs.Device(device_id = 0, streams = [pyrs.ColourStream(fps = 30), pyrs.DepthStream(fps=30)])

    while motorrunning:
        timethen = time.time()
        framecount += 1

        ## IMAGE PROCESSING
        py_dev.wait_for_frame()
        c_im = py_dev.colour
        rgb_im = c_im[..., ::-1]

        # scaling to map better to color/grayscale
        d_im = py_dev.depth * 0.05
        
        # close holes without removing segmentation by doing it before converting to image
        d_im_col = cv2.applyColorMap(d_im.astype(np.uint8), cv2.COLORMAP_HOT)
        # every nth frame, update direction by analysing depth map
        # error sampling rate << framerate, to try and reduce jitter. Could also sample at a higher rate and filter.
        
        # two nav options: segmentation or gradient. Segmentation is a problem with very noisy images
        if framecount > frameint:
            yaw_e_prev = yaw_error
            # thread.start_new_thread(depthmap_seg_nav, (d_im_col, ))
            thread.start_new_thread(depthmap_flow_nav, (d_im,))
            framecount = 1

        cv2.circle(d_im_col, (cX, cY), 7, (255, 255, 255), -1)
        cv2.putText(d_im_col, str(yaw_error), (cX - 20, cY - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
        #cd = np.concatenate((blur, gauss_template), axis=1)            
        cd = np.concatenate((rgb_im, d_im_col), axis=1)
            
        if REMOTE_VIEW:
            # eventually replace dots with an arrow indicating desired direction?
            # pushing every fifth frame to try and keep things moderately realtime
            # running ok but there is some lag. Deal with on queue end?
            # push to stack:
            imQueue.append(cd)

        else:
            # I do not recommend using local display over wifi unless you don't care if the robot runs in circles
            # but if connected to ethernet, you can change REMOTE_VIEW flag and uncomment the below
            cv2.imshow('', cd)
        

        # MOTOR CONTROL
        if abs(yaw_error) > 0:
            # basic PD motor control, kp and kd not tuned
            yaw_dev = (yaw_error - yaw_e_prev)*dt
            vd_left = vt + kp * yaw_error + kd*yaw_dev   # desired speed from left motors
            vd_right = vt - kp * yaw_error - kd*yaw_dev  # desired speed from right motor
            motorControl(vd_left, vd_right)
 

        if est_dist < 10:
            print("Too close?")
            # wallcount times out
            wallcount += 1
            timeout = 0
        else:
            timeout += 1

        if timeout > 2:
            wallcount = 0

        #if wallcount > 8:
        #    motorrunning = False
        #    break

        if cv2.waitKey(1) & 0xFF == ord('q'):
            motorrunning=False
            break
        timenow = time.time()
        #print(timenow - timethen)



    # time.sleep(1)
    print ("ended, stopping all motors")

    for i in range(1,5):
        imot = mh.getMotor(i)
        imot.run(Adafruit_MotorHAT.RELEASE)


    #shutdown_server(self)
