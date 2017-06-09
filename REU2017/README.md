# REU2017

This repository will be used for code associated with the SSR lab Summer 2017 REU project.

%%-----------------------------------------------%%
Description of script: Basic Navigation Functions

This is a simple navigation script, using some basic image filtering to feedback a visual loop to the wheel motors.

Motors: Motor speeds are fed through a lowpass filter, to reduce motor loading when a sudden speed or directional change is called for. The low-level motor control function takes inputs vdLeft, vdRight, which are the desired speeds of the left and right hand sides accordingly. This function could easily be expanded to control all four wheels independently. Note that because the Adafruit motor control drivers take unsigned input, we must be careful about which direction we drive the motors in (BACKWARDS or FORWARDS).

RealSense: Visual control is enabled through the RealSense depth camera. There are two depth filters currently implemented, as demonstration functions. 'depthmap_seg_nav' finds image regions, identifies large regions which are further away, calculates the centroid, and returns an error signal proportional to the offset between this centroid and the camera image centre (along the x-axis). 'depth_flow_nav' (which is a little more robust, and probably faster), low-pass filters the depth image to generate a pseudo-depth-flowmap, and cross correlates this with a basic gaussian template to look for a globally maximum flow peak. It returns the heading error between the current heading and the location of this global maxima. Both of these functions are quite crude, and easily confused!

Streaming: The realsense colour and depth frames are streamed to a web socket, which can be accessed in a browser at ip.address.ofthe.robot:5000. Over ethernet, this streaming is close to real time, however when the robot is untethered, the wifi connection throttles the data significantly, so we send only every 30th frame (roughly 1 frame a second). This process operates in parallel with the navigation loop, so slower frame output should not interfere with the robot performance.

Main control loop: High level control is (presently) fairly basic. The robot tries to keep moving forward at a steady speed, while avoiding obstacles. The image processing subfunctions return an error metric (yaw_error) which is proportional to the angular distance between the desired heading and the current heading. To steer, a proportional-derivative function translates this heading error into desired wheel speeds. To reduce processor load and oversteering, the heading error is only sampled every five frames.

Some basic functionality that could be added quickly:
- Stop or turn around when a dead-end is reached
- high speed tight turns (stop or reverse one set of motors while speeding up the other side significantly)
- size analysis of segmented depth field: can we fit through a gap?
