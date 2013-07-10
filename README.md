# orbit_camera_actuation

Actuates the Logitech Orbit QuickCam via ROS and libwebcam library.

## Components and Dependencies
The following components are involved in controlling the camera:
* [ROS](http://www.ros.org) - Robot Operation System
* CreeperCam.cc - Based on stephanie’s code for actuating the camera using libwebcam library.
* controls.cc - Class for camera actuating controllers.
* cam_actuator_node.py - ROS node for actuating the camera. Listens to IntArray messages ([Pan,Tilt,Zoom]) on /logitech_cam/camera_instr. Zoom value is ignored, and has to be handled by preprocessor.
* libwebcam - Library for actuating and controlling usb cameras. libwebcam0 in Ubuntu synaptic package manager.
* [usb_cam](http://www.ros.org/wiki/usb_cam) - ROS library for capturing images from camera  (optional for actuating the camera).

## Start the camera node. Note, roscore has to be started already. In a separate terminal type `roscore` to start

    $ rosrun camera_actuator camera_actuator_node

#### Send random commands
Randomly selected commands from a list of commands can be sent to the camera by:

    $ rosrun camera_actuator command_generator -c <command list> -t <time duration> -f <frequence>

E.g.

    $ rosrun camera_actuator command_generator -c [ [150,0,0],[-150,0,0],[0,100,0],[0,-100,0],[0,0,10],[0,0,-10] ] -t 10 -f 1

TODO: 
* Console prompt to generate commands

## Input commands
TODO: It would be useful to have all available controls supported for the ros package

### Current functionality
* array of relative displacements, [pan, tilt, zoom]. Subscribes to the topic `/logitech_cam/camera_instr`

#### TODO: Functionality to implement
* array of absolute displacements

## Workspace limits
Workspace limits are hard coded in c++. 
TODO: create rosparams for the limits

## Outputs
* The package does not give any image outputs at this time. For reading images, the package usb_cam may be used.
* executed commands - A valid command with respect to the workspace limits is executed and published on the topic `/logitech_cam/camera_executed`
