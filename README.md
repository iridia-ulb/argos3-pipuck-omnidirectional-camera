# Summary
This repository is an external plugin for use with the Pi-Puck in ARGoS3. It provides an omnidirectional camera sensor capable of locating Apriltags and detecting nearby LEDs.

# Quick start

## Compilation
If you are working with the hardware, the [meta-pipuck](https://github.com/iridia-ulb/meta-pipuck) layer will automatically install this plugin for you. If you are working in simulation, you will need to follow the steps to make the camera available to ARGoS. These steps assume that the main ARGoS simulator has been installed. If you get an error about FindARGoS.cmake not being in `CMAKE_MODULE_PATH`, it is likely that the main simulator is not properly installed.

```sh
# clone this repository
git clone https://github.com/iridia-ulb/argos3-pipuck-omnidirectional-camera.git
# compile
cd argos3-pipuck-omnidirectional-camera
mkdir build
cd build
cmake ../src
make
# run test
argos3 -c testing/test_omnidirectional_camera.argos
```

# Current state of the plugin
At the time of writing, the sensor is capable of detecting tags directly in front of it but the mirror and its optical properties have not been implemented. Incorporating the mirror into this plugin requires work on both the hardware and software implementations.

## Hardware
Assuming the Apriltags library is capable of detecting tags in the distorted image from the reflection of the mirror, the first step will be to update the corner and center pixel locations of each detected tag such that they reflect how the tag would have been detected through a normal camera. This should be done after [Line 266 of hardware / pipuck_omnidirectional_camera_default_sensor.cpp](https://github.com/iridia-ulb/argos3-pipuck-omnidirectional-camera/blob/4f72915eb284e3d92e693d7c0e10c80e78f2c6d8/src/plugins/robots/pi-puck/hardware/pipuck_omnidirectional_camera_default_sensor.cpp#L266). Specifically, the coordinates in `ptDetection->p` and `ptDetection->c` need to be updated.

If detecting the color of the LEDs nearby a tag is of interest, the `DetectLed` member function should be updated. This method (i) maps a 3D location (relative to the robot) to pixel coordinates in the captured image and (ii) samples those pixels to determine the color of an LED in that location. There is likely some overlap between the mapping component of this operation and the operation that updates the pixel coordinates of the detected tags.

## Simulation
To keep the simulation fast, the tags nearby the robots are simply converted into readings in the simulated sensor. The sensor does not simulate an image from the camera. The conversion of tags into readings is a two step process. The first step is a broad query against the simulator's indices to see if there are any nearby tags. This broad query occurs using the index method 
[`ForEntitiesInBoxRange`](https://github.com/iridia-ulb/argos3-pipuck-omnidirectional-camera/blob/4f72915eb284e3d92e693d7c0e10c80e78f2c6d8/src/plugins/robots/pi-puck/simulator/pipuck_omnidirectional_camera_default_sensor.cpp#L192). This function takes an axis-aligned bounding box that represents a rough upper approximation of what the camera/mirror can see. It is currently calculated using a frustum, however, it is unlikely that this will apply once the mirror is taking into consideration.

The second step is a refining query that iterates over the results of the first query to determine whether or not the tags can really be seen. This refining query is implemented in an [`operator()`](https://github.com/iridia-ulb/argos3-pipuck-omnidirectional-camera/blob/4f72915eb284e3d92e693d7c0e10c80e78f2c6d8/src/plugins/robots/pi-puck/simulator/pipuck_omnidirectional_camera_default_sensor.cpp#L222) member function that takes a `CTagEntity&` as an argument. This member function is called by `ForEntitiesInBoxRange` for each tag detected in the broad query and is responsible for writing the readings into the control interface. The function applies several checks to determine whether or not the tag really can be seen by the camera. For example, it currently checks, among other things, whether there is anything blocking the rays between the camera and the corners of the tags. This check is used since if more than one of these rays are blocked, it is unlikely that the tag would have been detected in the real world. Such checks will need to be updated to compensate for the optics of the mirror, i.e., to check the path between the corner of a tag and the mirror instead of checking the path between the corner of the tag and the camera.

For the detecting of nearby LEDs, it is possible that no changes will need to be made to the `DetectLed` method. It should only be necessary to pass the same bounding box to the LED index as was passed to the tag index on [Line 198 of simulator / pipuck_omnidirectional_camera_default_sensor.cpp](https://github.com/iridia-ulb/argos3-pipuck-omnidirectional-camera/blob/master/src/plugins/robots/pi-puck/simulator/pipuck_omnidirectional_camera_default_sensor.cpp#L198).

# Current limitations
## Focus control is not implemented
The ability to adjust camera focus requires adding an auxiliary or V4L2 I2C subdevice to the OV5647 driver. These changes will mostly need to be made in the [meta-pipuck](https://github.com/iridia-ulb/meta-pipuck) layer and include:
1. Enabling the video core I2C bus in config.txt
2. Creating an appropriate device tree overlay
3. Patching the OV5647 driver to exposure a control to focus motor

Once these steps are complete, the plugin in this repository can be updated to include the following code to control the camera focus:
```c++
struct v4l2_control sFocusControl;
memset(&sFocusControl, 0, sizeof (sFocusControl));
sFocusControl.id = V4L2_CID_FOCUS_ABSOLUTE;
sFocusControl.value = m_unCameraFocusAbsolute;
if (::ioctl(m_nCameraHandle, VIDIOC_S_CTRL, &sFocusControl) < 0)
   THROW_ARGOSEXCEPTION("Could not set camera focus");
```
The simulation implementation should also be updated to reflect how changing the focus influences tag detection so that there is reasonable correspondence between how the sensor works in the simulation and how it works on the real robot.

## Capture is limited to a single camera
On the real Pi-Puck, it is currently only possible to capture from either the front camera or the omnidirectional camera. This limitation is due to libcamera and its V4L2 compatibility layer which is loaded automatically to support the omnidirectional camera. The solution to this problem would be to use gstreamer in the camera sensors instead of directly using V4L2 (which is how the current camera sensors are implemented).

## Tag detection is very slow
The CPU on the Raspberry Pi Zero W is not the fastest (approximately the speed of the first generation of Raspberry Pi boards) and it can take almost a second to detect tags from a 640x480 resolution image. The solution to this problem would be to upgrade to the [Raspberry Pi Zero 2 W](https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/), which contains a quad-core processor and is probably capable of reducing the processing time down to around 200 milliseconds. The Raspberry Pi Zero 2 W appears to be mechanically compatible with the Raspberry Pi Zero W, so it should be compatible with the existing Pi-Puck expansion PCBs.

