# Summary
This repository is an external plugin for use with the Pi-Puck in ARGoS3. It provides an omnidirectional camera sensor capable of locating Apriltags.

# Quick start

## Compilation
If you are working with the hardware, the [meta-pipuck](https://github.com/iridia-ulb/meta-pipuck) layer will automatically install this plugin for you. If you are working in simulation, you will need to the follow the steps to make the camera available to ARGoS:

```sh
# clone this repository
git clone https://github.com/iridia-ulb/argos3-pipuck-omnidirectional-camera.git
# compile
cd argos3-pipuck-omnidirectional-camera
mkdir build
cd build
cmake ../src
make
# run
argos3 -c configuration.argos
```

