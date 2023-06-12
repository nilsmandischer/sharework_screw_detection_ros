# Screw Detection ROS

This package is a ROS wrapper for the screw_detection library in combination with a customized ROI for circular ROIs on concentric workpieces. This package also includes a docker image to run all included offline features if installing ROS on the operating machine is not an option, which also includes a quick start guide.

## Dependencies

The package has following external dependencies:
- OpenCV 

Uses following ROS packages:
- geometry_msgs
- cv_bridge
- roscpp
- sensor_msgs
- dynamic_reconfigure
- screw_detection_msgs

## Installation
Prerequisits: Installed all dependencies

First update all submodules
```console
git submodule init && git submodule update
```
Then build the screw_detection_library
```console
cd modules/sharework_screw_detection/screw_detection/ && mkdir build && cd build && cmake .. && make
```
and finally build the workspace

## Structure

The repository contains three packages:
- screw_detection_ros           << Nodes wrapping the screw detection library 
- screw_detection_msgs          << Messages published by the nodes
- image_utilities               << Nodes to record and publish images
<br />

The screw_detection_ros contains two nodes:
- Screw Detection Node
- Screw Detetcion Training Node
<br />

A common library with ROS utility functions:
- ros_utils
<br />

A CustomROI according to the screw_detection library:
- custom_roi/donut_roi

A docker which is able to run all offline features included in the package without having to install ROS, including a quick start guide:
- screw_detection_docker    << Refer to the README included in the ../docker/ folder for instructions

## Initial setup

1) Take training images and store them with increasing numbering (This can be done using the included image utilities package). The identifier and start number can be chosen freely, e.g. "image_1.jpg". These images contain the whole ROI and an arbitrary number of screws inside those images. It is recommended to either have screws in all possible holes or in none per image. This is to ensure an easier classification during training.
2) Adjust parameters in "files_common.yaml" to fit your file system.
3) Adjust parameters in "detection_common.yaml" to fit your real-world setup. These can be modified during the process with dynamic reconfigure, so don't worry if they are not 100% correct.

## Calibration

Optionally use a pre trained model (found under .../modules/sharework_screw_detection/examples/), the Screw Detection Node and dynamic reconfigure to adjust your parameters to work best for the scenario. The following command launches the Detection Node together with the recommended rviz configuration and a window to reconfigure the dynamic parameters. The same config files as for the normal Screw Detection Node are used, but "use_dynamic_reconfigure" and "model_data_path" are reset.
```console
roslaunch screw_detection_ros calibrate.launch
```
To make this as efficient as possible, use the included rviz configuration to first ensure that the ROI gets detected accurately by adjusting the parameters refering to the table, afterwards adjust the parameters refering to the screws. The best way to determin the parameters if unsure is, to first lower the contour threshold and the accumulator threshold to e.g. 10 each and then iteratively find out the size of the circle by changing the circle_size. As soon as the size of the circle is determined, the contour threshold and the accumulator threshold should be optimized to rule out all false detections if possible. These two parameters have to be optimized in parallel since they influence each other. Since the circles contours are rather prominent the contour threshold can be anywhere between 2x to 4x the accumulator threshold for the table and up to 10x for the screws as a rule of thumb.

## Usage
To use either node the corresponding launch file has to be adjusted first. <br />
Both Nodes share parameters corresponding to the circle detection under .../config/detection_common.yaml
and .../config/files_common.yaml
__Screw Detection Training Node:__

Has two main functions depending on the chosen launch file:
- Loads multiple images which show multiple possible screw candidates at once, detects circles on all images and classifies all found circles according to user input as Empty Holes, Screws or Errors. The classified circles get 
saved in corresponding subdirectories of the image folder.
- Loads all spliced images and trains a Model to classify circles as Empty holes or Screws. Training result gets saved in image folder as model_data.yaml

Prepare training data:
```console
roslaunch screw_detection_ros crop_data.launch
```
Train Model:
```console
roslaunch screw_detection_ros train.launch
```

- - - -

__Screw Detection Node:__

Subscribes to an image topic and performs circle detection followed by a classifier to find possible screws.
Publishes positions of found circles as Screw msgs containing geometry_msgs/Point for the location and classifier results as uint16 with:  <br />
_0 :=  Empty Hole, 1:= Screw_ <br />
Dynamic reconfigure can be used to adjust detection parameters on the fly to optimize circle detection results.

```console
roslaunch screw_detection_ros detector.launch
```
