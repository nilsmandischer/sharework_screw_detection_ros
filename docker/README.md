# Screw Detection Docker 

## Requirements

- docker

## Usage

This Docker image can be used for building and executing the screw_detection_ros package.

The image comes with the required packages and libraries to run all nodes.

### Build

To build the image, execute 
```
docker build -t screw_detection_docker . 
```

### Quick Start Guide
0. Take some training images containing the whole ROI and screws and place them in a folder.
1. Build the docker using: 
```
docker build -t screw_detection_docker .
```
2. Run the following command __on your local machine__ to allow docker to forward its display (This needs to be done every time you restart your pc before running the container):
```
xhost +local:docker
```
2. Run the docker using (fill in the <path_to_your_images>):
```
docker run -it -v <path_to_your_images>:/images -v ~/.ssh:/root/.ssh --name screw_detection --privileged --net host -e DISPLAY=$DISPLAY --env ROS_MASTER_URI=http://localhost:11311 screw_detection_docker
```
3. Export the necessary variables to the environment to pass as arguments for later bash functions:
```
export SD_FILE_NAME=image_ && \
export SD_SUFFIX=.jpg && \
export SD_FIRST_NUMBER=1 && \
export SD_LAST_NUMBER=35
```
The image files need to have the naming format: <br />
SD_FILE_NAME + <NUMBER> + SD_SUFFIX <br />
With the numbers ranging from SD_FIRST_NUMBER to SD_LAST_NUMBER.
4. Launch the calibration to set your parameters (refer to the Calibration section of screw_detection_ros's README for precise instructions on how to adjust the parameters) and follow the instructions inside the terminal:
```
roslaunch /catkin_ws/src/sharework_screw_detection_ros/docker/quick_launch/calibration.launch data_folder:=$SD_DATA_FOLDER file_name:=$SD_FILE_NAME suffix:=$SD_SUFFIX first_number:=$SD_FIRST_NUMBER last_number:=$SD_LAST_NUMBER
```
5. Transfer your chosen parameters from the rqt window to the following file:
```
gedit /catkin_ws/src/sharework_screw_detection_ros/ros/screw_detection_ros/config/detection_common.yaml
```
6. Launch the following launch file for cropping the unlabeled images and follow the instrcutions inside the terminal:
```
roslaunch /catkin_ws/src/sharework_screw_detection_ros/docker/quick_launch/crop_data.launch data_folder:=$SD_DATA_FOLDER file_name:=$SD_FILE_NAME suffix:=$SD_SUFFIX first_number:=$SD_FIRST_NUMBER last_number:=$SD_LAST_NUMBER
```
7. Launch the following launch file for training the Model:
```
roslaunch /catkin_ws/src/sharework_screw_detection_ros/docker/quick_launch/train.launch data_folder:=$SD_DATA_FOLDER file_name:=$SD_FILE_NAME suffix:=$SD_SUFFIX first_number:=$SD_FIRST_NUMBER last_number:=$SD_LAST_NUMBER
```
8. Preview a Demo with the provided images and follow the instructions inside the terminal:
```
roslaunch /catkin_ws/src/sharework_screw_detection_ros/docker/quick_launch/demo.launch data_folder:=$SD_DATA_FOLDER file_name:=$SD_FILE_NAME suffix:=$SD_SUFFIX first_number:=$SD_FIRST_NUMBER last_number:=$SD_LAST_NUMBER
```
9. The created model_data.yaml and misc_data.yaml file can now be used to run with your own setup online. This requires ROS installed on your machine.

### Run 

To run the nodes, execute 
```
docker run -it -v <path_to_your_images>:/images -v ~/.ssh:/root/.ssh --name screw_detection --privileged --net host -e DISPLAY=$DISPLAY --env ROS_MASTER_URI=http://localhost:11311 screw_detection_docker
```
with your custom variables. The "<path_to_your_images>" corresponds to a folder containing images to use. The folder will be mounted into the docker and contain all the files generated by the docker. Be aware that you have to __adjust the launch files corresponding on the directories inside the docker__ (/images/...). 
Additionally the ROS_MASTER_URI can be replaced by your local rosmaster if you wish to do so. Afterwards you end up in a bash terminal in the catkin workspace where you can launch the nodes from the terminal as you would do without docker. For further information regarding this refer to the screw_detection_ros documentation and the Quick Start Guide.

It is possible to attach more terminals to the same docker session in case more than one launchfile is necessary. For this use:
```
docker exec -it screw_detection /bin/bash
```
and execute the additional commands.
If you close the docker use:
```
docker start -i screw_detection
```
to restart an interactive session.