cmake_minimum_required(VERSION 2.8.3)
project(screw_detection_msgs)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    message_generation
    geometry_msgs
    std_msgs
)


################################################
############# Declare ROS services #############
################################################

## Generate actions in the 'msg' folder
add_message_files(
    FILES
        Screw.msg
        DetectionResult.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
        geometry_msgs
        std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## CATKIN_DEPENDS: catkin_packages dependent projects also need
catkin_package(
    CATKIN_DEPENDS
        message_runtime
        geometry_msgs
        std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${catkin_INCLUDE_DIRS}
)
