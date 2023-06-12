cmake_minimum_required(VERSION 2.8.3)
project(screw_detection_ros)
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17")

find_package(OpenCV REQUIRED)

include(cmake/FindScrewDetection.cmake)
find_package(ScrewDetection CONFIG REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    cv_bridge
    roscpp
    sensor_msgs
    dynamic_reconfigure
    screw_detection_msgs
)

## Generate dynamic reconfigure parameters in the 'config' folder
generate_dynamic_reconfigure_options(
    cfg/ScrewDetectorParams.cfg
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ros_utils_lib
    CATKIN_DEPENDS
        roscpp
        sensor_msgs
        cv_bridge
        geometry_msgs
        dynamic_reconfigure
        screw_detection_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#Add ROS Utils
add_library(ros_utils_lib
    src/ros_utils/ros_utils.cpp
)
target_include_directories(ros_utils_lib
    PUBLIC
        include
        ${catkin_INCLUDE_DIRS}
)
target_link_libraries(ros_utils_lib
    ${catkin_LIBRARIES}
)

#Add Screw Detection Node
add_executable(screw_detection_node
    src/screw_detection_node.cpp
    src/screw_detection_node/screw_detection_ros.cpp
    src/custom_roi/donut_roi.cpp
)
target_include_directories(screw_detection_node
    PUBLIC
        include
        ${catkin_INCLUDE_DIRS}
    PRIVATE
        src
)
target_link_libraries(screw_detection_node
    ros_utils_lib
    screw_detection
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)
add_dependencies(screw_detection_node
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
)
        
#Add Screw Detection Training Node
add_executable(screw_detection_training_node
    src/screw_detection_training_node.cpp
    src/screw_detection_training_node/screw_detection_training_ros.cpp
    src/custom_roi/donut_roi.cpp
)
target_include_directories(screw_detection_training_node
    PUBLIC
        include
        ${catkin_INCLUDE_DIRS}
    PRIVATE
        src
)
target_link_libraries(screw_detection_training_node
    ros_utils_lib
    screw_detection
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)
add_dependencies(screw_detection_training_node
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

######
###INSTALL

install(TARGETS ros_utils_lib screw_detection_node screw_detection_training_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/screw_detection_ros/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
