cmake_minimum_required(VERSION 2.8.3)
project(image_utilities)


add_compile_options(-std=c++17)

############### dependencies ######################
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# opencv
find_package(OpenCV REQUIRED )

#ros
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    roscpp
    image_transport
    sensor_msgs
    std_msgs
)
catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        std_msgs
)

############### Include path ######################
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
link_directories(${OpenCV_LIBRARY_DIR})

add_executable(image_publisher
    src/image_publisher.cpp
)
add_dependencies(image_publisher
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(image_publisher
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

add_executable(image_recorder
    src/image_recorder.cpp
)
add_dependencies(image_recorder
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(image_recorder
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

######
###INSTALL

install(TARGETS image_recorder image_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/image_utilities/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)