#include "screw_detection_node/screw_detection_ros.h"
#include <ros/ros.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "screw_detector");
    ros::NodeHandle nh;

    screw_detection::ROSScrewDetector detector(nh);

    detector.run();

    ros::spin();

    return 0;
}
