#include "screw_detection_training_node/screw_detection_training_ros.h"
#include <ros/ros.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "screw_trainer");
    ros::NodeHandle nh;

    screw_detection::ROSScrewTrainer trainer(nh);

    trainer.run();

    return 0;
}
