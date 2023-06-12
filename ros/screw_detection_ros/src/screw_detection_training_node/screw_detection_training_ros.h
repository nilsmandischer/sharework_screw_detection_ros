/**
 * @file
 * @brief This file contains the declaration of the ROSScrewTrainer class.
 *
 * @author Sebastian Döbler
 * @version 1.0
 */

#ifndef SCREW_DETECTION_TRAINING_NODE_H
#define SCREW_DETECTION_TRAINING_NODE_H

// ROS
#include <ros/ros.h>

// OpenCV
#include <opencv2/opencv.hpp>

// STD
#include <fstream>
#include <string>
#include <vector>

// Screw Detector
#include <custom_roi/donut_roi.h>
#include <screw_detection/detection.h>
#include <screw_detection/training.h>

//ROS Utils
#include <ros_utils/ros_utils.h>

using namespace ros_utils;

namespace screw_detection {

/**
 * @brief ROSScrewTrainer
 * is the ros wrapper for the ScrewTrainer class.
 */
class ROSScrewTrainer {

public:
    /**
   * @brief Constructor that copies a pointer to the node handle, loads all parameters
   * from the parameter server and creates first all necessary parameter storage
   * objects for ScrewTrainer, including the DonutROI, and creates the
   * ScrewTrainer object
   */
    ROSScrewTrainer(ros::NodeHandle& nh);

    /**
   * @brief Either calls ScrewTrainer::spliceImages() or ScrewTrainer::trainModel()
   * depending on save_cut_training_data
   */
    void run(void);

private:
    // ROS
    ros::NodeHandle* nh_ptr_;

    // Trainer
    std::unique_ptr<ExtractorParameters> extractor_parameters_;
    std::unique_ptr<ScrewTrainer> trainer_;

    //Control
    bool save_cut_training_data_;

    // Functions
    /**
   * @brief Loads the parameters from the parameter server and creates all objects
   * including the ScrewTrainer
   */
    void loadParameters(void);
};
} // namespace screw_detection_training

#endif
