/**
 * @file
 * @brief This file contains the declaration of the ROSScrewDetector class.
 *
 * @author Sebastian Döbler
 * @version 1.0
 */

#ifndef SCREW_DETECTION_NODE_H
#define SCREW_DETECTION_NODE_H

//ROS
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <screw_detection_ros/ScrewDetectorParamsConfig.h>

//Msgs
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

//ROS
#include <geometry_msgs/Point.h>

//Custom MSGs
#include <screw_detection_msgs/DetectionResult.h>
#include <screw_detection_msgs/Screw.h>

//OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp> //For cv::Filled
#include <opencv2/opencv.hpp>

//STD
#include <filesystem>
#include <string>
#include <vector>

//ROS Utils
#include <ros_utils/ros_utils.h>

//Self
#include <custom_roi/donut_roi.h>
#include <screw_detection/detection.h>
#include <screw_detection/roi_extractor.h>

using namespace ros_utils;

namespace screw_detection {

/**
 * @brief ROSScrewDetector
 * This class is the ros wrapper for screw_detection::ScrewDetector.
 */
class ROSScrewDetector {
public:
    /**
   * @brief Constructor that copies a pointer to the node handle.
   * @param  nh Nodehandle of the created node
   */
    ROSScrewDetector(ros::NodeHandle& nh);

    /**
   * @brief Initiates parameters, creates Detector and establishes subscriber callbacks
   */
    void run(void);

private:
    //Control
    const bool DEBUG_MODE_ = true;

    //ROS
    ros::NodeHandle* nh_ptr_;

    ros::Subscriber image_subscriber_;

    ros::Publisher result_publisher_;
    ros::Publisher roi_publisher_;
    ros::Publisher roi_contour_publisher_;
    ros::Publisher screw_publisher_;

    //Reconfigure
    using reconf_config_t = screw_detection_ros::ScrewDetectorParamsConfig;
    using reconf_server_t = dynamic_reconfigure::Server<reconf_config_t>;
    reconf_server_t reconf_server_;

    bool update_allowed_ = false;

    //Topics
    std::string p_image_sub_topic_;
    std::string p_camera_frame_name_;
    std::string p_model_data_path_;

    //Screw Detector
    std::shared_ptr<ExtractorParameters> extractor_parameters_;

    std::unique_ptr<ScrewDetector> detector_;

    //Functions
    /**
    * @brief Loads the parameters from the parameter server.
    */
    void loadParameters(void);

    /**
   * @brief Publishes circles drawn on the ROI and the ROI bounds on the original
   * image
   * @param  og_image Original image received
   * @param  roi Extracted region of interest
   * @param  circles Circles to draw
   * @param  roi_reference to draw the ROI bounds
   */
    void publishCircles(const sensor_msgs::Image& og_image, const cv::Mat& roi,
        const std::vector<cv::Vec3f>& circles, const cv::Vec2f& roi_reference);

    /**
   * @brief Callback for dynamic reconfigure.
   */
    void reconfigureCallback(screw_detection_ros::ScrewDetectorParamsConfig& config, uint32_t level);

    /**
   * @brief Publishes coloured image with screws highlighted
   * @param  image Image to be drawn on
   * @param  circles Circles which each represent a screw
   * @param  are_screws Vector with each element corresponding to an element in circles, true if circle is screw
   * @param  og_image container for the encoding used to publish the image
   */
    bool publishScrews(const cv::Mat& image, std::vector<cv::Vec3f>& circles,
        std::vector<bool>& are_screws, const sensor_msgs::Image& og_image);

    /**
   * @brief Image entry point. Calls detection on received image and publishes results
   */
    void imageCallback(const sensor_msgs::Image& msg);
};

}

#endif
