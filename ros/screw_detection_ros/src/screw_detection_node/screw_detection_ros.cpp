#include "screw_detection_ros.h"

namespace screw_detection {

ROSScrewDetector::ROSScrewDetector(ros::NodeHandle& nh)
{
    //Copy node handle pointer
    nh_ptr_ = &nh;

    //Load dynamic reconfigure
    reconf_server_t::CallbackType dyn_reconf_callback;
    dyn_reconf_callback = boost::bind(&ROSScrewDetector::reconfigureCallback, this, _1, _2);
    reconf_server_.setCallback(dyn_reconf_callback);

    //Load parameters from parameter server
    loadParameters();

    //Initialize publishers
    result_publisher_ = nh_ptr_->advertise<screw_detection_msgs::DetectionResult>(ros::this_node::getName() + "/result", 1, true);

    if (DEBUG_MODE_) {
        roi_publisher_ = nh_ptr_->advertise<sensor_msgs::Image>(ros::this_node::getName() + "/roi", 1, true);
        roi_contour_publisher_ = nh_ptr_->advertise<sensor_msgs::Image>(ros::this_node::getName() + "/roi_contours", 1, true);
        screw_publisher_ = nh_ptr_->advertise<sensor_msgs::Image>(ros::this_node::getName() + "/screw_image", 1, true);
    }
}

void ROSScrewDetector::loadParameters()
{
    //Control
    ros::NodeHandle pnh(ros::this_node::getName());
    readParameter("model_data_path", p_model_data_path_, pnh);
    readParameter("camera_frame_name", p_camera_frame_name_, pnh);

    //Camera
    CM camera_height;
    float camera_angle;
    readParameter("camera_height", camera_height, pnh);
    readParameter("camera_angle", camera_angle, pnh);
    ImageParameters image_parameters(camera_height, camera_angle);

    //Hough Screws
    CM screw_size;
    int contour_threshold_screw;
    int accumulator_threshold_screw;
    readParameter("hole_size_cm", screw_size, pnh);
    readParameter("contour_threshold", contour_threshold_screw, pnh);
    readParameter("accumulator_threshold", accumulator_threshold_screw, pnh);
    HoughParameters screw_parameters(screw_size, accumulator_threshold_screw, contour_threshold_screw);

    //Donut ROI
    CM table_size;
    int contour_threshold_table;
    int accumulator_threshold_table;
    readParameter("table_size_cm", table_size, pnh);
    readParameter("accumulator_threshold_table", accumulator_threshold_table, pnh);
    readParameter("contour_threshold_table", contour_threshold_table, pnh);
    HoughParameters table_parameters(table_size, accumulator_threshold_table, contour_threshold_table);

    CM screw_radius_cm;
    readParameter("screw_radius_cm", screw_radius_cm, pnh);
    std::shared_ptr<DonutROI> donut_roi(new DonutROI(table_parameters, screw_radius_cm));

    extractor_parameters_.reset(new ExtractorParameters(screw_parameters, image_parameters, donut_roi));

    //Topics
    readParameter("image_sub_topic", p_image_sub_topic_, pnh);

    //IO
    ROS_INFO("Loaded Parameters!");
}

void ROSScrewDetector::reconfigureCallback(screw_detection_ros::ScrewDetectorParamsConfig& config, uint32_t level)
{
    //Required to not call this before the Extractor was first initialized
    if (update_allowed_) {
        //Creates all necessary objects for the ScrewDetector
        HoughParameters screw_parameters(static_cast<float>(config.screw_size_cm), config.accumulator_threshold,
            config.contour_threshold);
        HoughParameters table_parameters(static_cast<float>(config.table_size_cm), config.accumulator_threshold_table,
            config.contour_threshold_table);
        std::shared_ptr<DonutROI> donut_roi(new DonutROI(table_parameters, static_cast<float>(config.screw_radius_cm)));

        ImageParameters image_parameters(extractor_parameters_->image_parameters());

        extractor_parameters_.reset(new ExtractorParameters(screw_parameters, image_parameters, donut_roi));

        extractor_parameters_->initiateParameters(extractor_parameters_->image_parameters().image_height(),
            extractor_parameters_->image_parameters().image_width());

        detector_->setParameters(*extractor_parameters_);
    }
}

void ROSScrewDetector::run()
{
    ROS_INFO("Ready to receive images!");

    sensor_msgs::Image::ConstPtr image;

    //Wait for he first image to get height and width of the image
    //To init the parameters
    while (image == NULL) {
        image = ros::topic::waitForMessage<sensor_msgs::Image>(p_image_sub_topic_, ros::Duration(3));
        if (image == NULL) {
            ROS_ERROR("Waiting for first image to set up ExtractorParameters!");
        }
    }
    //Create detector
    extractor_parameters_->initiateParameters((int)image->height, (int)image->width);
    detector_.reset(new ScrewDetector(*extractor_parameters_, p_model_data_path_));
    update_allowed_ = true;

    image_subscriber_ = nh_ptr_->subscribe(p_image_sub_topic_, 1, &ROSScrewDetector::imageCallback, this);
}

void ROSScrewDetector::imageCallback(const sensor_msgs::Image& msg)
{
    ROS_INFO("Processing image");

    cv::Mat image = cv_bridge::toCvCopy(msg)->image.clone();

    std::vector<cv::Vec3f> circles;
    cv::Vec2f roi_reference;
    std::vector<bool> are_screws;
    cv::Mat roi;
    bool result = detector_->processImage(circles, roi_reference, roi, are_screws, image);
    if (DEBUG_MODE_) {
        publishCircles(msg, roi, circles, roi_reference);
    }
    if (result) {
        publishScrews(image, circles, are_screws, msg);
    }
}

void ROSScrewDetector::publishCircles(const sensor_msgs::Image& og_image, const cv::Mat& roi,
    const std::vector<cv::Vec3f>& circles, const cv::Vec2f& roi_reference)
{
    //ROI containing circles
    cv::Mat roi_image = roi.clone();
    for (size_t i = 0; i != circles.size(); i++) {
        cv::Vec3i circ = circles[i];
        cv::Point center = cv::Point(circ[0], circ[1]);

        // circle center
        cv::circle(roi_image, center, 1, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);

        // circle outline
        int radius = circ[2];
        cv::circle(roi_image, center, radius, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
    }

    //Full image containing DonutROI outlines
    cv::Mat roi_contour_image = cv_bridge::toCvCopy(og_image)->image.clone();
    if (roi_reference[0] != 0) {
        cv::circle(roi_contour_image, cv::Point(roi_reference[0], roi_reference[1]), 5, cv::Scalar(50, 205, 154), 3, cv::LINE_AA);

        DonutROI* donut_ref = dynamic_cast<DonutROI*>(&(*(extractor_parameters_->roi_parameters())));

        cv::circle(roi_contour_image, cv::Point(roi_reference[0], roi_reference[1]),
            donut_ref->table_parameters().circle_size_px(), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        cv::circle(roi_contour_image, cv::Point(roi_reference[0], roi_reference[1]), donut_ref->screw_radius_px_max(),
            cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
        cv::circle(roi_contour_image, cv::Point(roi_reference[0], roi_reference[1]), donut_ref->screw_radius_px_min(),
            cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
    }

    //Publish both
    cv_bridge::CvImage image_w_roi;
    image_w_roi.encoding = og_image.encoding;
    image_w_roi.image = roi_image.clone();
    roi_publisher_.publish(image_w_roi.toImageMsg());

    cv_bridge::CvImage image_w_contours;
    image_w_contours.encoding = og_image.encoding;
    image_w_contours.image = roi_contour_image.clone();
    roi_contour_publisher_.publish(image_w_contours.toImageMsg());
}

bool ROSScrewDetector::publishScrews(const cv::Mat& image, std::vector<cv::Vec3f>& circles, std::vector<bool>& are_screws,
    const sensor_msgs::Image& og_image)
{
    //Image
    if (DEBUG_MODE_) {
        cv::Mat circle_image = image.clone();
        for (uint c_id = 0; c_id != circles.size(); c_id++) {
            cv::Vec3f circle = circles[c_id];

            //Draw circle
            int radius = circle[2];
            cv::Point center = cv::Point(circle[0], circle[1]);

            if (are_screws[c_id]) {
                cv::circle(circle_image, center, radius + 10, cv::Scalar(0, 255, 0), 5, cv::LINE_AA);
            } else {
                cv::Point corner1(center.x - (radius + 10), center.y - (radius + 10));
                cv::Point corner2(center.x + (radius + 10), center.y + (radius + 10));
                cv::rectangle(circle_image, corner1, corner2, cv::Scalar(0, 0, 255), 5, cv::LINE_AA);
            }
        }
        cv_bridge::CvImage circle_image_bridge;
        circle_image_bridge.encoding = og_image.encoding;
        circle_image_bridge.image = circle_image.clone();
        screw_publisher_.publish(circle_image_bridge.toImageMsg());
    }

    //Result msg
    screw_detection_msgs::DetectionResult result;
    result.header.frame_id = p_camera_frame_name_;

    for (uint c_id = 0; c_id != circles.size(); c_id++) {
        screw_detection_msgs::Screw new_screw;
        new_screw.result = are_screws[c_id];

        new_screw.point.x = (circles[c_id][0] - extractor_parameters_->image_parameters().image_width() / 2)
            / extractor_parameters_->image_parameters().ppc() / 100;
        new_screw.point.y = (circles[c_id][1] - extractor_parameters_->image_parameters().image_height() / 2)
            / extractor_parameters_->image_parameters().ppc() / 100;
        new_screw.point.z = extractor_parameters_->image_parameters().camera_height();

        result.screws.push_back(new_screw);
    }

    result.header.stamp = ros::Time::now();
    result_publisher_.publish(result);
    return true;
}
}
