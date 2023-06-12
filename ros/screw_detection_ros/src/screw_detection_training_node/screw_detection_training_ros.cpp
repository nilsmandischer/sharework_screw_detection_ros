#include "screw_detection_training_ros.h"

namespace screw_detection {

ROSScrewTrainer::ROSScrewTrainer(ros::NodeHandle& nh)
{
    //Copy node handle pointer
    nh_ptr_ = &nh;

    //Load parameters from parameter server
    loadParameters();
}

void ROSScrewTrainer::loadParameters()
{
    TrainerParameters trainer_params;
    ModelParameters model_params;

    //Control
    ros::NodeHandle pnh(ros::this_node::getName());
    readParameter("save_cut_training_data", save_cut_training_data_, pnh);

    if (save_cut_training_data_) {
        readParameter("image_name_prefix", trainer_params.image_prefix, pnh);
        readParameter("image_name_suffix", trainer_params.image_suffix, pnh);
        readParameter("image_starting_number", trainer_params.image_start_number, pnh);
    }

    //Trainer parameters
    readParameter("image_path", trainer_params.image_path, pnh);

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
    //Forest Parameters
    readParameter("max_depth", model_params.max_depth, pnh);
    readParameter("min_sample_count", model_params.min_sample_count, pnh);
    readParameter("max_iterations", model_params.max_iterations, pnh);
    readParameter("weight_screws", model_params.weight_screws, pnh);

    trainer_.reset(new ScrewTrainer(*extractor_parameters_, trainer_params, model_params));

    //IO
    ROS_INFO("Loaded Parameters!");
}

void ROSScrewTrainer::run()
{
    if (save_cut_training_data_) {
        trainer_->spliceImages();
    } else {
        trainer_->trainModel();
    }
}
}
