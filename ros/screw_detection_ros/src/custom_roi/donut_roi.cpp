#include "donut_roi.h"

namespace screw_detection {
DonutROI::DonutROI(const HoughParameters& parameters, const CM& screw_radius)
    : table_parameters_(parameters)
    , screw_radius_cm_(screw_radius)
{
}

void DonutROI::initiateParameters(const float& ppc)
{
    table_parameters_.initiateParameters(ppc, Single);

    //Convert raw inputs to Pixel measures and apply tolerances defined as constants
    screw_radius_px_ = screw_radius_cm_ * ppc;
    screw_radius_px_max_ = screw_radius_px_ * SIZE_TO_MAX_SIZE_SCREW_RADIUS;
    screw_radius_px_min_ = screw_radius_px_ * SIZE_TO_MIN_SIZE_SCREW_RADIUS;
    initiated_ = true;
}

cv::Mat DonutROI::getROI(cv::Vec2f& reference, const cv::Mat& image, const Pixel& max_circle_size)
{
    //Get the base table circle
    std::vector<cv::Vec3f> table_circle;
    roi_utils::getCirclesHough(table_circle, image, table_parameters_);

    if (table_circle.size() == 0) {
        throw std::runtime_error("\e[1;31m No table found!\e[0m");
    }

    reference = cv::Vec2f(table_circle[0][0], table_circle[0][1]); //[0][0] := x of circle zero

    if (table_circle.size() != 1) {
        throw std::runtime_error("\e[1;31m More than one table found!\e[0m");
    }

    cv::Scalar foreground = cv::Scalar(255, 255, 255);
    cv::Scalar background = cv::Scalar(0, 0, 0);
    cv::Mat donut_mask = cv::Mat(image.rows, image.cols, image.type(), background);

    //When considering the ROI it accounts for all screws whos center is inside the ROI
    //Thus extends the ROI by the max possible screw size
    int outer_circle_radius = screw_radius_px_max() + max_circle_size / 2;
    int inner_circle_radius = screw_radius_px_min() - max_circle_size / 2;

    //The mask starts as all background, the first circle applies a filled circle
    //as foreground, and the second circle accounts for the hole ine the middle of
    //donut by applying the background
    cv::circle(donut_mask, cv::Point(reference[0], reference[1]),
        screw_radius_px_max(), foreground, -1);
    cv::circle(donut_mask, cv::Point(reference[0], reference[1]),
        screw_radius_px_min(), background, -1);

    cv::Mat roi;
    image.copyTo(roi, donut_mask); //Copies area inside of donut mask of image into roi
    return roi;
}

}
