/**
 * @file
 * @brief This file contains the declaration of the custom DonutROI class.
 *
 * @author Sebastian Döbler
 * @version 1.0
 */

#ifndef SCREW_DETECTION_DONUT_ROI_H
#define SCREW_DETECTION_DONUT_ROI_H

//OpenCV
#include <opencv2/opencv.hpp>

//STD
#include <stdexcept>

//Self
#include <screw_detection/extractor_parameters.h>
#include <screw_detection/roi_extractor.h>

namespace screw_detection {

/** DonutROI
 * @brief extracts a donut shaped ROI defined by a center radius and tolerances from
 * a circular base.
*/
class DonutROI : public CustomROI {
public:
    /**
     * @brief HoughParameters to detect the base circle and the radius on which the
     * screws are located
    */
    DonutROI(const HoughParameters& parameters, const CM& screw_radius);

    /**
     * @brief The returned ROI is an image containing the cut out donut from the passed
     * image. The reference is the center of the circle
    */
    virtual cv::Mat getROI(cv::Vec2f& reference, const cv::Mat& image, const Pixel& max_circle_size);

    /**
     * @brief Calculates internal parameters from CM values using ppc
    */
    virtual void initiateParameters(const float& ppc);

    //Misc

    const CM& screw_radius() const { return this->screw_radius_cm_; }
    const HoughParameters& table_parameters() const { return this->table_parameters_; }
    const Pixel& screw_radius_px() const
    {
        if (!initiated_) {
            throw std::logic_error("The ROIParameters were not initiated, thus this variable is not availabl!");
        }
        return this->screw_radius_px_;
    }
    const Pixel& screw_radius_px_max() const
    {
        if (!initiated_) {
            throw std::logic_error("The ROIParameters were not initiated, thus this variable is not availabl!");
        }
        return this->screw_radius_px_max_;
    }
    const Pixel& screw_radius_px_min() const
    {
        if (!initiated_) {
            throw std::logic_error("The ROIParameters were not initiated, thus this variable is not availabl!");
        }
        return this->screw_radius_px_min_;
    }

private:
    bool initiated_ = false;
    HoughParameters table_parameters_;

    //ROI Raw
    CM screw_radius_cm_; //radius on which to find ROI

    //ROI Calculated
    Pixel screw_radius_px_;
    Pixel screw_radius_px_max_;
    Pixel screw_radius_px_min_;

    //Constants
    const float SIZE_TO_MAX_SIZE_SCREW_RADIUS = 1.12;
    const float SIZE_TO_MIN_SIZE_SCREW_RADIUS = 0.85;
};

}
#endif
