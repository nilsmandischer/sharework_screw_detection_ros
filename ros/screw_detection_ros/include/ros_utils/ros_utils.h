/**
 * @file
 * @brief This file contains functions commonly used in ros
 *
 * @author Sebastian Döbler
 * @version 1.0
 */

#ifndef ROS_UTILS_LIB
#define ROS_UTILS_LIB

//STD
#include <string>

//ROS
#include <ros/ros.h>

namespace ros_utils {
/**
     * @brief  Reads a parameter from the parameter server and handles errors accordingly
     * @param  member Container for the variable
     * @param  parameter_name Name of the parameter to be loaded
     * @param  pnh Private Nodehandle to get the Parameter
     * */
/*
     * Not return type first for readability
    */
template <typename T>
bool readParameter(const std::string& parameter_name, T& member, const ros::NodeHandle& pnh)
{
    // Load parameters from parameter server
    if (pnh.getParam(parameter_name, member)) {
        std::cout << "Successfully loaded parameter: " + pnh.getNamespace() + "/" + parameter_name.c_str()
                  << std::endl;
        return true;
    } else {
        std::cout << "\033[1;31mFailed to load parameter: \033[0m" + pnh.getNamespace() + "/" + parameter_name.c_str()
                  << std::endl;
        return false;
    }
}
};

#endif
