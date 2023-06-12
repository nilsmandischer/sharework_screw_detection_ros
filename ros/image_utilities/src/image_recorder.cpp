#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

//STD C++ INCLUDES
#include <cstring>
#include <filesystem>
#include <iostream>

// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

//for getch
#include <termios.h>
#define ICANON 0000002

std::string data_folder_;
const std::string P_IMAGE_DIRECTORY_ = "/images";
std::string image_topic;

bool CheckOldFiles()
{
    // Remove image list
    if (std::filesystem::exists(data_folder_ + P_IMAGE_DIRECTORY_)) {
        ROS_ERROR("Data folder already exists! Please remove previous images");
        return false;
    } else
        std::filesystem::create_directory(data_folder_ + P_IMAGE_DIRECTORY_);
    return true;
}

cv::Mat toCVMatrix(const sensor_msgs::ImageConstPtr& image_msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return std::move(cv_ptr->image.clone());
    }
    return std::move(cv_ptr->image.clone());
}

bool storeTopicImage(const std::string& file)
{
    ros::Duration three_seconds(3.0);

    sensor_msgs::ImageConstPtr new_image = ros::topic::waitForMessage<sensor_msgs::Image>(image_topic,
        three_seconds);
    if (new_image == NULL) {
        ROS_ERROR("Recieved no image on %s!", image_topic.c_str());
        return false;
    }

    cv::Mat new_image_cv = toCVMatrix(new_image);

    if (imwrite(file, new_image_cv)) {
        return true;
    }

    return false;
}

bool takePicture(int timestamp)
{
    bool took_picture = true;
    std::string file_name, path;
    file_name = "/image_" + std::to_string(timestamp) + ".png";
    path = data_folder_ + P_IMAGE_DIRECTORY_ + file_name;

    if (storeTopicImage(path)) {
        ROS_INFO("Saving %s", file_name.c_str());
    } else {
        took_picture = false;
        ROS_WARN("Unable to grab camera feed for timestamp %i",
            timestamp);
    }

    return took_picture;
}

int getch()
{
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt); // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON); // disable buffering
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // apply new settings

    int c = getchar(); // read character (non-blocking)

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore old settings
    return c;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "image_recorder");
    ros::NodeHandle nh("~");

    if (!nh.getParam("image_topic", image_topic)) {
        ROS_ERROR("No image_topic provided, aborting!");
        return 0;
    }
    if (!nh.getParam("data_folder", data_folder_)) {
        ROS_ERROR("No data_folder provided, aborting!");
        return 0;
    }

    if (!CheckOldFiles())
        return 0;

    uint timestamp = 0;

    while (nh.ok()) {
        std::string user_input = "not_empty_or_q";
        do {
            std::cout << "Press Enter to take more pictures or q to quit. \n Currently at "
                    + std::to_string(timestamp) + " images!"
                      << std::endl;
            std::getline(std::cin, user_input);
        } while (!(user_input == "q" || user_input.length() == 0));

        if (user_input == "q") {
            break;
        }

        // Control parameters

        if (!takePicture(timestamp)) {
            continue;
        }

        timestamp++;
    }
    return 0;
}
