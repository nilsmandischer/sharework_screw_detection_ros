#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
//STD C++ INCLUDES
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>

// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

//for getch
#include <termios.h>
#define ICANON 0000002

std::string data_folder_;
std::string filename_ = "image_";
std::string suffix_ = ".png";

int first_number = 0;
int last_number = -1;

std::vector<cv::Mat> input;
image_transport::Publisher image_pub;
int current_input = 0;

bool loadData()
{
    ROS_INFO("Starting to import Images");

    std::string image_path = data_folder_;

    // Container for loop
    std::string file_name;

    cv::Mat image;

    uint image_count = 0;

    ROS_INFO("Loading images from %s.", image_path.c_str());
    if (!std::filesystem::exists(image_path)) {
        ROS_ERROR("Image folder does not exist!");
        return false;
    }

    for (uint t_id = first_number;; t_id++) {
        file_name = "/" + filename_ + std::to_string(t_id) + suffix_;
        image = cv::imread(image_path + file_name);

        if (!image.empty()) {
            input.push_back(image);
            image_count++;
        } else {
            if (last_number != -1) {
                if (t_id >= last_number) {
                    break;
                }
            } else {
                break;
            }
        }
    }

    if (image_count != 0) {
        ROS_INFO("Loaded %i images.",
            image_count);
    } else {
        ROS_ERROR("Did not load any images!");
        return false;
    }
    return true;
}

void timerCallback(const ros::TimerEvent& event)
{
    //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", input[current_input]).toImageMsg();

    sensor_msgs::Image msg;

    //Get image
    std_msgs::Header header;
    cv_bridge::CvImage cv_image = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, input[current_input]);
    cv_image.toImageMsg(msg);

    //Set header
    msg.header.frame_id = "camera_rgb";
    msg.header.stamp = ros::Time::now();

    image_pub.publish(msg);
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

void activateTimer(ros::NodeHandle nh)
{
    ros::Timer timer = nh.createTimer(ros::Duration(0.2), &timerCallback);
    ros::spin();
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh("~");

    std::string image_topic;

    if (!nh.getParam("image_topic", image_topic)) {
        ROS_ERROR("No data_folder provided, aborting!");
        return 0;
    }

    if (!nh.getParam("data_folder", data_folder_)) {
        ROS_ERROR("No data_folder provided, aborting!");
        return 0;
    }
    if (!nh.getParam("filename", filename_)) {
        ROS_ERROR("No file_name provided, aborting!");
        return 0;
    }
    if (!nh.getParam("suffix", suffix_)) {
        ROS_ERROR("No suffix provided, aborting!");
        return 0;
    }
    if (!nh.getParam("first_number", first_number)) {
        ROS_ERROR("No first_number provided, using 0!");
    }
    if (!nh.getParam("last_number", last_number)) {
        ROS_ERROR("No last_number provided, aborting when not continuously numbered!");
    }

    if (!loadData()) {
        ROS_ERROR("Failed loading data, aborting!");
        return 0;
    }

    image_transport::ImageTransport it(nh);
    image_pub = it.advertise(image_topic, 1, true);
    std::thread t1(activateTimer, nh);

    int key = 0;
    ROS_INFO("\e[1;35m Press n to progress to next data timestamp, f to return to the first input or q to quit node.\e[0m");

    while (ros::ok()) {
        key = getch();
        if ((key == 'n') || (key == 'N')) {
            if (current_input != input.size() - 1) {
                current_input++;
                ROS_INFO("Currently at timestamp %i.", current_input);
            } else {
                ROS_WARN("Reached the last input, currently at #%i", current_input);
            }
        }
        if ((key == 'f') || (key == 'F')) {
            current_input = 0;
            ROS_INFO("Currently at timestamp %i.", current_input);
        }
        if (key == 'q') {
            break;
        }
    }

    t1.join();
    return 0;
}
