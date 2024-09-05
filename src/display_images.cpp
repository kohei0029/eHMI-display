#include "../include/display/display_images.hpp"

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>

#include <interactive_rmpc/RobotState.h>
#include <interactive_rmpc/PedestrianState.h>
#include <interactive_rmpc/MPCStates.h>

#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

DisplayImages::DisplayImages(): _nh(""), _pnh("")
{
    std::string ehmi_judgment_in_topic, ehmi_display_image_out_topic;
    _pnh.param<std::string>("ehmi_judgment_in",ehmi_judgment_in_topic,"/eHMI/judgment");
    _pnh.param<std::string>("ehmi_display_image_out", ehmi_display_image_out_topic, "/eHMI/display_image");
    
    _judgment_sub = _nh.subscribe(ehmi_judgment_in_topic, 1, &DisplayImages::ehmi_judgment_callback, this);
    _display_image_pub = _nh.advertise<sensor_msgs::Image>(ehmi_display_image_out_topic, 1);
}

DisplayImages::~DisplayImages(){}

void DisplayImages::add_information_contents(const std_msgs::String& judgment, cv::Mat& information_contents)
{
    //imagesフォルダの絶対パスを取得
    std::string package_path = ros::package::getPath("display");
    std::string images_path = package_path + "/images/";
    std::string robot_first_image_name, pedestrian_first_image_name;
    //パタメータ（表示内容のファイル名）
    _pnh.param<std::string>("robot_first_image_name",robot_first_image_name,"robot_first_pictogram_kousuke.png");
    _pnh.param<std::string>("pedestrian_first_image_name",pedestrian_first_image_name,"pedestrian_first_pictogram_kousuke.png");

    //画像のパスを取得
    std::string robot_first_image_path, pedestrian_first_image_path;
    robot_first_image_path = images_path + robot_first_image_name;
    pedestrian_first_image_path = images_path + pedestrian_first_image_name;

    //画像の読み込み
    cv::Mat robot_first_image = cv::imread(robot_first_image_path, cv::IMREAD_COLOR);
    cv::Mat pedestrian_first_image = cv::imread(pedestrian_first_image_path, cv::IMREAD_COLOR);

    //画像の決定
    if (judgment.data == "Robot will go first.")
    {
        information_contents = robot_first_image;
    }
    else if (judgment.data == "Pedestrian will go first.")
    {
        information_contents = pedestrian_first_image;
    }
    else
    {
        information_contents = cv::Mat();
    }
}

void DisplayImages::ehmi_judgment_callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I received Judgment : %s", msg->data.c_str());

    //画像の取得
    cv::Mat information_contents;
    add_information_contents(*msg, information_contents);

    //画像の表示とパブリッシュ
    if (!information_contents.empty()) 
    {
        cv::imshow("Display Image", information_contents);
        sensor_msgs::ImagePtr display_image;
        display_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", information_contents).toImageMsg();
        _display_image_pub.publish(display_image);
    }

    // cv::namedWindow("Display Image", cv::WINDOW_NORMAL);
    // cv::startWindowThread();
    // cv::Mat display = cv_bridge::toCvCopy(information_contents, "bgr8")->image;
    // cv::imshow("Display Image", information_contents);
    // cv::waitKey(30);

    // sensor_msgs::ImagePtr display_image;
    // display_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", information_contents).toImageMsg();
    // _display_image_pub.publish(display_image);
}
