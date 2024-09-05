#include "../include/display/display_images.hpp"
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


int main(int argc,char **argv)
{
    ros::init(argc, argv, "display_images");
    cv::namedWindow("Display Image", cv::WINDOW_NORMAL);
    //cv::startWindowThread();

    DisplayImages di;
    ros::Rate loop_rate(30);

    //ros::spin();
    while (ros::ok()) {
        ros::spinOnce();  // ROSのイベントを処理
        cv::waitKey(30);  // OpenCVのイベントを処理
        loop_rate.sleep();
    }

    cv::destroyWindow("Display Image");

    return 0;
}