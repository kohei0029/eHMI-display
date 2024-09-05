#include <ros/ros.h>

#include <std_msgs/String.h> 

#include <interactive_rmpc/RobotState.h>
#include <interactive_rmpc/PedestrianState.h>
#include <interactive_rmpc/MPCStates.h>

#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


class DisplayImages
{
public:
    DisplayImages();
    ~DisplayImages();

private:
    ros::NodeHandle _nh;
    ros::NodeHandle _pnh;
    ros::Subscriber _judgment_sub;
    ros::Publisher _display_image_pub;

    void ehmi_judgment_callback(const std_msgs::String::ConstPtr& msg);
    void add_information_contents(const std_msgs::String& judgment, cv::Mat& information_contents);
};
