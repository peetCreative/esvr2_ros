#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Trigger.h>

bool mQuit {false};

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
        int new_cols = 1920.f*((float)img.rows/1080.f);
        cv::Mat back = cv::Mat::zeros(img.rows, new_cols,CV_8UC3);
        cv::Mat tmp = back(cv::Rect((new_cols-img.cols)/2,0,img.cols,img.rows));
        img.copyTo(tmp);
        cv::imshow("view", back);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

bool quit(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &resp)
{
    resp.success = true;
    mQuit = true;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    ros::ServiceServer quitService = nh.advertiseService("display/quit", &quit);
    cv::namedWindow("view", cv::WND_PROP_FULLSCREEN);
    cv::setWindowProperty("view", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    cv::resizeWindow("view",1920,1080);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("image_raw", 1, imageCallback, image_transport::TransportHints("compressed"));
    while (ros::ok() && !mQuit)
        ros::spinOnce();
    cv::destroyWindow("view");
}
