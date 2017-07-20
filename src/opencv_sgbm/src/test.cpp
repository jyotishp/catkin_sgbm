#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <cv_bridge/cv_bridge.h>

cv::Mat image_disp;

void onClick( int event, int x, int y, int, void* )
{
  if( event != CV_EVENT_LBUTTONDOWN )
    return;

  cv::Point pt = cv::Point(x,y);
  std::cout << "Depth: " << double(image_disp.at<double>(x,y)) << std::endl;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    image_disp = cv_bridge::toCvShare(msg, "32FC1")->image;
    cv::Mat norm_disp;
    cv::normalize(image_disp, norm_disp, 0, 255, CV_MINMAX, CV_8U);
    cv::imshow("view", norm_disp);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  cv::setMouseCallback("view", onClick, 0);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("depth_map/image", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}