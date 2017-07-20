#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
// #include <stereo_msgs/DisparityImage.h>
#include <camera_calibration_parsers/parse.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;

image_transport::Publisher disparity_pub, image_left_pub, image_right_pub, depth_pub;

void onClick( int event, int x, int y, int, void* )
{
	if( event != CV_EVENT_LBUTTONDOWN )
		return;

	Point pt = Point(x,y);
	std::cout << "x = " << pt.x << "\t y = " << pt.y << std::endl;
}

void callback(const ImageConstPtr& image_left, const ImageConstPtr& image_right)
{
	const std::string left_info_path = "data/left_camera.yml";
	const std::string right_info_path = "data/right_camera.yml";

	Mat left_image_rect, right_image_rect, left_image_gray, right_image_gray, disparity, disparity_image;
	CameraInfo left_info, right_info;
	std::string left_camera_name, right_camera_name;
	cv_bridge::CvImageConstPtr left_image, right_image;

	bool left = camera_calibration_parsers::readCalibration(left_info_path, left_camera_name, left_info);
	bool right = camera_calibration_parsers::readCalibration(right_info_path, right_camera_name, right_info);

	left_image = cv_bridge::toCvCopy(image_left, image_encodings::BGR8);
	right_image = cv_bridge::toCvCopy(image_right, image_encodings::BGR8);

	Mat intrinsic_left = Mat(3, 3, CV_32FC1);
	Mat intrinsic_right = Mat(3, 3, CV_32FC1);
	Mat distortion_left = Mat(5, 1, CV_32FC1);
	Mat distortion_right = Mat(5, 1, CV_32FC1);

	intrinsic_left.at<float>(0,0) = left_info.K[0];
	intrinsic_left.at<float>(0,1) = left_info.K[1];
	intrinsic_left.at<float>(0,2) = left_info.K[2];
	intrinsic_left.at<float>(1,0) = left_info.K[3];
	intrinsic_left.at<float>(1,1) = left_info.K[4];
	intrinsic_left.at<float>(1,2) = left_info.K[5];
	intrinsic_left.at<float>(2,0) = left_info.K[6];
	intrinsic_left.at<float>(2,1) = left_info.K[7];
	intrinsic_left.at<float>(2,2) = left_info.K[8];

	intrinsic_right.at<float>(0,0) = right_info.K[0];
	intrinsic_right.at<float>(0,1) = right_info.K[1];
	intrinsic_right.at<float>(0,2) = right_info.K[2];
	intrinsic_right.at<float>(1,0) = right_info.K[3];
	intrinsic_right.at<float>(1,1) = right_info.K[4];
	intrinsic_right.at<float>(1,2) = right_info.K[5];
	intrinsic_right.at<float>(2,0) = right_info.K[6];
	intrinsic_right.at<float>(2,1) = right_info.K[7];
	intrinsic_right.at<float>(2,2) = right_info.K[8];

	distortion_left.at<float>(0,0) = left_info.D[0];
	distortion_left.at<float>(1,0) = left_info.D[1];
	distortion_left.at<float>(2,0) = left_info.D[2];
	distortion_left.at<float>(3,0) = left_info.D[3];
	distortion_left.at<float>(4,0) = left_info.D[4];

	distortion_right.at<float>(0,0) = right_info.D[0];
	distortion_right.at<float>(1,0) = right_info.D[1];
	distortion_right.at<float>(2,0) = right_info.D[2];
	distortion_right.at<float>(3,0) = right_info.D[3];
	distortion_right.at<float>(4,0) = right_info.D[4];

	undistort(left_image->image, left_image_rect, intrinsic_left, distortion_left);
	undistort(right_image->image, right_image_rect, intrinsic_right, distortion_right);

	cvtColor(left_image_rect, left_image_gray, CV_BGR2GRAY);
	cvtColor(right_image_rect, right_image_gray, CV_BGR2GRAY);

	StereoSGBM sbm;
	sbm.SADWindowSize = 5;
	sbm.numberOfDisparities = 128;
	sbm.preFilterCap = 31;
	sbm.minDisparity = 0;
	sbm.uniquenessRatio = 10;
	sbm.speckleWindowSize = 150;
	sbm.speckleRange = 6;
	sbm.disp12MaxDiff = 0;
	sbm.fullDP = true;
	sbm.P1 = 600; // sbm.SADWindowSize * sbm.SADWindowSize * 8;
	sbm.P2 = 2700; // sbm.SADWindowSize * sbm.SADWindowSize * 32;
	sbm(left_image_gray, right_image_gray, disparity);

	normalize(disparity, disparity_image, 0, 255, CV_MINMAX, CV_8U);

	// Mat Q = Mat(4, 4, CV_32FC1);
	// // stereoRectify(intrinsic_left, intrinsic_right, distortion_left, distortion_right, left_image->image.size(), R, T, R1, R2, P1, P2, Q);
	// Q.at<double>(0,0)=1.0;
	// Q.at<double>(0,1)=0.0;
	// Q.at<double>(0,2)=0.0;
	// Q.at<double>(0,3)=-3.9986589950323105e+02;
	// Q.at<double>(1,0)=0.0;
	// Q.at<double>(1,1)=1.0;
	// Q.at<double>(1,2)=0.0;
	// Q.at<double>(1,3)=-2.6547610736638308e+02;
	// Q.at<double>(2,0)=0.0;
	// Q.at<double>(2,1)=0.0;
	// Q.at<double>(2,2)=0.0;
	// Q.at<double>(2,3)=4.7435756507001634e+02;
	// Q.at<double>(3,0)=0.0;
	// Q.at<double>(3,1)=0.0;
	// Q.at<double>(3,2)=-4.7435756507001634e+02/(9.5302534383415178e+01);
	// Q.at<double>(3,3)=0;

	// Mat depth, depth_image;
	// // cvTranspose(&depth, &matrixTr);
	// reprojectImageTo3D(disparity, depth, Q, (int)false);
	// normalize(depth, depth_image, 0, 255, CV_MINMAX, CV_8U);

	// std::cout << "Disparity: " << disparity.at<double>(148,366) << std::endl;
	// std::cout << "Depth: " << 9.5302534383415178e+01 / disparity.at<double>(132,383) << std::endl;

	std_msgs::Header header;
	header.stamp = ros::Time::now();

	ImagePtr disparity_msg = cv_bridge::CvImage(header, "32FC1", disparity).toImageMsg();
	ImagePtr image_left_msg = cv_bridge::CvImage(header, "bgr8", left_image_rect).toImageMsg();
	ImagePtr image_right_msg = cv_bridge::CvImage(header, "bgr8", right_image_rect).toImageMsg();
	// ImagePtr depth_msg = cv_bridge::CvImage(header, "32FC1", depth).toImageMsg();

	disparity_pub.publish(disparity_msg);
	image_left_pub.publish(image_left_msg);
	image_right_pub.publish(image_right_msg);
	// depth_pub.publish(depth_msg);

	imshow("left_rect_color", left_image_rect);
	imshow("Disparity", disparity_image);
	// imshow("Depth", depth_image);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sgbm");

	ros::NodeHandle nh;
	ros::NodeHandle pub;

	image_transport::ImageTransport it(pub);
	disparity_pub = it.advertise("sync/disparity", 1);
	depth_pub = it.advertise("sync/depth", 1);
	image_left_pub = it.advertise("sync/left/image_rect_color", 1);
	image_right_pub = it.advertise("sync/right/image_rect_color", 1);

	namedWindow("Disparity");
	namedWindow("left_rect_color");
	setMouseCallback("left_rect_color", onClick, 0);
	// namedWindow("Depth");
	startWindowThread();

	message_filters::Subscriber<Image> image_left_sub(nh, "left_rgb/image", 1);
	message_filters::Subscriber<Image> image_right_sub(nh, "right_rgb/image", 1);

	typedef sync_policies::ApproximateTime<Image, Image> stereoSync;
	Synchronizer<stereoSync> sync(stereoSync(10), image_left_sub, image_right_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));

	ros::spin();
	destroyWindow("left_rect_color");
	destroyWindow("Disparity");
	// destroyWindow("Depth");

	return 0;
}