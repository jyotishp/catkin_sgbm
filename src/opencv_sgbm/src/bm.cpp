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
// ros::Publisher disparity_pub;
Mat disparity, depth, depth_image, disparity_image;


string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

void onClick( int event, int x, int y, int, void* )
{
	if( event != CV_EVENT_LBUTTONDOWN )
		return;

	Point pt = Point(x,y);
	std::cout << "Depth at (" << pt.x << "," << pt.y << ") : " << right_info.P[3] * 16 / disparity.at<short>(pt.y, pt.x) << std::endl;
	// std::cout << "Depth at (" << pt.x << "," << pt.y << ") : " << (float)depth.at<Vec3f>(pt.y, pt.x)[2] << std::endl;
	std::cout << "Disparity at (" << pt.x << "," << pt.y << ") : " << disparity.at<short>(pt.y, pt.x) / 16 << std::endl;
	// std::cout << type2str(disparity.type()) << std::endl;
	// std::cout << disparity.cols << "x" << disparity.rows << std::endl;
}

void callback(const ImageConstPtr& image_left, const ImageConstPtr& image_right)
{
	const std::string left_info_path = "data/left_camera.yml";
	const std::string right_info_path = "data/right_camera.yml";

	Mat left_image_rect, right_image_rect, left_image_gray, right_image_gray;
	CameraInfo left_info, right_info;
	std::string left_camera_name, right_camera_name;
	cv_bridge::CvImageConstPtr left_image, right_image;

	bool left = camera_calibration_parsers::readCalibration(left_info_path, left_camera_name, left_info);
	bool right = camera_calibration_parsers::readCalibration(right_info_path, right_camera_name, right_info);

	left_image = cv_bridge::toCvCopy(image_left, image_encodings::BGR8);
	right_image = cv_bridge::toCvCopy(image_right, image_encodings::BGR8);

	Mat K1 = Mat(3, 3, CV_32FC1);
	Mat K2 = Mat(3, 3, CV_32FC1);
	Mat D1 = Mat(5, 1, CV_32FC1);
	Mat D2 = Mat(5, 1, CV_32FC1);
	Mat R1 = Mat(3, 3, CV_32FC1);
	Mat R2 = Mat(3, 3, CV_32FC1);
	Mat P1 = Mat(4, 3, CV_32FC1);
	Mat P2 = Mat(4, 3, CV_32FC1);
	Mat R = Mat(3,3, CV_32FC1);

	K1.at<float>(0,0) = left_info.K[0];
	K1.at<float>(0,1) = left_info.K[1];
	K1.at<float>(0,2) = left_info.K[2];
	K1.at<float>(1,0) = left_info.K[3];
	K1.at<float>(1,1) = left_info.K[4];
	K1.at<float>(1,2) = left_info.K[5];
	K1.at<float>(2,0) = left_info.K[6];
	K1.at<float>(2,1) = left_info.K[7];
	K1.at<float>(2,2) = left_info.K[8];

	K2.at<float>(0,0) = right_info.K[0];
	K2.at<float>(0,1) = right_info.K[1];
	K2.at<float>(0,2) = right_info.K[2];
	K2.at<float>(1,0) = right_info.K[3];
	K2.at<float>(1,1) = right_info.K[4];
	K2.at<float>(1,2) = right_info.K[5];
	K2.at<float>(2,0) = right_info.K[6];
	K2.at<float>(2,1) = right_info.K[7];
	K2.at<float>(2,2) = right_info.K[8];

	D1.at<float>(0,0) = left_info.D[0];
	D1.at<float>(1,0) = left_info.D[1];
	D1.at<float>(2,0) = left_info.D[2];
	D1.at<float>(3,0) = left_info.D[3];
	D1.at<float>(4,0) = left_info.D[4];

	D2.at<float>(0,0) = right_info.D[0];
	D2.at<float>(1,0) = right_info.D[1];
	D2.at<float>(2,0) = right_info.D[2];
	D2.at<float>(3,0) = right_info.D[3];
	D2.at<float>(4,0) = right_info.D[4];

	R1.at<float>(0,0) = left_info.R[0];
	R1.at<float>(0,1) = left_info.R[1];
	R1.at<float>(0,2) = left_info.R[2];
	R1.at<float>(1,0) = left_info.R[3];
	R1.at<float>(1,1) = left_info.R[4];
	R1.at<float>(1,2) = left_info.R[5];
	R1.at<float>(2,0) = left_info.R[6];
	R1.at<float>(2,1) = left_info.R[7];
	R1.at<float>(2,2) = left_info.R[8];

	R2.at<float>(0,0) = right_info.R[0];
	R2.at<float>(0,1) = right_info.R[1];
	R2.at<float>(0,2) = right_info.R[2];
	R2.at<float>(1,0) = right_info.R[3];
	R2.at<float>(1,1) = right_info.R[4];
	R2.at<float>(1,2) = right_info.R[5];
	R2.at<float>(2,0) = right_info.R[6];
	R2.at<float>(2,1) = right_info.R[7];
	R2.at<float>(2,2) = right_info.R[8];

	P1.at<float>(0,0) = left_info.P[0];
	P1.at<float>(0,1) = left_info.P[1];
	P1.at<float>(0,2) = left_info.P[2];
	P1.at<float>(0,3) = left_info.P[3];
	P1.at<float>(1,0) = left_info.P[4];
	P1.at<float>(1,1) = left_info.P[5];
	P1.at<float>(1,2) = left_info.P[6];
	P1.at<float>(1,3) = left_info.P[7];
	P1.at<float>(2,0) = left_info.P[8];
	P1.at<float>(2,1) = left_info.P[9];
	P1.at<float>(2,2) = left_info.P[10];
	P1.at<float>(2,3) = left_info.P[11];

	P2.at<float>(0,0) = right_info.P[0];
	P2.at<float>(0,1) = right_info.P[1];
	P2.at<float>(0,2) = right_info.P[2];
	P2.at<float>(0,3) = right_info.P[3];
	P2.at<float>(1,0) = right_info.P[4];
	P2.at<float>(1,1) = right_info.P[5];
	P2.at<float>(1,2) = right_info.P[6];
	P2.at<float>(1,3) = right_info.P[7];
	P2.at<float>(2,0) = right_info.P[8];
	P2.at<float>(2,1) = right_info.P[9];
	P2.at<float>(2,2) = right_info.P[10];
	P2.at<float>(2,3) = right_info.P[11];

	// FileStorage fs;
	// fs.open("data/left_camera.yml", FileStorage::READ);
	// fs["camera_matrix"] >> K1;
	// fs["distortion_coefficients"] >> D1;
	// fs["rectification_matrix"] >> R1;
	// fs["projection_matrix"] >> P1;

	// std::cout << left_info.K[0] << std::endl;

	R.at<float>(0,0) = left_info.P[0];
	R.at<float>(0,1) = left_info.P[1];
	R.at<float>(0,2) = left_info.P[2];
	R.at<float>(1,0) = left_info.P[5];
	R.at<float>(1,1) = left_info.P[6];
	R.at<float>(1,2) = left_info.P[7];
	R.at<float>(2,0) = left_info.P[9];
	R.at<float>(2,1) = left_info.P[10];
	R.at<float>(2,2) = left_info.P[11];

	Mat maplx, maply, maprx, mapry;

	// initUndistortRectifyMap(K1, D1, R1, P1, left_image->image.size(), CV_32FC1, maplx, maply);
	// initUndistortRectifyMap(K2, D2, R2, P2, right_image->image.size(), CV_32FC1, maprx, mapry);

	// remap(left_image->image, left_image_rect, maplx, maply, INTER_LINEAR, BORDER_CONSTANT, Scalar());
	// remap(right_image->image, right_image_rect, maprx, mapry, INTER_LINEAR, BORDER_CONSTANT, Scalar());

	undistort(left_image->image, left_image_rect, intrinsic_left, distortion_left);
	undistort(right_image->image, right_image_rect, intrinsic_right, distortion_right);

	cvtColor(left_image_rect, left_image_gray, CV_BGR2GRAY);
	cvtColor(right_image_rect, right_image_gray, CV_BGR2GRAY);

	StereoBM sbm;
	sbm.state->SADWindowSize = 21;
	sbm.state->numberOfDisparities = 128;
	sbm.state->preFilterSize = 5;
	sbm.state->preFilterCap = 31;
	sbm.state->minDisparity = 0;
	sbm.state->textureThreshold = 0;
	sbm.state->uniquenessRatio = 5;
	sbm.state->speckleWindowSize = 100;
	sbm.state->speckleRange = 4;
	sbm.state->disp12MaxDiff = 0;
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

	// // cvTranspose(&depth, &matrixTr);
	// reprojectImageTo3D(disparity, depth, Q, (int)false);
	// normalize(depth, depth_image, 0, 255, CV_MINMAX, CV_8U);

	// // std::cout << type2str(depth.type()) << std::endl;
	// // std::cout << "Disparity: " << (int)disparity.at<int>(374,140) << std::endl;
	// // std::cout << disparity.depth() << std::endl;
	// // std::cout << disparity.size() << std::endl;
	// // std::cout << left_image_gray.size() << std::endl << std::endl;
	// // std::cout << "Depth: " << 9.5302534383415178e+01 / disparity.at<short>(375, 142) << std::endl;

	std_msgs::Header header;
	header.stamp = ros::Time::now();

	// stereo_msgs::DisparityImage disparity_msg;
	// disparity_msg.header = header;

	Mat actual_disp = disparity / 16;
	// sensor_msgs::Image& dimage = disparity_msg.image;
	// dimage.width  = (actual_disp).size().width ;
	// dimage.height = (actual_disp).size().height ;
	// dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
	// dimage.step = dimage.width * sizeof(float);
	// dimage.data.resize(dimage.step * dimage.height);
	// cv::Mat_<float> dmat(dimage.height, dimage.width, reinterpret_cast<float*>(&dimage.data[0]), dimage.step);
	// (actual_disp).convertTo(dmat,dmat.type());

	ImagePtr disparity_image_msg = cv_bridge::CvImage(header, "32FC1", actual_disp).toImageMsg();
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
	// disparity_pub = nh.advertise<stereo_msgs::DisparityImage>("sync/disparity", 1);
	// depth_pub = it.advertise("sync/depth", 1);
	disparity_pub = it.advertise("sync/disparity", 1);
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