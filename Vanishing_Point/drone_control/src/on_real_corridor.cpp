#include <iostream>
#include <fstream>
// For img processing
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// For Kalman filter
#include <opencv/cv.h>
#include "opencv2/video/tracking.hpp"

// Control masege includeing
#include <time.h>
#include <drone_control/corridorFlyCommand.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>


#define PI 3.1415926
#define tresholdH 65

// Encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;

drone_control::corridorFlyCommand controlMsg;

// Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed corridor";

// Use method of ImageTransport to create image publisher
image_transport::Publisher pub;
// Publisher for fly control
ros::Publisher pubFly;

// -------------Kalman estimation--------------------
KalmanFilter KF(4, 2, 0, CV_32F);
Mat_<float> state(4, 1); /* (x, y, Vx, Vy) */
Mat processNoise(4, 1, CV_32F);
Mat_<float> measurmentVP(2, 1);
vector<Point> kalmanVec;
int nrNotDetect = 0;
Point observation, estimation;
ros::Time frameAt;

// Checking between two lines the crossing point
Point2f *crossPoint(Vec4i l1, Vec4i l2)
{
	Point2f *cP;
	cP = new Point2f;
	Point2f o1(l1[0], l1[1]); // start point line1
	Point2f p1(l1[2], l1[3]); // end point line 1
	Point2f o2(l2[0], l2[1]); // start point line2
	Point2f p2(l2[2], l2[3]); // end point line2
	Point2f x = o2 - o1;
	Point2f d1 = p1 - o1;
	Point2f d2 = p2 - o2;
	float cross = d1.x * d2.y - d1.y * d2.x;
	if (abs(cross) > 1e-8)
	{
		double t1 = (x.x * d2.y - x.y * d2.x) / cross;
		*cP = o1 + d1 * t1;
	}
	else
	{
		Point2f cPt(-1, -1);
		*cP = cPt;
	}
	return cP;
}

void initKF()
{
	measurmentVP.setTo(Scalar(0.0));
	KF.statePre.at<float>(0) = 16;
	KF.statePre.at<float>(1) = 11;
	KF.statePre.at<float>(2) = 0;
	KF.statePre.at<float>(3) = 0;
	KF.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
	// Proces noise setup, faster response
	KF.processNoiseCov.at<float>(10) = pow(10, -3);
	KF.processNoiseCov.at<float>(15) = pow(10, -3);
	KF.processNoiseCov.at<float>(0) = pow(10, -2.8);
	KF.processNoiseCov.at<float>(5) = pow(10, -2.8);
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(KF.errorCovPost, Scalar::all(.1));
	kalmanVec.clear();
}

Point estimationVP(Point at)
{
	// Estimation
	Mat prediction = KF.predict();
	Point predictPt(prediction.at<float>(0), prediction.at<float>(1));
	measurmentVP(0) = at.x;
	measurmentVP(1) = at.y;
	// Measurement
	Mat estimated = KF.correct(measurmentVP);
	Point statePt(estimated.at<float>(0), estimated.at<float>(1));
	kalmanVec.push_back(statePt);
	return statePt;
}

void whereToFly(Point p, Point ref, bool stop)
{
	drone_control::corridorFlyCommand controlMsg;
	// geometry_msgs::Twist controlMsg;
	controlMsg.rotation = "no";  // left right no
	controlMsg.direction = "no"; // left right no
	controlMsg.ahead = "no";	 // front back no land
	controlMsg.speed = 0.0;		 // float in (-1,1)!!
	if (!stop)
	{
		if (p.x > ref.x + 5)
		{
			controlMsg.direction = "right";
			printf("right\n");
		}
		else if (p.x < ref.x - 5)
		{
			controlMsg.direction = "left";
			printf("left\n");
		}
		controlMsg.ahead = "front";
		controlMsg.speed = 0.02;
		printf("ahead");
	}
	else
	{
		controlMsg.ahead = "land";
		printf("land");
	}
	controlMsg.observationX = observation.x;
	controlMsg.observationY = observation.y;
	controlMsg.estimationX = estimation.x;
	controlMsg.estimationY = estimation.x;
	controlMsg.frameTimeStamp = frameAt;
	pubFly.publish(controlMsg);

	// // // std_msgs::Empty emp_msg;
	// // // ros::NodeHandle node;
	// // // ros::Publisher pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	// // // ros::Publisher pub_land = node.advertise<std_msgs::Empty>("ardrone/land",10);
	// // // geometry_msgs::Twist comm;
	// // // comm.linear.x = 0.05;
	// // // comm.linear.y = 1.0;
	// // // comm.linear.z = 0.0;
	// // // comm.angular.x = 0.0;
	// // // comm.angular.y = 0.0;
	// // // comm.angular.z = 0.0;

	// // // ros::Rate rate(5);
	// // // geometry_msgs::Twist comma;
	// // // comma.linear.x = 0.05;
	// // // comma.linear.y = -1.0;
	// // // comma.linear.z = 0.0;
	// // // comma.angular.x = 0.0;
	// // // comma.angular.y = 0.0;
	// // // comma.angular.z = 0.0;


	// // // if(!stop)
	// // // {
	// // // 	if (p.x > ref.x + 5)
	// // // 	{
	// // // 		printf("Right\n");
	// // // 		printf("%d",p.x);
	// // // 		printf("------------");
	// // // 		printf("%d\n",ref.x);
	// // // 		pub.publish(comma);
	// // // 		rate.sleep();
	// // // 	}
	// // // 	else if (p.x < ref.x - 5)
	// // // 	{
	// // // 		printf("Left\n");
	// // // 		printf("%d",p.x);
	// // // 		printf("------------");
	// // // 		printf("%d\n",ref.x);
	// // // 		pub.publish(comm);
	// // // 		rate.sleep();
	// // // 	}
	// // // 	// comm.linear.x = 0.1;
	// // // 	// comm.linear.y = 0.0;
	// // // 	// pub.publish(comm);
	// // // 	// rate.sleep();
	// // // }else
	// // // {
	// // // 	// comm.linear.x=0.0;
	// // // 	// comm.linear.y=0.0;
	// // // 	rate.sleep();
	// // // 	pub_land.publish(emp_msg);
	// // // }
	// // // // pub.publish(comm);
    
}

// Serach for the corridor end/vanishing point
Mat &corridorProces(Mat &result)
{

	Mat src = result.clone();
	cvtColor(src, src, CV_RGB2GRAY);
	Canny(src, src, 50, 200, 3);
	imshow("Canny",src);
	vector<Vec4i> p_lines;
	HoughLinesP(src, p_lines, 1, CV_PI / 180, tresholdH, 85, 15); // indoor setup last two var: 45,15);
	Point VP;
	// The center of the image
	line(result, Point(src.cols / 2 - 15, src.rows / 2), Point(src.cols / 2 + 15, src.rows / 2), Scalar(255, 255, 255), 2);
	line(result, Point(src.cols / 2, src.rows / 2 - 15), Point(src.cols / 2, src.rows / 2 + 15), Scalar(255, 255, 255), 2);

	if (nrNotDetect < 20)
	{

		for (int i = 0; i < p_lines.size();)
		{
			Vec4i l = p_lines[i];
			float the = atan2(l[1] - l[3], l[2] - l[0]) * 180 / PI;
			if ((the > -5 && the < 5) || (abs(the) < 100 && abs(the) > 80) || (abs(the) < 185 && abs(the) > 175) || (abs(the) < 280 && abs(the) > 250))
			{
				p_lines.erase(p_lines.begin() + i);
			}
			else
			{
				i++;
				line(result, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 1);
			}
		}
		int sizeY = src.rows / 23, sizeX = src.cols / 33;
		if (p_lines.size() > 1 && sizeX > 0 && sizeY > 0)
		{
			int imgGrid[33][23];		 // Grid of crossing points
			for (int i = 0; i < 33; i++) // Initialization
			{
				for (int j = 0; j < 23; j++)
					imgGrid[i][j] = 0;
			}
			Point2f *cP;

			if (p_lines.size() == 2)
			{
				cP = crossPoint(p_lines[0], p_lines[1]);
				if (cP->x > 0.0 && cP->y > 0.0 && cP->x < sizeX * 33.0 && cP->y < sizeY * 23.0)
				{
					imgGrid[int(cP->x / sizeX)][int(cP->y / sizeY)]++;
				}
			}
			else
			{
				for (int i = 0; i < p_lines.size() - 1; i++)
				{
					for (int j = i + 1; j < p_lines.size(); j++)
					{
						cP = crossPoint(p_lines[i], p_lines[j]);

						if (cP->x > 0.0 && cP->y > 0.0 && cP->x < sizeX * 33.0 && cP->y < sizeY * 23.0)
						{
							imgGrid[int(cP->x / sizeX)][int(cP->y / sizeY)]++;
						}
					}
				}
			}
			delete cP;
			int countP = 0, maxi = 0, maxj = 0;
			for (int i = 0; i < 33; i++)
			{
				for (int j = 0; j < 23; j++)
				{
					if (countP < imgGrid[i][j])
					{
						countP = imgGrid[i][j];
						maxi = i;
						maxj = j;
					}
				}
			}
			if (countP > 0)
			{
				Point vanishP1 = cvPoint(maxi * sizeX, maxj * sizeY), vanishP2 = cvPoint((maxi + 1) * sizeX, (maxj + 1) * sizeY);
				Scalar red = CV_RGB(220, 0, 0);
				rectangle(result, vanishP1, vanishP2, red, 2, 8, 0);
				red = CV_RGB(20, 120, 220);

				Point estimatedP = estimationVP(vanishP1), eP2 = Point(estimatedP.x + sizeX, estimatedP.y + sizeY);
				rectangle(result, estimatedP, eP2, red, 2, 8, 0);
				VP = estimatedP;
				// For data logging
				observation = vanishP1;
				estimation = estimatedP;
				nrNotDetect = 0;
			}
			else
			{
				Scalar red = CV_RGB(120, 0, 120);
				Mat prediction = KF.predict();
				Point predictPt(prediction.at<float>(0), prediction.at<float>(1));
				rectangle(result, predictPt, predictPt + Point(sizeX, sizeY), red, 2, 8, 0);
				nrNotDetect++;
				VP = predictPt;
				// For data logging
				observation = Point(-1, -1);
				estimation = VP;
			}
		}
		else
		{
			Scalar red = CV_RGB(120, 0, 120);
			Mat prediction = KF.predict();
			Point predictPt(prediction.at<float>(0), prediction.at<float>(1));
			rectangle(result, predictPt, predictPt + Point(sizeX, sizeY), red, 2, 8, 0);
			VP = predictPt;
			nrNotDetect++;
			// For data logging
			observation = Point(-1, -1);
			estimation = VP;
		}
		whereToFly(VP, Point(sizeX * 16, sizeY * 11), false);
	}
	else
	{
		whereToFly(Point(0, 0), Point(0, 0), true);
	}
	return result;
}

//---------------------------------------------------------------------------------------
// Function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr &original_image)
{
	// Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		// Always copy, returning a mutable CvImage. OpenCV expects color images to use BGR channel order.
		cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
	}
	catch (cv_bridge::Exception &e)
	{
		// If there is an error during conversion, display it
		ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
		return;
	}
	// Image processing section
	frameAt = original_image->header.stamp;
	// Row col 320 640 of the image
	cv::Mat src = corridorProces(cv_ptr->image);

	// Display the image using OpenCV
	cv::imshow(WINDOW, src);
	// Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
	cv::waitKey(3);
	// Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
	cv_ptr->image = src;
	pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv)
{
	initKF();
	ros::init(argc, argv, "corridor_processor");
	ros::NodeHandle nh;
	// Create an ImageTransport instance, initializing it with our NodeHandle.
	image_transport::ImageTransport it(nh);
	// OpenCV HighGUI call to create a display window on start-up.
	cv::namedWindow(WINDOW, CV_WINDOW_NORMAL);
	image_transport::Subscriber sub = it.subscribe("/ardrone/front/image_rect_color", 1, imageCallback);
	// Flight direction massage publisher
	pubFly = nh.advertise<drone_control::corridorFlyCommand>("/corridor/fly_command", 1);
	// pubFly = nh.advertise<drone_control::corridorFlyCommand>("cmd_vel", 10);
	// OpenCV HighGUI call to destroy a display window on shut-down.
	cv::destroyWindow(WINDOW);
	pub = it.advertise("/ardrone/image_processed_corridor", 1);
	ros::spin();
	ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
	while (ros::ok())
	{
	}
}
