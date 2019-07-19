#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <unistd.h>
#include <geometry_msgs/Twist.h>
#include "ardrone_autonomy/Navdata.h"

// for img processing z
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Image.h"

// for Kalman filter
#include <opencv/cv.h>
#include "opencv2/video/tracking.hpp"
#include "opencv2/core/core.hpp"

#define tresholdH 65

using namespace ros;
using namespace std;
using namespace cv;

static const char WINDOW[] = "Image Processed corridor";

// 	//this is a demo, so we can afford globals
geometry_msgs::Twist command;
cv::Point position1, position2;
image_transport::Publisher pub;

// -------------Kalman estimation creation--------------------
KalmanFilter KF(4, 2, 0, CV_32F); //=CreateKalman(4, 2, 0);
Mat_<float> state(4, 1);		  /* (x, y, Vx, Vy) */
Mat processNoise(4, 1, CV_32F);
Mat_<float> measurmentVP(2, 1);
vector<Point> kalmanVec;
int nrNotDetect = 0;
Point observation, estimation;
ros::Time frameAt;
double lastTimeStamp = 0.0;

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
	// 4x4 for the constant velocity model
	KF.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);

	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
	// proces noise setup, faster response
	// can be finetuned these values for the actual config
	KF.processNoiseCov.at<float>(10) = pow(10, -3);
	KF.processNoiseCov.at<float>(15) = pow(10, -3);
	KF.processNoiseCov.at<float>(0) = pow(10, -2.8);
	KF.processNoiseCov.at<float>(5) = pow(10, -2.8);
	// measurment noise setup - look for the actual values from the experiments
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(KF.errorCovPost, Scalar::all(.1));
	kalmanVec.clear();
}

Point estimationVP(Point at)
{
	//estimation
	Mat prediction = KF.predict();
	Point predictPt(prediction.at<float>(0), prediction.at<float>(1));
	measurmentVP(0) = at.x;
	measurmentVP(1) = at.y;
	// measurement += KF.measurementMatrix*state;
	Mat estimated = KF.correct(measurmentVP);
	Point statePt(estimated.at<float>(0), estimated.at<float>(1));
	kalmanVec.push_back(statePt);
	return statePt;
}
// ------------END-Kalman estimation--------------------

// function processing the naviagtional data from divers sensors and observations. TAG detection included.
void navCallback(const ardrone_autonomy::Navdata::ConstPtr &usm)
{
	double deltT = (usm->header.stamp.toSec() - lastTimeStamp) * 10;
	// Check if any tag was detected
	if (usm->tags_count == 1)
	{
		// scale it down and take the midle part
		position1.x = usm->tags_xc[0] * 64 / 120;
		position1.y = usm->tags_yc[0] * 36 / 120;
		position2.x = position1.x + usm->tags_width[0] * 64 / 120;  // -360/2;
		position2.y = position1.y + usm->tags_height[0] * 36 / 120; // -640/2;

		estimation = estimationVP(Point(position1.x, position1.y));
	}
	else
	{
		// no tag detected, go for blind prediction
		position1.x = -1;
		position1.y = -1;
		position2.x = -1;
		position2.y = -1;
		Mat prediction = KF.predict();
		estimation = Point(prediction.at<float>(0), prediction.at<float>(1));
	}
	lastTimeStamp = usm->header.stamp.toSec();
}

void whereToFly(Point p, Point ref, bool stop)
{

	if (!stop)
	{
		if (p.x > ref.x + 5)
			printf("right\n");
		else if (p.x < ref.x - 5)
			printf("left\n");
		printf("ahead:0.02\n");
	}
	else
	{
		printf("land");
	}
}

Mat &corridorProces(Mat &result)
{

	Mat src = result.clone();
	cvtColor(src, src, CV_RGB2GRAY);
	Canny(src, src, 50, 150, 3);
	// Mat vertical = src.clone();
	// int vert_size = vertical.rows / 60;
	// Mat verticalStructure = getStructuringElement(MORPH_RECT, Size(1, vert_size));
	// erode(vertical, vertical, verticalStructure, Point(-1, -1));
	// dilate(vertical, vertical, verticalStructure, Point(-1, -1), 3);
	// src = src - vertical;
	Mat horizontal = src.clone();
	int horiz_size = horizontal.rows / 60;
	Mat horizontalStructure = getStructuringElement(MORPH_RECT, Size(1, horiz_size));
	erode(horizontal, horizontal, horizontalStructure, Point(-1, -1));
	dilate(horizontal, horizontal, horizontalStructure, Point(-1, -1), 3);
	src = src - horizontal;

	imshow("Canny", src);
	vector<Vec4i> p_lines;
	HoughLinesP(src, p_lines, 1, CV_PI / 180, tresholdH, 45, 15); // indoor setup last two var: 45,15);
	Point VP;
	// The center of the image
	line(result, Point(src.cols / 2 - 15, src.rows / 2), Point(src.cols / 2 + 15, src.rows / 2), Scalar(255, 255, 255), 2);
	line(result, Point(src.cols / 2, src.rows / 2 - 15), Point(src.cols / 2, src.rows / 2 + 15), Scalar(255, 255, 255), 2);

	if (nrNotDetect < 20)
	{

		for (int i = 0; i < p_lines.size();)
		{
			Vec4i l = p_lines[i];
			float the = atan2(l[1] - l[3], l[2] - l[0]) * 180 / CV_PI;
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

// image processing and visualisation function
void imageCallback(const sensor_msgs::ImageConstPtr &msg_ptr)
{

	cv_bridge::CvImagePtr cv_image;
	try
	{
		cv_image = cv_bridge::toCvCopy(msg_ptr, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Image processing section
	frameAt = msg_ptr->header.stamp;
	// Row col 320 640 of the image
	cv::Mat src = corridorProces(cv_image->image);

	cv::imshow(WINDOW, src);
	// Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
	cv::waitKey(3);

	// // Scalar red = CV_RGB(220, 0, 0);
	// // //   cv::rectangle(cv_image->image,estimation, Point(estimation.x+70,estimation.y+25),Scalar(220,0,0),3);
	// // //   cv::rectangle(cv_image->image,position1, position2,red,3);
	// // cv::circle(cv_image->image, position1, 3, Scalar(220, 0, 0), -1, 8, 0);
	// // cv::circle(cv_image->image, position2, 3, Scalar(120, 0, 0), -1, 8, 0);

	// // cv::imshow("Image window", cv_image->image);
	// // cv::waitKey(3);
}

int main(int argc, char **argv)
{
	// Initialize the node
	init(argc, argv, "corridor_proc");
	// Initialize the Kalman filter
	initKF();
	// Create a node handle
	NodeHandle node;

	// A publisher for the movement data
	Publisher pub2 = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	image_transport::ImageTransport it(node);
	image_transport::Subscriber sub = it.subscribe("/ardrone/image_raw", 1, imageCallback);
	// Subscriber sub2 = node.subscribe("/ardrone/navdata", 10, navCallback);

	// cv::destroyWindow(WINDOW);
	pub = it.advertise("/ardrone/image_processed_corridor", 1);
	// Loop at 10Hz, publishing movement commands until we shut down.
	// int i = 0;
	Rate rate(10);
	while (ros::ok())
	{ // while key pressed
		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
