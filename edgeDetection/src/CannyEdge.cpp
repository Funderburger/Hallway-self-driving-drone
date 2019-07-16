#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>

using namespace cv;
using namespace std;

/// Global variables

Mat src, src_gray;
Mat dst, detected_edges, cdst;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
char* window_name = "Edge Map";

/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */
void CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 3x3
  blur( src_gray, detected_edges, Size(3,3) );
  //imshow("Blured image",detected_edges);

  /// Canny detector
  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
  //imshow("Not Last",detected_edges);
  /// Using Canny's output as a mask, we display our result
  dst = Scalar::all(0);

  src.copyTo( dst, detected_edges);
  imshow( window_name, dst );
 }


/** @function main */
int main( int argc, char** argv )
{
  /// Load an image
  src = imread( argv[1] );

  if( !src.data )
  { return -1; }

  /// Create a matrix of the same type and size as src (for dst)
  dst.create( src.size(), src.type() );

  /// Convert the image to grayscale
  cvtColor( src, src_gray, CV_BGR2GRAY );
  //imshow("Gray",src_gray);

  Canny(src, dst, 150, 150, 3);
  /// Create a window
  cvtColor(dst, cdst, CV_GRAY2BGR);

  vector<Vec4i> lines;
  HoughLinesP(dst, lines, 1, CV_PI/180, 65, 50, 10 );
  for( size_t i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
    line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
  }

//    vector<Vec2f> lines;
//   HoughLines(dst, lines, 1, CV_PI/180, 65, 85, 0 );

//   for( size_t i = 0; i < lines.size(); i++ )
//   {
//      float rho = lines[i][0], theta = lines[i][1];
//      Point pt1, pt2;
//      double a = cos(theta), b = sin(theta);
//      double x0 = a*rho, y0 = b*rho;
//      pt1.x = cvRound(x0 + 1000*(-b));
//      pt1.y = cvRound(y0 + 1000*(a));
//      pt2.x = cvRound(x0 - 1000*(-b));
//      pt2.y = cvRound(y0 - 1000*(a));
//      line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
//   }

  imshow("source", src);
  imshow("detected lines", cdst);

  namedWindow( window_name, CV_WINDOW_AUTOSIZE );
  imshow(window_name,dst);
  /// Create a Trackbar for user to enter threshold
  //createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );

  /// Show the image
  //CannyThreshold(0, 0);

  /// Wait until user exit program by pressing a key
  while((cv::waitKey() & 0xEFFFFF) != 27);

  return 0;
  }