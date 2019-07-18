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
int lowThreshold = 70;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
string window_name = "Edge Map";

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

  Canny(src, dst, lowThreshold, lowThreshold*3, 3);
  /// Create a window
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ar trebui micsorata grosimea liniilor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

namedWindow("gray", CV_WINDOW_NORMAL );
  imshow("gray", dst);
  resizeWindow("gray",600,900);
  Mat horizontal = dst.clone();
  int horizontal_size = horizontal.cols / 30;
  Mat horizontalStructure = getStructuringElement(MORPH_RECT, Size(horizontal_size, 1));

namedWindow("eroded", CV_WINDOW_NORMAL );
  imshow("eroded", dst);
  resizeWindow("eroded",600,900);

  erode(horizontal, horizontal, horizontalStructure, Point(-1, -1));
  cvtColor(dst, cdst, CV_GRAY2BGR);

  vector<Vec4i> lines;
  HoughLinesP(dst, lines, 1, CV_PI/180, 90, 300, 80 );
  for( size_t i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
    line( src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
  }

  //  vector<Vec2f> lines;
  // HoughLines(dst, lines, 1, CV_PI/180, 150, 85, 0 );

  // for( size_t i = 0; i < lines.size(); i++ )
  // {
  //    float rho = lines[i][0], theta = lines[i][1];
  //    Point pt1, pt2;
  //    double a = cos(theta), b = sin(theta);
  //    double x0 = a*rho, y0 = b*rho;
  //    pt1.x = cvRound(x0 + 1000*(-b));
  //    pt1.y = cvRound(y0 + 1000*(a));
  //    pt2.x = cvRound(x0 - 1000*(-b));
  //    pt2.y = cvRound(y0 - 1000*(a));
  //    line( src, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
  // }

  namedWindow("source", CV_WINDOW_NORMAL );
  imshow("source", src);
  resizeWindow("source",600,900);

  // namedWindow("detected_lines", CV_WINDOW_NORMAL );
  // imshow("detected_lines", cdst);
  // resizeWindow("detected_lines",600,900);
  
  
  namedWindow( window_name, CV_WINDOW_NORMAL );
  imshow(window_name,dst);
  resizeWindow(window_name,600,900);

  /// Wait until user exit program by pressing a key
  while((cv::waitKey() & 0xEFFFFF) != 27);

  return 0;
  }