#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>

using namespace cv;
using namespace std;

/// Global variables

Mat src, src_gray;
Mat black_white, detected_edges, cblack_white;

int edgeThresh = 1;
int lowThreshold = 50; //70
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
string window_name = "Edge Map";

/** @function main */
int main(int argc, char **argv)
{
    CvCapture *video = cvCaptureFromFile(argv[1]);

    if (!video)
    {
        printf("Fail!!! Probably error 404!");
        return -1;
    }

    int fps = (int)cvGetCaptureProperty(video, CV_CAP_PROP_FPS);
    printf(" FPS: %d\n", fps);

    IplImage *frame = NULL;
    int frame_number = 0;
    char key = 0;

    while (key != 'q')
    {

        //get frame
        frame = cvQueryFrame(video);
        if(!frame)
        {
            printf("No frame!");
            break;
        }

        src = cvarrToMat(frame);

        /// Create a matrix of the same type and size as src (for black_white)
        black_white.create(src.size(), src.type());

        /// Convert the image to grayscale
        cvtColor(src, src_gray, CV_BGR2GRAY);
        //imshow("Gray",src_gray);

        Canny(src, black_white, lowThreshold, lowThreshold * 4, 3);
        /// Create a window

        Mat vertical = black_white.clone();
        int vert_size = vertical.rows/30;
        Mat verticalStructure = getStructuringElement(MORPH_RECT, Size(1, vert_size));
        erode(vertical, vertical, verticalStructure, Point(-1, -1));
        
        cvtColor(black_white, cblack_white, CV_GRAY2BGR);

        vector<Vec4i> lines;
        HoughLinesP(black_white, lines, 1, CV_PI / 180, 65, 85, 15); //90,300,80
        for (size_t i = 0; i < lines.size(); i++)
        {
            Vec4i l = lines[i];
            line(src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, CV_AA);
        }

        //    vector<Vec2f> lines;
        //   HoughLines(black_white, lines, 1, CV_PI/180, 65, 85, 0 );

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
        //      line( cblack_white, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
        //   }

        namedWindow("source", CV_WINDOW_NORMAL);
        imshow("source", src);
        resizeWindow("source", 600, 900);

        namedWindow("detected_lines", CV_WINDOW_NORMAL);
        imshow("detected_lines", cblack_white);
        resizeWindow("detected_lines", 600, 900);

        namedWindow(window_name, CV_WINDOW_NORMAL);

        imshow(window_name, black_white);
        resizeWindow(window_name, 600, 900);
        
        namedWindow("verticals", CV_WINDOW_NORMAL);
        imshow("verticals", vertical);
        resizeWindow("vertical", 600, 900);
        
        key = cvWaitKey(300/fps);

    }

    /// Wait until user exit program by pressing a key
    // while ((cv::waitKey() & 0xEFFFFF) != 27)
    //     ;

    cvReleaseCapture(&video);
    
    return 0;
}