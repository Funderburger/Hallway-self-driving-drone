#include <stdio.h>
#include <opencv2/opencv.hpp>
using namespace cv;


int main(int argc, char** argv )
{
    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }
    Mat image;
    image = imread( argv[1], 1 );
    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }

    Mat gray;
    if (image.channels() == 3)
    {
        cvtColor(image, gray, CV_BGR2GRAY);
    }
    else
    {
        gray = image;
    }

    //show the gray image
    imshow("Gray",gray);

    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", image);
    waitKey(0);
    return 0;
}
