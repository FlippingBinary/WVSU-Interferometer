#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <cstdlib>
using namespace std;

void on_low_r_thresh_trackbar(int, void *);
void on_high_r_thresh_trackbar(int, void *);
void on_low_g_thresh_trackbar(int, void *);
void on_high_g_thresh_trackbar(int, void *);
void on_low_b_thresh_trackbar(int, void *);
void on_high_b_thresh_trackbar(int, void *);
int low_r=155, low_g=0, low_b=0;
int high_r=255, high_g=70, high_b=70;
int main()
{
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::Mat frame, frame_threshold, frame_circles;
    char in;
    /*
    ** Interferometer.mp4 must be a video of interferometer fringes in the CWD.
    ** This is a hard coded filename for now because it is only for testing purposes.
    */
    cv::VideoCapture cap("Interferometer.mp4");
    cv::namedWindow("1. Source Video", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("2. Binary Stage", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("3. Circle Stage", cv::WINDOW_AUTOSIZE);
    
    // Trackbars to set thresholds for RGB values
    cv::createTrackbar("Low R","2. Binary Stage", &low_r, 255, on_low_r_thresh_trackbar);
    cv::createTrackbar("High R","2. Binary Stage", &high_r, 255, on_high_r_thresh_trackbar);
    cv::createTrackbar("Low G","2. Binary Stage", &low_g, 255, on_low_g_thresh_trackbar);
    cv::createTrackbar("High G","2. Binary Stage", &high_g, 255, on_high_g_thresh_trackbar);
    cv::createTrackbar("Low B","2. Binary Stage", &low_b, 255, on_low_b_thresh_trackbar);
    cv::createTrackbar("High B","2. Binary Stage", &high_b, 255, on_high_b_thresh_trackbar);
    while((in=(char)cv::waitKey(1))!='q'){
        /*
        ** This snippet below just causes the sample video to loop
        */
        if(cap.get(cv::CAP_PROP_POS_FRAMES)==cap.get(cv::CAP_PROP_FRAME_COUNT)) {
            cap.set(cv::CAP_PROP_POS_FRAMES,0);
        }
        /*
        ** This conditional below causes the video to only progress one frame each time
        ** the user presses the 'n' key. This makes it easier to see groups when random colors
        ** are used below. Otherwise, the colors flash so fast you can't tell what's going on.
        */
        if ( in == 'n' ) {
            // Bring in the next frame.
            cap>>frame;

            // Break out of the loop if the frame is empty since it can't be processed.
            if(frame.empty())
                break;
            
            // Detect the object based on RGB Range Values
            // The defaults I set above seem to work well, but the sliders allow the developer
            // to adjust those values for a particular video. These values are very important
            // to get right.
            inRange(frame,cv::Scalar(low_b,low_g,low_r), cv::Scalar(high_b,high_g,high_r),frame_threshold);

            // Now that we have a binary image (frame_threshold) we can find contours.
            findContours(frame_threshold, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

            // Approximate contours to polygons + get bounding circles
            vector<vector<cv::Point> > contours_poly( contours.size() );
            vector<cv::Point2f>center( contours.size() );
            vector<float>radius( contours.size() );

            // Start with a clean slate for displaying circles
            frame_circles = cv::Mat::zeros( frame.size(), CV_8UC3 );
            for( int i = 0; i < contours.size(); i++ )
            {
                approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
                cv::minEnclosingCircle( (cv::Mat)contours_poly[i], center[i], radius[i] );
                cv::Scalar color( rand()&255, rand()&255, rand()&255 );
                cv::drawContours( frame_circles, contours_poly, i, color, 1, 8, vector<cv::Vec4i>(), 0, cv::Point() );
                cv::circle( frame_circles, center[i], (int)radius[i], color, 2, 8, 0 );
            }

            // TODO: determine the hierarchy of how circles are nested
            // TODO: select the deepest nesting of circles as the likely target choice
            // TODO: average the center points of those circles
            // TODO: evaluate the color of the pixel at the average center point
            // TODO: determine if the fringes moved in or out

            // Display all detection images (this will not be in production version)
            imshow("1. Source Video",frame);
            imshow("2. Binary Stage",frame_threshold);
            imshow("3. Circle Stage",frame_circles);
        }
    }
    return 0;
}

/*
** These implement the trackbars and will not be in the final code.
*/
void on_low_r_thresh_trackbar(int, void *)
{
    low_r = min(high_r-1, low_r);
    cv::setTrackbarPos("Low R","2. Binary Stage", low_r);
}
void on_high_r_thresh_trackbar(int, void *)
{
    high_r = max(high_r, low_r+1);
    cv::setTrackbarPos("High R", "2. Binary Stage", high_r);
}
void on_low_g_thresh_trackbar(int, void *)
{
    low_g = min(high_g-1, low_g);
    cv::setTrackbarPos("Low G","2. Binary Stage", low_g);
}
void on_high_g_thresh_trackbar(int, void *)
{
    high_g = max(high_g, low_g+1);
    cv::setTrackbarPos("High G", "2. Binary Stage", high_g);
}
void on_low_b_thresh_trackbar(int, void *)
{
    low_b= min(high_b-1, low_b);
    cv::setTrackbarPos("Low B","2. Binary Stage", low_b);
}
void on_high_b_thresh_trackbar(int, void *)
{
    high_b = max(high_b, low_b+1);
    cv::setTrackbarPos("High B", "2. Binary Stage", high_b);
}
