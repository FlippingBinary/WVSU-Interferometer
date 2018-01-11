#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <cstdlib>
using namespace std;
//using namespace cv;
void on_low_r_thresh_trackbar(int, void *);
void on_high_r_thresh_trackbar(int, void *);
void on_low_g_thresh_trackbar(int, void *);
void on_high_g_thresh_trackbar(int, void *);
void on_low_b_thresh_trackbar(int, void *);
void on_high_b_thresh_trackbar(int, void *);
int low_r=155, low_g=0, low_b=0;
int high_r=255, high_g=70, high_b=70;
vector<vector<cv::Point>> contours;
vector<cv::Vec4i> hierarchy;
int main()
{
    cv::Mat frame, frame_threshold;
    char in;
    /*
    ** Interferometer.mp4 must be a video of interferometer fringes in the CWD.
    ** This is a hard coded filename for now because it is only for testing purposes.
    */
    cv::VideoCapture cap("Interferometer.mp4");
    cv::namedWindow("Video Capture", cv::WINDOW_NORMAL);
    cv::namedWindow("Object Detection", cv::WINDOW_NORMAL);
    // Trackbars to set thresholds for RGB values
    cv::createTrackbar("Low R","Object Detection", &low_r, 255, on_low_r_thresh_trackbar);
    cv::createTrackbar("High R","Object Detection", &high_r, 255, on_high_r_thresh_trackbar);
    cv::createTrackbar("Low G","Object Detection", &low_g, 255, on_low_g_thresh_trackbar);
    cv::createTrackbar("High G","Object Detection", &high_g, 255, on_high_g_thresh_trackbar);
    cv::createTrackbar("Low B","Object Detection", &low_b, 255, on_low_b_thresh_trackbar);
    cv::createTrackbar("High B","Object Detection", &high_b, 255, on_high_b_thresh_trackbar);
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
            // Now that we have a binary image (frame_threshold) we can find circles.
            // The function below finds Contours, which are oddly-shaped objects and not yet circles.
            findContours(frame_threshold, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
            /*
            ** TODO: Replace the code below with code that detects the minEnclosingCircle of the
            ** detected contour areas. Then the deepest set of nested circles should be considered
            ** the target of the interferometer. These nested circles will need their center points to be
            ** averaged together to find the center-most point of the interferometer fringes. Then the center
            ** can be evaluated to see if it is a fringe or not. Something else to consider: did the fringes
            ** move in or out?
            */
            /* example from Python:
            for cnt in contours:
                if cv2.contourArea(cnt)>min_cnt_area:
                    (x,y),radius = cv2.minEnclosingCircle(cnt)
                    center = (int(x),int(y))
                    radius = int(radius)
                    cv2.circle(img,center,radius,(255,0,0),1)
            */
            int idx = 0;
            for( ; idx >= 0; idx = hierarchy[idx][0] )
            {
                cv::Scalar color( rand()&255, rand()&255, rand()&255 );
                drawContours( frame, contours, idx, color, cv::FILLED, 8, hierarchy );
            }
            // Show the frames. The "Object Detection" window is warped, but can be resized.
            imshow("Video Capture",frame);
            imshow("Object Detection",frame_threshold);
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
    cv::setTrackbarPos("Low R","Object Detection", low_r);
}
void on_high_r_thresh_trackbar(int, void *)
{
    high_r = max(high_r, low_r+1);
    cv::setTrackbarPos("High R", "Object Detection", high_r);
}
void on_low_g_thresh_trackbar(int, void *)
{
    low_g = min(high_g-1, low_g);
    cv::setTrackbarPos("Low G","Object Detection", low_g);
}
void on_high_g_thresh_trackbar(int, void *)
{
    high_g = max(high_g, low_g+1);
    cv::setTrackbarPos("High G", "Object Detection", high_g);
}
void on_low_b_thresh_trackbar(int, void *)
{
    low_b= min(high_b-1, low_b);
    cv::setTrackbarPos("Low B","Object Detection", low_b);
}
void on_high_b_thresh_trackbar(int, void *)
{
    high_b = max(high_b, low_b+1);
    cv::setTrackbarPos("High B", "Object Detection", high_b);
}
