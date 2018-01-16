#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <cstdlib>
#include <iostream>
#include <string>
using namespace std;

void on_low_r_thresh_trackbar(int, void *);
void on_high_r_thresh_trackbar(int, void *);
void on_low_g_thresh_trackbar(int, void *);
void on_high_g_thresh_trackbar(int, void *);
void on_low_b_thresh_trackbar(int, void *);
void on_high_b_thresh_trackbar(int, void *);
int low_r = 140, low_g = 0, low_b = 0;
int high_r = 255, high_g = 100, high_b = 100;
int main()
{
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::Mat frame, frame_masked, frame_detect, frame_barcode;
  cv::Mat target, target_greyscale, target_inverted;
  cv::Mat barcode, barcode_mean, barcode_greyscale;
  vector<int> contour_depths;
  int contour_deepest;
  char in;
  // Interferometer.mp4 must be a video of interferometer fringes in the CWD.
  // The filename is hardcoded for testing purposes.
  cv::VideoCapture cap("Interferometer.mp4");
  cv::namedWindow("1. Source Video", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("2. Detect Stage", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("3. Target Stage", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("4. Barcode Stage", cv::WINDOW_AUTOSIZE);

  // Trackbars to set thresholds for RGB values
  cv::createTrackbar("Low R", "2. Detect Stage", &low_r, 255,
                     on_low_r_thresh_trackbar);
  cv::createTrackbar("High R", "2. Detect Stage", &high_r, 255,
                     on_high_r_thresh_trackbar);
  cv::createTrackbar("Low G", "2. Detect Stage", &low_g, 255,
                     on_low_g_thresh_trackbar);
  cv::createTrackbar("High G", "2. Detect Stage", &high_g, 255,
                     on_high_g_thresh_trackbar);
  cv::createTrackbar("Low B", "2. Detect Stage", &low_b, 255,
                     on_low_b_thresh_trackbar);
  cv::createTrackbar("High B", "2. Detect Stage", &high_b, 255,
                     on_high_b_thresh_trackbar);
  while ((in = (char)cv::waitKey(1)) != 'q')
  {
    // Loop sample video
    if (cap.get(cv::CAP_PROP_POS_FRAMES) == cap.get(cv::CAP_PROP_FRAME_COUNT))
    {
      cap.set(cv::CAP_PROP_POS_FRAMES, 0);
    }
    // Advance the video only when user presses 'n' so the effects of
    // processing can be fully evaluated
    if (in == 'n')
    {
      // Bring in the next frame.
      cap >> frame;

      // Exit if the video ended.
      if (frame.empty())
        break;

      // Blur the image to make contour detection go more smoothly
      cv::medianBlur(frame, frame, 5);

      // Detect the object based on RGB Range Values
      // The defaults set above seem to work well, but the sliders
      // allow the developer to adjust those values for a particular
      // video. These values are very important to get right.
      cv::inRange(frame, cv::Scalar(low_b, low_g, low_r),
                  cv::Scalar(high_b, high_g, high_r), frame_masked);

      // Apply adaptive thresholding
      cv::adaptiveThreshold(frame_masked, frame_detect, 255,
                            cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY,
                            11, 2);

      // Find contours in the binary image (frame_detect)
      cv::findContours(frame_detect, contours, hierarchy,
                       cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

      // Convert frame_detect to color so overlayed circles can be colorized
      cv::cvtColor(frame_detect, frame_detect, cv::COLOR_GRAY2BGR);

      // Determine hierarchical depth of each contour
      contour_depths.clear();
      contour_depths.resize(contours.size());
      contour_deepest = 0;
      for (int i = 0; i < contours.size(); i++)
      {
        int j = i;
        while (hierarchy[j][2] >= 0)
        {
          j = hierarchy[j][2];
          contour_depths[i] = contour_depths[i] + 1;
          if (contour_depths[i] > contour_deepest)
            contour_deepest = contour_depths[i];
        }
      }

      // Display depth of deepest contour
      string depth_string = to_string(contour_deepest);
      cv::putText(frame, depth_string,
                  cv::Point(frame.cols / 2, frame.rows / 2),
                  cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));

      // Identify deepest contour(s)
      vector<vector<cv::Point>> contours_poly(contours.size());
      vector<cv::Point2f> center(contours.size());
      vector<float> radius(contours.size());
      cv::Point2f center_average(0.0f, 0.0f);
      int center_count = 0;
      for (int i = 0; i < contours.size(); i++)
      {
        if (contour_depths[i] == contour_deepest)
        {
          int j = i;
          while (j >= 0)
          {
            // Approximate contour to polygons
            approxPolyDP(cv::Mat(contours[j]), contours_poly[j], 3, true);
            // Find smallest circle which contains whole polygon
            cv::minEnclosingCircle((cv::Mat)contours_poly[j], center[j],
                                   radius[j]);
            // Rule out the smaller circles. Gaussian blur might be preferable.
            if (radius[j] > 20)
            {
              cv::circle(frame_detect, center[j], (int)radius[j],
                         cv::Scalar(rand() & 255, rand() & 255, rand() & 255), 2, 8, 0);
              center_average.x += center[j].x;
              center_average.y += center[j].y;
              center_count++;
            }
            j = hierarchy[j][2];
          };
          //          break;
        }
      }

      // Average center points together to find detection point
      center_average.x = (center_average.x / center_count) - 1;
      center_average.y = (center_average.y / center_count) + 1;

      // Clip target area in separate cv::Mat
      target = frame(cv::Rect(center_average.x - 100,
                              center_average.y - 100, 200, 200));

      // Convert target area to greyscale
      cv::cvtColor(target, target_greyscale, cv::COLOR_BGR2GRAY);

      // Clip rectangle from around detection point
      frame_barcode = frame(cv::Rect(center_average.x - 100,
                                     center_average.y - 10, 200, 20));
      frame_barcode.copyTo(barcode);

      // Find barcode vertical mean (convert it to a single pixel row)
      cv::reduce(barcode, barcode_mean, 0, cv::REDUCE_AVG);

      // Vertical stretch it to be more visible
      cv::resize(barcode_mean, barcode, barcode.size(), 0, 0,
                 cv::INTER_NEAREST);

      // Convert barcode to greyscale
      cv::cvtColor(barcode, barcode_greyscale, cv::COLOR_BGR2GRAY);

      // TODO: evaluate the color of the pixel at the average center point
      // TODO: determine if the fringes moved in or out

      // Draw crosshairs over detection point
      cv::drawMarker(frame, center_average,
                     cv::Scalar(rand() & 255, rand() & 255, rand() & 255));
      // Display all detection images (this will not be in production version)
      imshow("1. Source Video", frame);
      imshow("2. Detect Stage", frame_detect);
      imshow("3. Target Stage", target_greyscale);
      imshow("4. Barcode Stage", barcode_greyscale);
    }
  }
  return 0;
}

/*
** These implement the trackbars and will not be in the final code.
*/
void on_low_r_thresh_trackbar(int, void *)
{
  low_r = min(high_r - 1, low_r);
  cv::setTrackbarPos("Low R", "2. Detect Stage", low_r);
}
void on_high_r_thresh_trackbar(int, void *)
{
  high_r = max(high_r, low_r + 1);
  cv::setTrackbarPos("High R", "2. Detect Stage", high_r);
}
void on_low_g_thresh_trackbar(int, void *)
{
  low_g = min(high_g - 1, low_g);
  cv::setTrackbarPos("Low G", "2. Detect Stage", low_g);
}
void on_high_g_thresh_trackbar(int, void *)
{
  high_g = max(high_g, low_g + 1);
  cv::setTrackbarPos("High G", "2. Detect Stage", high_g);
}
void on_low_b_thresh_trackbar(int, void *)
{
  low_b = min(high_b - 1, low_b);
  cv::setTrackbarPos("Low B", "2. Detect Stage", low_b);
}
void on_high_b_thresh_trackbar(int, void *)
{
  high_b = max(high_b, low_b + 1);
  cv::setTrackbarPos("High B", "2. Detect Stage", high_b);
}
