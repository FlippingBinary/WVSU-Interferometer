#include <bitset>
#include <cstdlib>
#include <iostream>
#include <string>
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#ifdef _WIN32
#include "opencv2/highgui.hpp"
#endif
using namespace std;

#ifdef _WIN32
void on_low_r_thresh_trackbar(int, void *);
void on_high_r_thresh_trackbar(int, void *);
void on_low_g_thresh_trackbar(int, void *);
void on_high_g_thresh_trackbar(int, void *);
void on_low_b_thresh_trackbar(int, void *);
void on_high_b_thresh_trackbar(int, void *);
#else
#include <unistd.h>  //Used for UART
#include <fcntl.h>   //Used for UART
#include <termios.h> //Used for UART

int setup_uart()
{
  int uart = -1;
  uart = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY);
  if (uart == -1)
  {
    cout << "FATAL ERROR: UART inaccessible!" << endl;
    exit(EXIT_FAILURE);
  }

  // For reference, see https://en.wikibooks.org/wiki/Serial_Programming/termios
  struct termios options;
  tcgetattr(uart, &options);
  options.c_cflag = B57600 | CS8 | CLOCAL | CREAD; //<Set baud rate
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  tcflush(uart, TCIFLUSH);
  tcsetattr(uart, TCSANOW, &options);
  return uart;
}

void transmit(int uart, string splat)
{
  // TODO: improve code for when write is partial
  if (uart != -1)
  {
    int count = write(uart, splat.c_str(), splat.length());
    if (count < 0)
    {
      printf("UART TX error\n");
    }
  }
}

#endif

int low_r = 140, low_g = 0, low_b = 0;
int high_r = 255, high_g = 100, high_b = 100;

// Prepare timing variable
double timing;
void reset_timing()
{
  timing = (double)cv::getTickCount();
}
// This function prints out the time elapsed since
// the last time it was executed.
void print_timing(string message = "elapsed time")
{
  timing = ((double)cv::getTickCount() - timing) / cv::getTickFrequency();
  cout << "(" << timing << ")[" << message << "]" << endl;
  timing = (double)cv::getTickCount();
}

int main(int argc, char **argv)
{
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::Mat frame, frame_masked, frame_detect, frame_barcode;
  cv::Mat target, target_greyscale, target_inverted;
  cv::Mat barcode, barcode_mean, barcode_greyscale;
  vector<int> contour_depths;
  int contour_deepest;
#ifndef _WIN32
  int uart = setup_uart();
#endif
  reset_timing();
  if (argc > 2)
  {
    // Support is only provided for a single argument, so print usage and exit
    cout << "Usage: " << argv[0] << " [source]" << endl;
    cout << "  source = ffmpeg-compatible video file or camera number" << endl;
    cout << endl;
  }
  cv::VideoCapture cap;
  if (argc > 1)
  {
    // An argument is provided, so try to use that as video source
    if (!cap.open(argv[1]))
    {
      // failed to open as string, so convert to integer and try again
      if (!cap.open(atoi(argv[1])))
      {
        // failed to open as integer, so print error and exit
        cout << "ERROR: Unable to open video source: " << argv[1] << endl;
      };
    }
  }
  else
  {
#ifdef _WIN32
    // Interferometer.mp4 must be a video of interferometer fringes in the CWD.
    cap.open("Interferometer.mp4");
#else
    cap.open(0);
#endif
  }
#ifdef _WIN32
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
#endif
  char in = ' ';
  while (1)
  {
#ifdef _WIN32
    if ((in = (char)cv::waitKey(1)) == 'q')
      break;
#endif
    // Loop sample video
    if (cap.get(cv::CAP_PROP_POS_FRAMES) == cap.get(cv::CAP_PROP_FRAME_COUNT))
      cap.set(cv::CAP_PROP_POS_FRAMES, 0);
    // Advance the video only when user presses 'n' so the effects of
    // processing can be fully evaluated
    if (in != 'p')
    {
      print_timing("Start loop");
      // Bring in the next frame.
      cap >> frame;
      print_timing("cap>>frame");

      // Exit if the video ended.
      if (frame.empty())
        break;

      // Blur the image to make contour detection go more smoothly
      cv::medianBlur(frame, frame, 5);
      print_timing("medianBlur");

      // Detect the object based on RGB Range Values
      // The defaults set above seem to work well, but the sliders
      // allow the developer to adjust those values for a particular
      // video. These values are very important to get right.
      cv::inRange(frame, cv::Scalar(low_b, low_g, low_r),
                  cv::Scalar(high_b, high_g, high_r), frame_masked);
      print_timing("inRange");

      // Apply adaptive thresholding
      cv::adaptiveThreshold(frame_masked, frame_detect, 255,
                            cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY,
                            11, 2);
      print_timing("adaptiveThreshold");

      // Find contours in the binary image (frame_detect)
      cv::findContours(frame_detect, contours, hierarchy,
                       cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
      print_timing("findContours");

      // Convert frame_detect to color so overlayed circles can be colorized
      cv::cvtColor(frame_detect, frame_detect, cv::COLOR_GRAY2BGR);
      print_timing("cvtColor");

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
      print_timing("find deepest contour");

#ifdef _WIN32
      // Display depth of deepest contour
      string depth_string = to_string(contour_deepest);
      cv::putText(frame, depth_string,
                  cv::Point(frame.cols / 2, frame.rows / 2),
                  cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
#endif
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
            print_timing("enter contours loop");
            // Approximate contour to polygons
            approxPolyDP(cv::Mat(contours[j]), contours_poly[j], 3, true);
            print_timing("approxPolyDP");
            // Find smallest circle which contains whole polygon
            cv::minEnclosingCircle((cv::Mat)contours_poly[j], center[j],
                                   radius[j]);
            print_timing("minEnclosingCircle");
            // Rule out the smaller circles. Gaussian blur might be preferable.
            if (radius[j] > 20)
            {
#ifdef _WIN32
              cv::circle(frame_detect, center[j], (int)radius[j],
                         cv::Scalar(rand() & 255, rand() & 255, rand() & 255), 2, 8, 0);
#endif
              center_average.x += center[j].x;
              center_average.y += center[j].y;
              center_count++;
            }
            j = hierarchy[j][2];
          }
          //          break;
        }
      }

      print_timing("loop finished");

      // Average center points together to find detection point
      center_average.x = (center_average.x / center_count) - 1;
      center_average.y = (center_average.y / center_count) + 1;

      // Clip target area in separate cv::Mat
      target = frame(cv::Rect(center_average.x - 100,
                              center_average.y - 100, 200, 200));
      print_timing("clip frame");

      // Convert target area to greyscale
      cv::cvtColor(target, target_greyscale, cv::COLOR_BGR2GRAY);
      print_timing("cvtColor to greyscale");

      // Clip rectangle from around detection point
      frame_barcode = frame(cv::Rect(center_average.x - 100,
                                     center_average.y - 10, 201, 21));
      frame_barcode.copyTo(barcode);
      print_timing("clip barcode and copy");

      // Convert barcode to greyscale
      cv::cvtColor(barcode, barcode_greyscale, cv::COLOR_BGR2GRAY);

      // Find barcode vertical mean (convert it to a single pixel row)
      cv::reduce(barcode_greyscale, barcode_mean, 0, cv::REDUCE_AVG);

      // Threshold the barcode so it's black and white
      cv::threshold(barcode_mean, barcode_mean, 48, 255, cv::THRESH_BINARY);

      // Vertical stretch it to be more visible
      cv::resize(barcode_mean, barcode, barcode.size(), 0, 0,
                 cv::INTER_NEAREST);

      // Convert barcode into center-out string of numbers
      uint8_t *pixelPtr = (uint8_t *)barcode.data;
      bitset<208> out_array;
      uint8_t out_array_count = 0;
      for (int j = 0; j < barcode.cols; j++)
      {
        out_array[j] = pixelPtr[j] / 255;
        if (pixelPtr[j] > 0)
          out_array_count++;
      }

        // // Insert number of flipped bits at end of bitset
        // bitset<7> out_array_count_bitset(out_array_count);
        // for (int i=0; i<7;i++)
        //   out_array[12][1+i] = out_array_count_bitset[i];

#ifdef _WIN32

      cout << out_array << endl;
#else
    transmit(uart, out_array.to_string());
#endif
      // TODO: reduce black and white barcode to average width of band pairs
      //       working from center point out to fringes. Cut off after about
      //       three band pairs, but be consistent so each dataset has the
      //       same number of values
      // TODO: determine if the fringes moved in or out

#ifdef _WIN32
      // Draw crosshairs over detection point
      cv::drawMarker(frame, center_average,
                     cv::Scalar(rand() & 255, rand() & 255, rand() & 255));
      // Display all detection images (this will not be in production version)
      imshow("1. Source Video", frame);
      imshow("2. Detect Stage", frame_detect);
      imshow("3. Target Stage", target_greyscale);
      imshow("4. Barcode Stage", barcode);
#endif
    }
  }
  //  return 0;
}

#ifdef _WIN32
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
#endif
