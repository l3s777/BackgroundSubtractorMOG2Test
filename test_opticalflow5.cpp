// opencv
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
// C
#include <stdio.h>
// C++
#include <iostream>
#include <sstream>

// librealsense libraries
#include <librealsense/rs.hpp>

using namespace cv;
using namespace std;

// Global variables
cv::Mat frame, fgMaskMOG2;
Ptr<BackgroundSubtractor> pMOG2; //MOG2 Background subtractor

/* Main */
int main( int argc, char** argv )
try {
	// Turn on logging.
	rs::log_to_console(rs::log_severity::warn);
	std::cout << "Starting..." << std::endl;

	// create Background Subtractor objects
    	pMOG2 = createBackgroundSubtractorMOG2(); //MOG2 approach

	// realsense context
	rs::context ctx;
        cout << "There are " << ctx.get_device_count() << " connected RealSense devices." << endl << endl;
	// exit if not device is already connected
        if (ctx.get_device_count() == 0) return EXIT_FAILURE;

	// rs defining device to be used
	rs::device * dev = ctx.get_device(0);

	// configure RGB to run at 60 frames per second
	dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 60);
	// configure DEPTH to run at 60 frames per second
	dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);

	// start the device
	dev->start();

	// capture first 50 frames to allow camera to stabilize
	for (int i = 0; i < 50; ++i) dev->wait_for_frames();

	// loop -- DATA ACQUISITION
	while (1) {

		// wait for new frame data
		dev->wait_for_frames();

		// RGB data acquisition
		uchar *rgb = (uchar *) dev->get_frame_data(rs::stream::color);
		// DEPTH data acquisition
		uchar *depth = (uchar *) dev->get_frame_data(rs::stream::depth);

		// data acquisition into opencv::Mat
		// RGB
		const uint8_t * rgb_frame = reinterpret_cast<const uint8_t *>(rgb);
		cv::Mat rgb_ = cv::Mat(480, 640, CV_8UC3, (void*) rgb_frame);
		cvtColor(rgb_, frame, CV_BGR2RGB);

		//update the background model
		pMOG2->apply(frame, fgMaskMOG2);
		Mat aux;
		fgMaskMOG2.copyTo(aux);
		// morphological opening
		cv::erode(aux, aux, cv::Mat());
		cv::erode(aux, aux, cv::Mat());
		cv::dilate(aux, aux, cv::Mat());
		rectangle(frame, cv::Point(5, 5), cv::Point(10,10), cv::Scalar(255,0,255), -1);



		// ***** TEST
		// find contours in the mask and initialize the current (x, y) center of the ball
		std::vector<std::vector<cv::Point> > contours;  // container for the contours
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(aux.clone(), contours,
                     hierarchy,                     /* find the image contours */
                     CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_SIMPLE);

    		cv::Point2f center(-1000,-1000);

		if (contours.size() > 0){

			int largest_area=0;
			int largest_contour_index=0;

			for(int i=0; i < contours.size();i++){
				// iterate through each contour.
				double a = cv::contourArea(cv::Mat(contours[i]),false);
		   		if(a > largest_area) {
					largest_area=a;
					largest_contour_index=i;
		    		}
			}

			cv::Point2f tempcenter;
  			float radius;
			cv::minEnclosingCircle((cv::Mat)contours[largest_contour_index], tempcenter, radius);
			cv::Moments M = cv::moments((cv::Mat)contours[largest_contour_index]);
			center = cv::Point2f(int(M.m10 / M.m00), int(M.m01 / M.m00));

			if (radius > 15) {
				circle(aux, cv::Point(int(tempcenter.x), int(tempcenter.y)), int(radius), cv::Scalar(0, 255, 255), 3);
				circle(aux, center, 5, cv::Scalar(0, 0, 255), -1);
		    		//mainCenter = center; 
			}
		} 



		//show the current frame and the fg masks
		imshow("Frame", frame);
		imshow("FG Mask MOG 2", fgMaskMOG2);
		imshow("Aux", aux);

		char c = (char)waitKey(10);
		if( c == 27 )
		    break;

    }

    return 0;
} catch (const rs::error & e) {
	// method calls against librealsense objects may throw exceptions of type rs::error
	cout << "rs::error was thrown when calling " << e.get_failed_function().c_str()
	<< "-" << e.get_failed_args().c_str() << "-:     " << e.what() << endl;
	return EXIT_FAILURE;
}
