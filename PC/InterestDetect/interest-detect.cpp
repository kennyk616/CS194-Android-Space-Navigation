#include <cv.h>
#include <highgui.h>

using namespace std;

int main ( int argc, const char **argv )
{
	cv::VideoCapture cap;

	if (argc < 2) {
		cap.open(0);
	} else {
		cap.open(string(argv[1]));
	}

	if(!cap.isOpened()) {  // check if we succeeded
		fprintf(stderr, "ERROR: capture is NULL \n");
		getchar();
		return -1;
	}

	// Create a window in which the captured images will be presented
	cv::namedWindow("mywindow", CV_WINDOW_AUTOSIZE);
	// Show the image captured from the camera in the window and repeat
	while (1) {
		// Get one frame
		cv::Mat frame;
		cap >> frame;

		cv::flip(frame, frame, 1);

		cv::ORB orb;
		cv::Mat grey1, desc1;
		vector<cv::KeyPoint> kp1;

		grey1 = frame;

		orb(grey1, cv::Mat(), kp1, desc1);
		cv::Mat img_kp1;
		cv::drawKeypoints(grey1, kp1, img_kp1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

		cv::imshow("mywindow", img_kp1);

		if ((cvWaitKey(10) & 255) == 27) // ESC
			break;
	}
	cv::destroyWindow("mywindow");
	return 0;
}
