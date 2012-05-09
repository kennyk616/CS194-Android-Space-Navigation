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

	cv::ORB orb;
	cv::Mat grey1, desc1, grey2, desc2;
	vector<cv::KeyPoint> kp1, kp2;

	cv::Mat frame1, frame2;
	cap >> frame2;
	cv::flip(frame2, frame2, 1);
	grey2 = frame2;
	orb(grey2, cv::Mat(), kp2, desc2);

	// Show the image captured from the camera in the window and repeat
	while (1) {
		// Get one frame
		frame1 = frame2;
		grey1 = grey2;
		desc1 = desc2;
		kp1 = kp2;
		cap >> frame2;
		cv::flip(frame2, frame2, 1);
		grey2 = frame2;
		orb(grey2, cv::Mat(), kp2, desc2);

		cv::Mat img_kp1;
//		cv::drawKeypoints(grey1, kp1, img_kp1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

//		cv::FlannBasedMatcher matcher;
//		std::vector<cv::DMatch> matches;
//		matcher.match(desc1, desc2, matches);

		cv::BruteForceMatcher<cv::HammingLUT > matcher;
		vector<cv::DMatch> matches;
		matcher.match(desc1, desc2, matches);

//		nth_element(matches.begin(), matches.begin()+24, matches.end());
//		matches.erase(matches.begin()+25, matches.end());

		cv::Mat img_matches;
		cv::drawMatches( grey1, kp1, grey2, kp2, matches, img_matches );

		cv::imshow("mywindow", img_matches);

		if ((cvWaitKey(10) & 255) == 27) // ESC
			break;
	}
	cv::destroyWindow("mywindow");
	return 0;
}
