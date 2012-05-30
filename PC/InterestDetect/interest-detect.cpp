#include <cv.h>
#include <highgui.h>
#include <string>

using namespace std;

void displayImage(const char* imageLocation)
{
  IplImage* img = 0;
  int height,width,step,channels;
  uchar *data;
  int i,j,k;

  // load an image
  img=cvLoadImage(imageLocation, 1);
  if(!img){
    printf("Could not load image file: %s\n",imageLocation);
    exit(0);
  }

  // get the image data
  height    = img->height;
  width     = img->width;
  step      = img->widthStep;
  channels  = img->nChannels;
  data      = (uchar *)img->imageData;
  printf("Processing a %dx%d image with %d channels\n",height,width,channels);

  // create a window
  cvNamedWindow("mainWin", CV_WINDOW_AUTOSIZE);
  cvMoveWindow("mainWin", 100, 100);

/*
  // invert the image
  for(i=0;i<height;i++) for(j=0;j<width;j++) for(k=0;k<channels;k++)
    data[i*step+j*channels+k]=255-data[i*step+j*channels+k];
*/

  // show the image
  cvShowImage("mainWin", img );

  // wait for a key
  cvWaitKey(0);

  // release the image
  cvReleaseImage(&img );
}

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

	cv::ORB orb (50);
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
