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

int main ( int argc, char **argv )
{
	/*
	cvNamedWindow( "My Window", 1 );
	IplImage *img = cvCreateImage( cvSize( 640, 480 ), IPL_DEPTH_8U, 1 );
	CvFont font;
	double hScale = 1.0;
	double vScale = 1.0;
	int lineWidth = 1;
	cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC,
			hScale, vScale, 0, lineWidth );
	cvPutText( img, "Hello World!", cvPoint( 200, 400 ), &font,
			cvScalar( 255, 255, 0 ) );
	cvShowImage( "My Window", img );
	cvWaitKey();
	*/
	string img = "mars_rocks.jpg";
	displayImage(img.c_str());
	return 0;
}
