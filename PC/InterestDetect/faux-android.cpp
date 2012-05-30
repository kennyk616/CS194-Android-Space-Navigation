/*
 * faux-android.cpp
 *
 *  Created on: May 29, 2012
 *      Author: david
 */


#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <highgui.h>
#include <iostream>

using namespace std;
using namespace cv;

#define __android_log_print(...) //nothing

bool initd = false;
cv::Mat prev_frame;
cv::Mat prev_desc;
vector<cv::KeyPoint> prev_kp;





//using namespace std;
//
//const int draw_shift_bits = 4;
//const int draw_multiplier = 1 << draw_shift_bits;
//
///*
// * Functions to draw keypoints and matches.
// */
//static inline void _drawKeypoint( Mat& img, const KeyPoint& p, const Scalar& color, int flags )
//{
//    CV_Assert( !img.empty() );
//    Point center( cvRound(p.pt.x * draw_multiplier), cvRound(p.pt.y * draw_multiplier) );
//
//    if( flags & DrawMatchesFlags::DRAW_RICH_KEYPOINTS )
//    {
//        int radius = cvRound(p.size/2 * draw_multiplier); // KeyPoint::size is a diameter
//
//        // draw the circles around keypoints with the keypoints size
//        circle( img, center, radius, color, 1, CV_AA, draw_shift_bits );
//
//        // draw orientation of the keypoint, if it is applicable
//        if( p.angle != -1 )
//        {
//            float srcAngleRad = p.angle*(float)CV_PI/180.f;
//            Point orient( cvRound( cos(srcAngleRad)*radius ),
//                          cvRound(-sin(srcAngleRad)*radius ) // "-" to invert orientation of axis y
//                        );
//            line( img, center, center+orient, color, 1, CV_AA, draw_shift_bits );
//        }
//#if 0
//        else
//        {
//            // draw center with R=1
//            int radius = 1 * draw_multiplier;
//            circle( img, center, radius, color, 1, CV_AA, draw_shift_bits );
//        }
//#endif
//    }
//    else
//    {
//        // draw center with R=3
//        int radius = 3 * draw_multiplier;
//        circle( img, center, radius, color, 1, CV_AA, draw_shift_bits );
//    }
//}
//
//void drawMyKeypoints( const Mat& image, const vector<KeyPoint>& keypoints, Mat& outImage,
//                    const Scalar& _color, int flags )
//{
//    if( !(flags & DrawMatchesFlags::DRAW_OVER_OUTIMG) )
//    {
//        if( image.type() == CV_8UC3 )
//        {
//            image.copyTo( outImage );
//        }
//        else if( image.type() == CV_8UC1 )
//        {
//            cvtColor( image, outImage, CV_GRAY2BGR );
//        }
//        else
//        {
//            CV_Error( CV_StsBadArg, "Incorrect type of input image.\n" );
//        }
//    }
//
//    RNG& rng=theRNG();
//    bool isRandColor = _color == Scalar::all(-1);
//
//    CV_Assert( !outImage.empty() );
//    vector<KeyPoint>::const_iterator it = keypoints.begin(),
//                                     end = keypoints.end();
//    for( ; it != end; ++it )
//    {
//        Scalar color = isRandColor ? Scalar(rng(256), rng(256), rng(256)) : _color;
//        _drawKeypoint( outImage, *it, color, flags );
//    }
//}
//
//static void _prepareImgAndDrawKeypoints( const Mat& img1, const vector<KeyPoint>& keypoints1,
//                                         const Mat& img2, const vector<KeyPoint>& keypoints2,
//                                         Mat& outImg, Mat& outImg1, Mat& outImg2,
//                                         const Scalar& singlePointColor, int flags )
//{
//    Size size( MAX(img1.cols, img2.cols),  img1.rows + img2.rows );
//    if( flags & DrawMatchesFlags::DRAW_OVER_OUTIMG )
//    {
//        if( size.width > outImg.cols || size.height > outImg.rows )
//            CV_Error( CV_StsBadSize, "outImg has size less than need to draw img1 and img2 together" );
//        outImg1 = outImg( Rect(0, 0, img1.cols, img1.rows) );
//        outImg2 = outImg( Rect(img1.cols, 0, img2.cols, img2.rows) );
//    }
//    else
//    {
//        outImg.create( size, CV_MAKETYPE(img1.depth(), 3) );
//        outImg1 = outImg( Rect(0, 0, img1.cols, img1.rows) );
//        outImg2 = outImg( Rect(0, img1.rows, img2.cols, img2.rows) );
//
//        if( img1.type() == CV_8U )
//            cvtColor( img1, outImg1, CV_GRAY2BGR );
//        else
//            img1.copyTo( outImg1 );
//
//        if( img2.type() == CV_8U )
//            cvtColor( img2, outImg2, CV_GRAY2BGR );
//        else
//            img2.copyTo( outImg2 );
//    }
//
//    // draw keypoints
//    if( !(flags & DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS) )
//    {
//        Mat outImg1 = outImg( Rect(0, 0, img1.cols, img1.rows) );
//        drawMyKeypoints( outImg1, keypoints1, outImg1, singlePointColor, flags + DrawMatchesFlags::DRAW_OVER_OUTIMG );
//
//        Mat outImg2 = outImg( Rect(img1.cols, 0, img2.cols, img2.rows) );
//        drawMyKeypoints( outImg2, keypoints2, outImg2, singlePointColor, flags + DrawMatchesFlags::DRAW_OVER_OUTIMG );
//    }
//}
//
//static inline void _drawMatch( Mat& outImg, Mat& outImg1, Mat& outImg2 ,
//                          const KeyPoint& kp1, const KeyPoint& kp2, const Scalar& matchColor, int flags )
//{
//    RNG& rng = theRNG();
//    bool isRandMatchColor = matchColor == Scalar::all(-1);
//    Scalar color = isRandMatchColor ? Scalar( rng(256), rng(256), rng(256) ) : matchColor;
//
//    _drawKeypoint( outImg1, kp1, color, flags );
//    _drawKeypoint( outImg2, kp2, color, flags );
//
//    Point2f pt1 = kp1.pt,
//            pt2 = kp2.pt,
//            dpt2 = Point2f( pt2.x, std::min(pt2.y+outImg1.rows, float(outImg.rows-1)) );
//
//    line( outImg,
//		  Point(cvRound(pt1.x*draw_multiplier), cvRound(pt1.y*draw_multiplier)),
//		  Point(cvRound(dpt2.x*draw_multiplier), cvRound(dpt2.y*draw_multiplier)),
//          color, 1, CV_AA, draw_shift_bits );
//}
//
//void drawMyMatches( const Mat& img1, const vector<KeyPoint>& keypoints1,
//                  const Mat& img2, const vector<KeyPoint>& keypoints2,
//                  const vector<DMatch>& matches1to2, Mat& outImg,
//                  const Scalar& matchColor, const Scalar& singlePointColor,
//                  const vector<char>& matchesMask, int flags )
//{
//    if( !matchesMask.empty() && matchesMask.size() != matches1to2.size() )
//        CV_Error( CV_StsBadSize, "matchesMask must have the same size as matches1to2" );
//
//    Mat outImg1, outImg2;
//    _prepareImgAndDrawKeypoints( img1, keypoints1, img2, keypoints2,
//                                 outImg, outImg1, outImg2, singlePointColor, flags );
//
//    // draw matches
//    for( size_t m = 0; m < matches1to2.size(); m++ )
//    {
//        int i1 = matches1to2[m].queryIdx;
//        int i2 = matches1to2[m].trainIdx;
//        if( matchesMask.empty() || matchesMask[m] )
//        {
//            const KeyPoint &kp1 = keypoints1[i1], &kp2 = keypoints2[i2];
//            _drawMatch( outImg, outImg1, outImg2, kp1, kp2, matchColor, flags );
//        }
//    }
//}
//
//void drawMyMatches( const Mat& img1, const vector<KeyPoint>& keypoints1,
//                  const Mat& img2, const vector<KeyPoint>& keypoints2,
//                  const vector<vector<DMatch> >& matches1to2, Mat& outImg,
//                  const Scalar& matchColor, const Scalar& singlePointColor,
//                  const vector<vector<char> >& matchesMask, int flags )
//{
//    if( !matchesMask.empty() && matchesMask.size() != matches1to2.size() )
//        CV_Error( CV_StsBadSize, "matchesMask must have the same size as matches1to2" );
//
//    Mat outImg1, outImg2;
//    _prepareImgAndDrawKeypoints( img1, keypoints1, img2, keypoints2,
//                                 outImg, outImg1, outImg2, singlePointColor, flags );
//
//    // draw matches
//    for( size_t i = 0; i < matches1to2.size(); i++ )
//    {
//        for( size_t j = 0; j < matches1to2[i].size(); j++ )
//        {
//            int i1 = matches1to2[i][j].queryIdx;
//            int i2 = matches1to2[i][j].trainIdx;
//            if( matchesMask.empty() || matchesMask[i][j] )
//            {
//                const KeyPoint &kp1 = keypoints1[i1], &kp2 = keypoints2[i2];
//                _drawMatch( outImg, outImg1, outImg2, kp1, kp2, matchColor, flags );
//            }
//        }
//    }
//}
//
//








void ORBDetect(Mat *pMatGr, Mat *pMatRgb)
{
    vector<KeyPoint> v;

	cv::ORB orb (50);
	cv::Mat desc;
	vector<cv::KeyPoint> kp;

	orb(*pMatGr, cv::Mat(), kp, desc);

	Mat out_canvas;
	//cv::drawKeypoints(*pMatGr, kp2, out_canvas, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

	if (initd) {
		cv::BruteForceMatcher<cv::HammingLUT > matcher;
		vector<cv::DMatch> matches;
		matcher.match(prev_desc, desc, matches);

		cv::drawMatches( prev_frame, prev_kp, *pMatGr, kp, matches, out_canvas );

//		drawMyMatches( prev_frame, prev_kp, *pMatGr, kp, matches, out_canvas,
//				 Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::DEFAULT
//		);

		cv::resize(out_canvas, out_canvas, Size(prev_frame.cols, prev_frame.rows), 0, 0);
	} else {
		out_canvas = *pMatRgb;
	}

	cvtColor(out_canvas, *pMatRgb, CV_BGR2BGRA);

	prev_frame = *pMatGr;
	prev_desc = desc;
	prev_kp = kp;
	initd = true;
}

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
		cv::Mat frame;
		cv::Mat pMatRgb, pMatGr;
		cap >> frame;
		cv::flip(frame, frame, 1);

		cout << frame.cols << "x"<<frame.rows << "CH: "<< frame.channels() << endl;

//		Mat zeros (frame.cols, frame.rows, CV_8UC1, 0);
//		Mat in[] = {frame, zeros};
//		Mat out[] = {*pMatRgb};
//		int from_to[] = { 0,0, 1,1, 2,2, 3,3 };
//		cv::mixChannels(in, 2, out, 1, from_to, 4);

		cvtColor(frame, pMatRgb, CV_BGR2BGRA);
		cvtColor(frame, pMatGr, CV_BGR2GRAY);

		ORBDetect(&pMatGr, &pMatRgb);

		cv::imshow("mywindow", pMatRgb);

		if ((cvWaitKey(10) & 255) == 27) // ESC
			break;
	}
}

