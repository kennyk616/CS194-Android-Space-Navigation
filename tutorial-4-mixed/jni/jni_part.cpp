#include <jni.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

#include <android/log.h>

#define APPNAME "MIXED_SAMPLE"

using namespace std;
using namespace cv;

bool initd = false;
cv::Mat prev_frame;
cv::Mat prev_desc;
vector<cv::KeyPoint> prev_kp;

extern "C" {
JNIEXPORT void JNICALL Java_org_opencv_samples_tutorial4_Sample4View_FindFeatures(JNIEnv* env, jobject thiz, jlong addrGray, jlong addrRgba)
{
    Mat* pMatGr=(Mat*)addrGray;
    Mat* pMatRgb=(Mat*)addrRgba;
    vector<KeyPoint> v;

    FastFeatureDetector detector(10);
    detector.detect(*pMatGr, v);
    for( size_t i = 0; i < v.size(); i++ )
        circle(*pMatRgb, Point(v[i].pt.x, v[i].pt.y), 10, Scalar(0,255,0,255));
}

JNIEXPORT void JNICALL Java_org_opencv_samples_tutorial4_Sample4View_ORBDetect(JNIEnv* env, jobject thiz, jlong addrGray, jlong addrRgba)
{
    Mat* pMatGr=(Mat*)addrGray;
    Mat* pMatRgb=(Mat*)addrRgba;

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

}
