#include <jni.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

#include <android/log.h>

#define APPNAME "MIXED_SAMPLE"

using namespace std;
using namespace cv;

extern "C" {
JNIEXPORT void JNICALL Java_org_opencv_samples_tutorial4_Sample4View_FindFeatures(JNIEnv* env, jobject thiz, jlong addrGray, jlong addrRgba)
{
	__android_log_print(ANDROID_LOG_VERBOSE, APPNAME, "The value of2 + 22222 is %d", 1+1);

    Mat* pMatGr=(Mat*)addrGray;
    Mat* pMatRgb=(Mat*)addrRgba;
    vector<KeyPoint> v;

    FastFeatureDetector detector(10);
    detector.detect(*pMatGr, v);
    for( size_t i = 0; i < v.size(); i++ )
        circle(*pMatRgb, Point(v[i].pt.x, v[i].pt.y), 10, Scalar(0,255,0,255));
    //
}

JNIEXPORT void JNICALL Java_org_opencv_samples_tutorial4_Sample4View_ORBDetect(JNIEnv* env, jobject thiz, jlong addrGray, jlong addrRgba)
{
    Mat* pMatGr=(Mat*)addrGray;
    Mat* pMatRgb=(Mat*)addrRgba;
    vector<KeyPoint> v;

	cv::ORB orb (50);
	cv::Mat desc2;
	vector<cv::KeyPoint> kp2;

	orb(*pMatGr, cv::Mat(), kp2, desc2);

	cv::drawKeypoints(*pMatGr, kp2, *pMatRgb, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);

	__android_log_print(ANDROID_LOG_VERBOSE, APPNAME, "!!");
}

}
