#include <iostream>
#include "opencv2/core.hpp"
#include <interest_point/matching.h>

#ifdef HAVE_OPENCV_XFEATURES2D
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <gflags/gflags.h>
using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;
DEFINE_int32( minHessian, 400, "minHessian" );
DEFINE_string( input1, "/srv/novus_1/amoravar/data/images/latest_map_imgs/2020-09-24/1599242127.7988508.jpg", "Path to input image 1." );
DEFINE_string( input2, "/srv/novus_1/amoravar/data/images/latest_map_imgs/2020-09-24/1599241674.0450184.jpg", "Path to input image 2." );
DEFINE_double( ratio, .8, "ratio test" );
const char* keys =
    "{ help h |                  | Print help message. }"
    "{ input1 | /srv/novus_1/amoravar/data/images/latest_map_imgs/2020-09-24/1599242127.7988508.jpg | Path to input image 1. }"
    "{ input2 | /srv/novus_1/amoravar/data/images/latest_map_imgs/2020-09-24/1599241674.0450184.jpg | Path to input image 2. }";
int main( int argc, char* argv[] )
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    Mat img1 = imread( (FLAGS_input1), IMREAD_GRAYSCALE );
    Mat img2 = imread( (FLAGS_input2), IMREAD_GRAYSCALE );
    if ( img1.empty() || img2.empty() )
    {
        cout << "Could not open or find the image!\n" << endl;
        return -1;
    }
    //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
    int minHessian = FLAGS_minHessian;
    Ptr<SURF> detector = SURF::create( minHessian );
    std::vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;
    detector->detectAndCompute( img1, noArray(), keypoints1, descriptors1 );
    detector->detectAndCompute( img2, noArray(), keypoints2, descriptors2 );
    std::cout<< "keypoints1.size() = " << keypoints1.size() << std::endl;
    std::cout<< "keypoints2.size() = " << keypoints2.size() << std::endl;
    // //-- Step 2: Matching descriptor vectors with a FLANN based matcher
    // // Since SURF is a floating-point descriptor NORM_L2 is used
    
    std::vector<cv::DMatch> find_matches;
    interest_point::FindMatches(descriptors1,
                                descriptors2,
                                &find_matches);

    std::cout<< "find_matches.size() = " << find_matches.size() << std::endl;

    // Traditional floating point descriptor
      cv::FlannBasedMatcher matcher;
      std::vector<cv::DMatch> * good_matches = new std::vector<cv::DMatch>;
      std::vector<std::vector<cv::DMatch> > possible_matches;
      matcher.knnMatch(descriptors1, descriptors2, possible_matches, 2);
      good_matches->clear();
      good_matches->reserve(possible_matches.size());
      for (std::vector<cv::DMatch> const& best_pair : possible_matches) {
        if (best_pair.size() == 1) {
          // This was the only best match, push it.
          good_matches->push_back(best_pair.at(0));
        } else {
          // Push back a match only if it is 25% better than the next best.
          if (best_pair.at(0).distance < FLAGS_ratio * best_pair.at(1).distance) {
            good_matches->push_back(best_pair[0]);
          }
        }
      }

    //-- Draw matches
    Mat img_matches;
    drawMatches( img1, keypoints1, img2, keypoints2, *good_matches, img_matches, Scalar::all(-1),
                 Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    //-- Show detected matches
    namedWindow("Good Matches", cv::WINDOW_FULLSCREEN);
    imshow("Good Matches", img_matches );
    waitKey();
    return 0;
}
#else
int main()
{
    std::cout << "This tutorial code needs the xfeatures2d contrib module to be run." << std::endl;
    return 0;
}
#endif