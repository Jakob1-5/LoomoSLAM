#include "LoomoVision.h"

using namespace std;
using namespace cv;

loomo::LoomoVision::LoomoVision() {
    detector = ORB::create(MAX_FEATURES);
    matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);
}



void loomo::LoomoVision::detectFeatures(Mat img, vector<KeyPoint>& keyPoints, Mat & descriptors) {
    //detector->detect(img, keyPoints);
    //detector->compute(img, keyPoints, descriptors);
    detector->detectAndCompute(img, noArray(), keyPoints, descriptors);
}

void loomo::LoomoVision::matchImages(cv::Mat & descriptors1, cv::Mat & descriptors2, vector<DMatch> & matches)
{
    matches.clear();
    //Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);
    //matcher->match(descriptors1, descriptors2, matches, noArray());

    vector<vector<DMatch>> allMatches;
    matcher->knnMatch(descriptors1, descriptors2, allMatches, 2);

    // Lowe's ratio test
    for (size_t i = 0; i < allMatches.size(); i++) {
        //matches.push_back(allMatches[i][0]);
        if (allMatches[i][0].distance < RATIO_THRESHOLD * allMatches[i][1].distance) {
            matches.push_back(allMatches[i][0]);
        }
    }

    // Keep only the n% best matches
    //sort(matches.begin(), matches.end());
    //const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
    //matches.erase(matches.begin() + numGoodMatches, matches.end());

}

bool loomo::LoomoVision::tryRecoverPose(std::vector<cv::KeyPoint>& queryKeyPoints, std::vector<cv::KeyPoint>& trainKeyPoints, std::vector<cv::DMatch>& matches, cv::Mat& E, cv::Mat& R, cv::Mat& t)
{
    if (matches.size() < 6)
        return false;
    vector<Point2f> currPts, currUndstPts;
    vector<Point2f> prevPts, prevUndstPts;

    for (size_t i = 0; i < matches.size(); ++i) {
        currPts.push_back(queryKeyPoints[matches[i].queryIdx].pt);
        prevPts.push_back(trainKeyPoints[matches[i].trainIdx].pt);
    }
    undistortPoints(currPts, currUndstPts, kMatrix, dCoeff, noArray(), kMatrix);
    undistortPoints(prevPts, prevUndstPts, kMatrix, dCoeff, noArray(), kMatrix);

    Mat mask;
    E = findEssentialMat(currUndstPts, prevUndstPts, kMatrix, RANSAC, 0.999, 1.0, mask);
    if ((E.rows == 3) && (E.cols == 3)) 
        recoverPose(E, currUndstPts, prevUndstPts, kMatrix, R, t, mask);
    else
        return false;



    return true;
}

