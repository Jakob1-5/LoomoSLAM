#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <algorithm>

namespace loomo {


    // Fisheye camera matrix
    static const cv::Mat kMatrix = (cv::Mat_<double>(3, 3) << 294.7754, 0.0, 329.4582, 0.0, 294.6943, 249.6908, 0.0, 0.0, 1.0000);
    // Fisheye distortion coefficients
    static const cv::Mat dCoeff = (cv::Mat_<double>(1, 5) << -0.2641, 0.0731, 0.0010, -0.0003, -0.0094);



    class LoomoVision {
        const int MAX_FEATURES = 3000;
        const float GOOD_MATCH_PERCENT = 0.15f;
        const float RATIO_THRESHOLD = 0.7f;

        cv::Ptr<cv::Feature2D> detector;
        cv::Ptr<cv::DescriptorMatcher> matcher;

    public:
        LoomoVision();

        void detectFeatures(cv::Mat img, std::vector<cv::KeyPoint>& keyPoints, cv::Mat & descriptors);
        void matchImages(cv::Mat &descriptors1, cv::Mat &descriptors2, std::vector<cv::DMatch> & matches);
        bool tryRecoverPose(std::vector<cv::KeyPoint>& queryKeyPoints, std::vector<cv::KeyPoint>& trainKeyPoints, std::vector<cv::DMatch>& matches, cv::Mat & E, cv::Mat & R, cv::Mat & t);

    };
}



