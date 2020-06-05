#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "LoomoOdo.h"
#include "PoseGraph.h"


namespace slamUtils {


    cv::Mat drawOdoTrajectory(std::vector<loomo::Pose> trajectory);

    cv::Mat drawGraph(poseGraph::PoseGraph graph);

    void drawGrid(cv::Mat img, cv::Point origin, int spacing);
}


