#include "Utils.h"

using namespace slamUtils;
using namespace cv;
using namespace std;
using namespace loomo;
using namespace poseGraph;


cv::Mat slamUtils::drawOdoTrajectory(std::vector<loomo::Pose> trajectory)
{
    Mat trajImg = Mat::zeros(820, 820, CV_8UC3);
    double xMax = 0.0, xMin = 0.0, yMax = 0.0, yMin = 0.0;
    for (const auto & pose : trajectory) {
        xMax = max(xMax, pose.x);
        xMin = min(xMin, pose.x);
        yMax = max(yMax, pose.y);
        yMin = min(yMin, pose.y);
    }
    double scale = 800 / max(xMax - xMin, yMax - yMin);

    for (const auto pose : trajectory) {
        circle(trajImg, Point(scale*(pose.x - xMin) + 10.0, scale*(pose.y - yMin) + 10.0), 2, Scalar(255.0, 0, 255.0), 1);
    }
    circle(trajImg, Point(scale*(trajectory.front().x - xMin) + 10.0, scale*(trajectory.front().y - yMin) + 10.0), 6, Scalar(0.0, 255.0, 0.0), 2);
    circle(trajImg, Point(scale*(trajectory.back().x - xMin) + 10.0, scale*(trajectory.back().y - yMin) + 10.0), 6, Scalar(0.0, 0.0, 255.0), 2);

    flip(trajImg, trajImg, 0);
    line(trajImg, Point(360.0, 20.0), Point(360.0 + 1000 * scale, 20.0), Scalar(190.0, 190.0, 190.0), 5, 4);
    putText(trajImg, "1 m", Point(360.0, 50.0), FONT_HERSHEY_TRIPLEX, 1.0, Scalar(190.0, 190.0, 190.0), 1);

    return trajImg;
}

cv::Mat slamUtils::drawGraph(poseGraph::PoseGraph graph)
{
    Mat img = Mat::zeros(820, 820, CV_8UC3);
    vector<Pose> graphTraj;
    vector<pair<Pose, Pose>> edgePosePairs;

    map<uint64_t, poseGraph::Node> nodes = *graph.getNodeMap();
    for (const auto & node : nodes) {
        Pose to = node.second.pose;
        graphTraj.push_back(to);
        for (const auto & edge : node.second.edges) {
            auto mapPtr = nodes.find(edge.from);
            if (mapPtr != nodes.end()) {
                Pose from = mapPtr->second.pose;
                edgePosePairs.push_back({ from, to });
            }
            else {
                // did not find the 'from' node
                cout << "Did not find node: " << edge.from << endl;
            }
        }
    }

    double xMax = 0.0, xMin = 0.0, yMax = 0.0, yMin = 0.0;
    for (const auto & pose : graphTraj) {
        xMax = max(xMax, pose.x);
        xMin = min(xMin, pose.x);
        yMax = max(yMax, pose.y);
        yMin = min(yMin, pose.y);
    }
    double scale = 800 / max(xMax - xMin, yMax - yMin);
    Point startPt = Point(scale*(graphTraj.front().x - xMin) + 10.0, scale*(graphTraj.front().y - yMin) + 10.0);
    drawGrid(img, startPt, scale * 1000);


    circle(img, startPt, 8, Scalar(0.0, 255.0, 0.0), 2);
    circle(img, Point(scale*(graphTraj.back().x - xMin) + 10.0, scale*(graphTraj.back().y - yMin) + 10.0), 8, Scalar(0.0, 0.0, 255.0), 2);
    for (const auto & pose : graphTraj) {
        Point pt = Point(scale*(pose.x - xMin) + 10.0, scale*(pose.y - yMin) + 10.0);
        Point pt1 = Point(15 * cos(pose.theta) - 0 * sin(pose.theta), 15 * sin(pose.theta) + 0 * cos(pose.theta)) + pt;
        Point pt2 = Point(-7 * cos(pose.theta) - 7 * sin(pose.theta), -7 * sin(pose.theta) + 7 * cos(pose.theta)) + pt;
        Point pt3 = Point(-7 * cos(pose.theta) + 7 * sin(pose.theta), -7 * sin(pose.theta) - 7 * cos(pose.theta)) + pt;
        vector<vector<Point>> pts = { {pt1, pt2, pt3} };
        drawContours(img, pts, -1, CV_RGB(200, 200, 200), FILLED);
        circle(img, pt, 3, Scalar(255.0, 0, 255.0), FILLED);
    }

    for (const auto & edge : edgePosePairs) {
        line(
            img,
            Point(scale*(edge.first.x - xMin) + 10.0, scale*(edge.first.y - yMin) + 10.0),
            Point(scale*(edge.second.x - xMin) + 10.0, scale*(edge.second.y - yMin) + 10.0),
            Scalar(255.0, 255.0, 0.0),
            1,
            LineTypes::LINE_8
        );
    }

    flip(img, img, 0);
    rectangle(img, Point(360.0, 17.0), Point(360.0 + 1000 * scale, 22.0), Scalar(190.0, 190.0, 190.0), -1);
    putText(img, "1 m", Point(360.0, 50.0), FONT_HERSHEY_TRIPLEX, 1.0, Scalar(190.0, 190.0, 190.0), 1);
    return img;
}

void slamUtils::drawGrid(cv::Mat img, cv::Point origin, int spacing) {
    int height = img.rows;
    int width = img.cols;
    int hLines = height / spacing + 1;
    int vLines = width / spacing + 1;

    int orStepOffY = origin.y / spacing + 1; // where to draw the first line
    int orStepOffX = origin.x / spacing + 1;

    for (int i = 0; i < hLines; ++i) {
        Point pt1 = { 0, (origin.y + (i - orStepOffY) * spacing) };
        Point pt2 = { width, (origin.y + (i - orStepOffY) * spacing) };
        line(img, pt1, pt2, Scalar(60.0, 60.0, 60.0));
    }
    for (int i = 0; i < vLines; ++i) {
        Point pt1 = { (origin.x + (i - orStepOffX) * spacing) , 0 };
        Point pt2 = { (origin.x + (i - orStepOffX) * spacing) , height };
        line(img, pt1, pt2, Scalar(60.0, 60.0, 60.0));
    }
}
