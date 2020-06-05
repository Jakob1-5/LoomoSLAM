

#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <algorithm>
#include <tuple>


#include "LoomoOdo.h"
#include "LoomoCSVReader.h"
#include "Utils.h"
#include "PoseGraph.h"
#include "LoomoVision.h"


#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <opencv2/core/eigen.hpp>


using namespace cv;
using namespace std;
using std::cout; using std::endl;
using loomo::Pose; using loomo::kMatrix; using loomo::dCoeff;

int main()
{
    string path = __FILE__; //gets source code path, including file name
    path = path.substr(0, 1 + path.find_last_of('\\')); //removes file name
    string pathLoomo1 = path + "LoomoRecordings\\0069\\";

    vector<CSVData> dataLoomo1 = LoomoCSVReader::string2data(LoomoCSVReader::getData(pathLoomo1 + "sensor_data.csv"));


    VideoCapture capFisheyeLoomo1(pathLoomo1 + "hallway_00_fisheye.avi");
    //VideoCapture capFisheyeLoomo1(0);
    if (!capFisheyeLoomo1.isOpened()) {
        cout << "Error openeing video file" << endl;
        return -1;
    }
    Mat capFrame, greyFrame, comp1, comp2, compImg;
    vector<KeyPoint> kp, kp1, kp2;
    Mat descriptors, dscr1, dscr2;
    //vector<DMatch> matches;
    loomo::LoomoVision mVision;





    loomo::LoomoOdo loomo1;
    vector<loomo::Pose> odoTraj = { {0.0, 0.0, 0.0} };
    vector<loomo::Pose> tmpOdoTraj = { {0.0, 0.0, 0.0} };


    poseGraph::PoseGraph graph1(1U);
    map<uint64_t, poseGraph::Node> *nodeMap = graph1.getNodeMap();
    uint64_t currentNodeKey = nodeMap->begin()->first;
    uint64_t prevNodeKey;


    Pose odoIncrement = { 0.0, 0.0, 0.0 };
    double grossTraveledDistance = 0;
    double incrementDistance = 0;
    //Eigen::Matrix3d odoUncertainty;

    for (size_t i = 1; i < dataLoomo1.size(); ++i) {
        int dTickL = dataLoomo1[i].tick_left - dataLoomo1[i - 1].tick_left;
        int dTickR = dataLoomo1[i].tick_right - dataLoomo1[i - 1].tick_right;
        loomo1.incrementPose(dTickL, dTickR);

        //odoUncertainty = loomo1.getUncertainty();

        odoIncrement += loomo1.getIncrement();
        tmpOdoTraj.push_back(odoIncrement);
        incrementDistance += loomo1.getIncrement().norm();
        grossTraveledDistance += loomo1.getIncrement().norm();

        if (dataLoomo1[i].fisheye_idx != dataLoomo1[i - 1].fisheye_idx) {
            capFisheyeLoomo1 >> capFrame;
            if (capFrame.empty()) {
                cout << "End of video" << endl;
                break;
            }
            cvtColor(capFrame, greyFrame, COLOR_BGR2GRAY);
        }


        //// add new node every 1m
        if ((incrementDistance > 1000.0) || (abs(odoIncrement.theta) > loomo::PI / 4)) {
            prevNodeKey = currentNodeKey;

            // append to odoTraj
            {
                double theta = odoTraj.back().theta;
                Eigen::Vector3d prevPose(odoTraj.back().x, odoTraj.back().y, theta);
                Eigen::Matrix3d rot;
                rot << cos(theta), -sin(theta), 0,
                    sin(theta), cos(theta), 0,
                    0, 0, 1;
                for (auto & pose : tmpOdoTraj) {
                    Eigen::Vector3d tmp(pose.x, pose.y, pose.theta);
                    tmp = rot * tmp;
                    tmp += prevPose;
                    pose = { tmp(0), tmp(1), tmp(2) };
                }
                odoTraj.insert(odoTraj.end(), tmpOdoTraj.begin(), tmpOdoTraj.end());
            }
            currentNodeKey = graph1.addNodeFromOdo(loomo1.getPose(), loomo1.getUncertainty());

            // iterator (pointer) to nodes
            auto itCurrentNode = nodeMap->find(currentNodeKey);
            auto itPrevNode = nodeMap->find(prevNodeKey);

            mVision.detectFeatures(
                greyFrame,
                itCurrentNode->second.keyPoints,
                itCurrentNode->second.descriptors
            );

            //Match current and previous node
            if (!(itPrevNode->second.descriptors).empty()) {
                vector<DMatch> matches;
                mVision.matchImages(
                    itCurrentNode->second.descriptors,
                    itPrevNode->second.descriptors,
                    matches
                );
                // need at least 6 points to estimate the essential matrix
                //if (matches.size() > 5) {
                Mat E, R, t;
                if (mVision.tryRecoverPose(itCurrentNode->second.keyPoints, itPrevNode->second.keyPoints, matches, E, R, t)) {
                    //    //Eigen::Vector3d pose;
                    //    //pose << itCurrentNode->second.pose.x, itCurrentNode->second.pose.y, itCurrentNode->second.pose.theta;
                    //    ////Eigen::Matrix4d rotPitch, rot, T;
                    //    //Mat rotPitch, rot, T = Mat::zeros(4,4, CV_64FC1);
                    //    //rotPitch = (cv::Mat_<double>(4, 4) << 
                    //    //    0, 0, -1, 0,
                    //    //    0, 1, 0, 0,
                    //    //    1, 0, 0, 0,
                    //    //    0, 0, 0, 1);
                    //    //rot = (Mat_<double>(4,4) <<
                    //    //    cos(pose(2)), -sin(pose(2)), 0, 0,
                    //    //    sin(pose(2)), cos(pose(2)), 0, 0,
                    //    //    0, 0, 1, 0,
                    //    //    0, 0, 0, 1);
                    //    //Rect srcRectR(Point(0, 0), Size(R.rows, R.cols));
                    //    //Rect dstRectR(Point(0, 0), srcRectR.size());
                    //    //R(srcRectR).copyTo(T(dstRectR));
                    //    //Rect srcRectT(Point(0, 0), Size(R.rows, R.cols));
                    //    //Rect dstRectT(Point(0, 0), srcRectT.size());
                    //    //t(srcRectT).copyTo(T(dstRectT));

                    //    //Mat constraint6dof = rot * rotPitch*T;



                    //    //t = R * t;
                    double ang = asin(R.at<double>(0, 1));
                    cout << "Ang = " << ang << ", theta = " << odoIncrement.theta << endl;
                    //if (R.at<double>(1, 0) < 0)
                    //    ang *= -1;
                    double scale = odoIncrement.norm();
                    Pose tmp = { t.at<double>(2)*scale, t.at<double>(1)*scale, ang };
                    Eigen::Matrix3d covar = Eigen::Matrix3d::Zero();
                    covar.block<2, 2>(0, 0) = loomo1.getUncertainty().block<2, 2>(0, 0);
                    covar.block(0, 0, 2, 2) *= 1.5;
                    covar(2, 2) = loomo1.getUncertainty()(2, 2);
                    //covar.block<2, 1>(0, 2) = loomo1.getUncertainty().block<2, 1>(0, 2) * 0.5;
                    //covar.block<1, 2>(2, 0) = loomo1.getUncertainty().block<1, 2>(2, 0) * 0.5;
                    double scale1 = 4.8 * exp(-0.04*(matches.size() - 6)) + 0.2;
                    double scale2 = 50.0 / matches.size();
                    //cout << "Covar theta odo: " << covar(2, 2) << endl;
                    covar *= scale1;
                    //cout << "Covar theta scaled: " << covar(2, 2) << endl;
                    //covar(2, 2) = scale1;
                    graph1.addVirtualEdge(tmp, prevNodeKey, covar);

                    //graph1.optimizeGraph();
                //    //cout << "R:" << endl << R << endl;
                //    //cout << "Matches: " << matches.size() << endl;
                //    //cout << "Scale log: " << scale2 << ", scale exp: " << scale1 << endl;
                //    //cout << "t:" << endl << t << endl;
                //    //cout << "img matching: (" << tmp.x << ", " << tmp.y << ", " << tmp.theta << ")" << endl;
                //    //cout << "odo: (" << odoIncrement.x << ", " << odoIncrement.y << ", " << odoIncrement.theta << ")" << endl;
                }
                else {
                    cout << "Recover pose unsuccessful. N.o. matches = " << matches.size() << endl;
                }

            }


            odoIncrement = { 0.0, 0.0, 0.0 };
            tmpOdoTraj.clear();
            loomo1.reset();
            incrementDistance = 0;
        }

    }

    cout << "Gross distance: " << grossTraveledDistance / 1000 << " m" << endl;
    cout << "Odo \"miss\": " << odoTraj.back().norm() << " mm" << endl;




    /*
    check for matches
    */
    //for (auto node_i = nodeMap->begin(); node_i != nodeMap->end(); node_i++) {
    //    int i = 0;
    //    //cout << "Node " << (uint32_t)node_i->first << endl;
    //    for (auto node_j = nodeMap->begin(); node_j != nodeMap->end(); node_j++) {
    //        vector<DMatch> matches;
    //        ++i;

    //        if (!(node_j->second.descriptors).empty() && !(node_i->second.descriptors).empty()) {
    //            mVision.matchImages(
    //                node_i->second.descriptors,
    //                node_j->second.descriptors,
    //                matches
    //            );
    //            cout << matches.size();
    //        }
    //        else {
    //            cout << "0";
    //        }
    //        //cout << (uint32_t)node_j->first;
    //        cout << ",";
    //    }
    //    cout << endl;
    //    //cout << --i << endl;
    //}




    poseGraph::PoseGraph graph2 = graph1;
    //Eigen::Matrix3d covar = Eigen::Matrix3d::Identity() *1000;
    //Pose tmp = { 0.0, 0.0, loomo::PI };
    //graph2.addVirtualEdge(tmp, nodeMap->begin()->first, covar);
    cv::imshow("Graph trajectory w/o final edge", slamUtils::drawGraph(graph2));
    graph2.optimizeGraph();
    graph2.optimizeGraph();
    graph2.optimizeGraph();
    graph2.optimizeGraph();
    graph2.optimizeGraph();
    graph2.optimizeGraph();
    cv::imshow("Graph trajectory w/o final edge, w. post optimization", slamUtils::drawGraph(graph2));

    Eigen::Matrix3d covar = Eigen::Matrix3d::Identity();// *0.1;
    Pose tmp = { 0.0, 0.0, loomo::PI };
    //Pose tmp = { 0.0, 0.0, 0.0 };
    graph1.addVirtualEdge(tmp, nodeMap->begin()->first, covar);
    cv::imshow("Graph trajectory", slamUtils::drawGraph(graph1));
    graph1.optimizeGraph();
    graph1.optimizeGraph();
    graph1.optimizeGraph();
    graph1.optimizeGraph();
    graph1.optimizeGraph();
    graph1.optimizeGraph();
    cv::imshow("Graph trajectory w. post optimization", slamUtils::drawGraph(graph1));

    cout << "\"odo\"-Graph \"miss\": " << graph2.getNodeMap()->find(currentNodeKey)->second.pose.norm() << " mm" << endl;
    cout << "Graph \"miss\": " << graph1.getNodeMap()->find(currentNodeKey)->second.pose.norm() << " mm" << endl;


    cv::imshow("Odometry trajectory", slamUtils::drawOdoTrajectory(odoTraj));
    cv::waitKey(0);
}
