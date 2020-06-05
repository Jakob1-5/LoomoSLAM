#pragma once

#include <list>
#include <map>
#include <cmath>
#include <cstdint>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "LoomoOdo.h"


namespace poseGraph {


    struct Edge
    {
        loomo::Pose constraint;
        Eigen::Matrix3d covariance;
        uint64_t to, from;
    };

    struct Node
    {
        loomo::Pose pose;
        std::list<Edge> edges;
        uint64_t idx;
        std::vector<cv::KeyPoint> keyPoints;
        cv::Mat descriptors;
    };


    // Constants (consider using capital letters and snake_case instead)
    static const double kXOdo = 10.0;
    static const double kYOdo = 10.0;
    static const double kThetaOdo = 0.8;
    static const double kXCV = 1.0;
    static const double kYCV = 1.0;
    static const double kThetaCV = 1.0;// kThetaOdo / 5.0;


    class PoseGraph
    {
        std::map<uint64_t, Node> nodeMap_;

        uint32_t id_;
        uint64_t nodeCounter_ = 0;

        Eigen::Matrix3d jacAij(loomo::Pose pose_i, loomo::Pose pose_j, loomo::Pose pose_ij);
        Eigen::Matrix3d jacBij(loomo::Pose pose_i, loomo::Pose pose_j, loomo::Pose pose_ij);
        Eigen::Vector3d err(loomo::Pose pose_i, loomo::Pose pose_j, loomo::Pose pose_ij);

    public:
        PoseGraph(uint32_t id = 0, loomo::Pose initialPose = { 0.0, 0.0, 0.0 });
        


        uint64_t addNodeFromOdo(loomo::Pose constraint, Eigen::Matrix3d covariance);
        uint64_t addVirtualEdge(loomo::Pose constraint, uint64_t fromNode, Eigen::Matrix3d covariance);


        std::map<uint64_t, Node> * getNodeMap();
        std::list<Edge> getEdges(); // Not very useful. Instead, acces edges through the nodes

        void optimizeGraph();
    };
}


