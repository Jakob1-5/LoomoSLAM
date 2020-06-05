#include "PoseGraph.h"

using namespace loomo;
using namespace poseGraph;
using namespace Eigen;


poseGraph::PoseGraph::PoseGraph(uint32_t id, loomo::Pose initialPose)
    :id_(id)
{
    nodeCounter_ = ((uint64_t)id_ << 32) | 0;

    Node initialNode;
    Edge initialEdge;
    initialEdge.constraint = initialPose;
    initialEdge.covariance <<
        1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    initialEdge.to = nodeCounter_;
    initialEdge.from = nodeCounter_;
    initialNode.pose = initialPose;
    //initialNode.edges.push_back(initialEdge);
    nodeMap_.insert({ nodeCounter_, initialNode });
}

uint64_t PoseGraph::addNodeFromOdo(loomo::Pose constraint, Eigen::Matrix3d covariance)
{
    Edge newEdge;
    newEdge.constraint = constraint;
    newEdge.covariance = covariance;

    newEdge.from = nodeCounter_;
    ++nodeCounter_;


    Pose prevPose = (nodeMap_.find(newEdge.from))->second.pose;
    double theta = prevPose.theta;
    Eigen::Vector3d tmp1(prevPose.x, prevPose.y, theta);
    Eigen::Vector3d tmp2(constraint.x, constraint.y, constraint.theta);
    Eigen::Matrix3d rot;
    rot << cos(theta), -sin(theta), 0,
        sin(theta), cos(theta), 0,
        0, 0, 1;
    tmp2 = rot * tmp2;
    tmp2 += tmp1;

    Node newNode;
    newNode.idx = nodeCounter_;
    newNode.pose = { tmp2(0), tmp2(1), tmp2(2) };
    newEdge.to = nodeCounter_;

    newNode.edges.push_back(newEdge);

    // Obs: will override if there already is element at newNode.idx (not expected to happen, but might be smart with a sanity check)
    nodeMap_[newNode.idx] = newNode; 

    return newNode.idx;
}

uint64_t poseGraph::PoseGraph::addVirtualEdge(loomo::Pose constraint, uint64_t fromNode, Eigen::Matrix3d covariance)
{
    Edge newEdge;
    newEdge.constraint = constraint;
    newEdge.covariance = covariance;
    //<<
    //    kXCV * constraint.x, 0, 0,
    //    0, kYCV*constraint.y, 0,
    //    0, 0, kThetaCV*constraint.theta;

    newEdge.from = fromNode;
    newEdge.to = nodeCounter_;
    (nodeMap_.find(nodeCounter_))->second.edges.push_back(newEdge);

    return nodeCounter_;
}

std::map<uint64_t, Node>* poseGraph::PoseGraph::getNodeMap()
{
    return &nodeMap_;
}


std::list<Edge> PoseGraph::getEdges()
{
    std::list<Edge> edges;
    for (const auto & node : *getNodeMap()) {
        for (const auto & edge : node.second.edges) {
            edges.push_back(edge);
        }
    }
    return edges;
}

void poseGraph::PoseGraph::optimizeGraph()
{
    
    const size_t n = nodeMap_.size();
    VectorXd b = VectorXd::Zero(3*n);

    typedef Eigen::Triplet<double> T;
    std::vector<T> coeff;

    uint32_t j = 0;
    for (auto it = nodeMap_.begin(); it != nodeMap_.end(); it++) {
    //for (std::_Tree<std::_Tmap_traits<uint64_t, poseGraph::Node, std::less<uint64_t>, std::allocator<std::pair<const uint64_t, poseGraph::Node>>, false>>::iterator it = nodeMap_.begin(); it != nodeMap_.end(); it++) {
        Node node_j = it->second;
        for (Edge edge : node_j.edges) {
            uint64_t fromNode = edge.from;
            uint32_t i = (uint32_t)(fromNode) * 3; // OBS/TODO, account for potential duplicates
            Pose z_ij = edge.constraint;
            Matrix3d info = edge.covariance.inverse();
            Node node_i = nodeMap_.find(edge.from)->second;
            if (nodeMap_.find(edge.from) == nodeMap_.end())
            {
                std::cout << "Did not find node index " << edge.from << " (number " << (uint32_t)(edge.from) << ")" << std::endl;
                continue;
            }
            Vector3d error = err(node_i.pose, node_j.pose, z_ij);
            Matrix3d A = jacAij(node_i.pose, node_j.pose, z_ij);
            Matrix3d B = jacBij(node_i.pose, node_j.pose, z_ij);

            b.block(i, 0, 3, 1) += A.transpose() * info * error;
            b.block(j, 0, 3, 1) += B.transpose() * info * error; // b_i^T = e^T * Omega * B

            Matrix3d Hii = A.transpose() * info * A;
            Matrix3d Hij = A.transpose() * info * B;
            Matrix3d Hji = B.transpose() * info * A;
            Matrix3d Hjj = B.transpose() * info * B;
            for (int rowOffset = 0; rowOffset < 3; ++rowOffset) {
                for (int colOffset = 0; colOffset < 3; ++colOffset) {
                    coeff.push_back(T(i+rowOffset, i+colOffset, Hii(rowOffset, colOffset)));
                    coeff.push_back(T(i+rowOffset, j+colOffset, Hij(rowOffset, colOffset)));
                    coeff.push_back(T(j+rowOffset, i+colOffset, Hji(rowOffset, colOffset)));
                    coeff.push_back(T(j+rowOffset, j+colOffset, Hjj(rowOffset, colOffset)));
                }
            }

        }
        j += 3;
    }

    // Keep the first node fixed
    coeff.push_back(T(0, 0, 1.0));
    coeff.push_back(T(1, 1, 1.0));
    coeff.push_back(T(2, 2, 1.0));

    SparseMatrix<double> H(3*n, 3*n);
    H.setFromTriplets(coeff.begin(), coeff.end());


    // Solving H*deltaX = -b
    b *= -1;
    SimplicialCholesky<SparseMatrix<double>> chol(H);  // performs a Cholesky factorization (LLT) of H
    VectorXd deltaX = chol.solve(b);         // use the factorization to solve for the given right hand side

    //std::cout << deltaX << std::endl;

    int i = 0;
    for (auto it = nodeMap_.begin(); it != nodeMap_.end(); it++) {
        //if (it == nodeMap_.begin()) {
        //    i += 3;
        //    continue;
        //}
        it->second.pose.x += deltaX(i++);
        it->second.pose.y += deltaX(i++);
        it->second.pose.theta += deltaX(i++);
    }
}


Eigen::Matrix3d poseGraph::PoseGraph::jacAij(loomo::Pose pose_i, loomo::Pose pose_j, loomo::Pose pose_ij)
{
    Matrix3d mAij;
    mAij <<
        sin(pose_i.theta)*sin(pose_ij.theta) - cos(pose_i.theta)*cos(pose_ij.theta), -cos(pose_i.theta)*sin(pose_ij.theta) - cos(pose_ij.theta)*sin(pose_i.theta), sin(pose_ij.theta)*(cos(pose_i.theta)*(pose_i.x - pose_j.x) + sin(pose_i.theta)*(pose_i.y - pose_j.y)) - cos(pose_ij.theta)*(cos(pose_i.theta)*(pose_i.y - pose_j.y) - sin(pose_i.theta)*(pose_i.x - pose_j.x)),
        cos(pose_i.theta)*sin(pose_ij.theta) + cos(pose_ij.theta)*sin(pose_i.theta), sin(pose_i.theta)*sin(pose_ij.theta) - cos(pose_i.theta)*cos(pose_ij.theta), cos(pose_ij.theta)*(cos(pose_i.theta)*(pose_i.x - pose_j.x) + sin(pose_i.theta)*(pose_i.y - pose_j.y)) + sin(pose_ij.theta)*(cos(pose_i.theta)*(pose_i.y - pose_j.y) - sin(pose_i.theta)*(pose_i.x - pose_j.x)),
        0, 0, -1;
    return mAij;
}

Eigen::Matrix3d poseGraph::PoseGraph::jacBij(loomo::Pose pose_i, loomo::Pose pose_j, loomo::Pose pose_ij)
{
    Matrix3d mBij;
    mBij <<
        cos(pose_i.theta)*cos(pose_ij.theta) - sin(pose_i.theta)*sin(pose_ij.theta), cos(pose_i.theta)*sin(pose_ij.theta) + cos(pose_ij.theta)*sin(pose_i.theta), 0,
        - cos(pose_i.theta)*sin(pose_ij.theta) - cos(pose_ij.theta)*sin(pose_i.theta), cos(pose_i.theta)*cos(pose_ij.theta) - sin(pose_i.theta)*sin(pose_ij.theta), 0,
        0, 0, 1;
    return mBij;
}

Eigen::Vector3d poseGraph::PoseGraph::err(loomo::Pose pose_i, loomo::Pose pose_j, loomo::Pose pose_ij)
{
    Vector3d err;
    err <<
        -pose_ij.x - cos(pose_ij.theta)*(cos(pose_i.theta)*(pose_i.x - pose_j.x) + sin(pose_i.theta)*(pose_i.y - pose_j.y)) - sin(pose_ij.theta)*(cos(pose_i.theta)*(pose_i.y - pose_j.y) - sin(pose_i.theta)*(pose_i.x - pose_j.x)),
        sin(pose_ij.theta)*(cos(pose_i.theta)*(pose_i.x - pose_j.x) + sin(pose_i.theta)*(pose_i.y - pose_j.y)) - cos(pose_ij.theta)*(cos(pose_i.theta)*(pose_i.y - pose_j.y) - sin(pose_i.theta)*(pose_i.x - pose_j.x)) - pose_ij.y,
        pose_j.theta - pose_i.theta - pose_ij.theta;
    return err;
}
