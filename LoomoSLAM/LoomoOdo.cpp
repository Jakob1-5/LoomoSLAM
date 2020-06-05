#include "LoomoOdo.h"

using namespace loomo;
using namespace Eigen;


void LoomoOdo::incrementPose(int dTickL, int dTickR)
{
    double dsL = ((double)dTickL / TICK_PER_REV)*PI*WHEEL_DIAMETER;
    double dsR = ((double)dTickR / TICK_PER_REV)*PI*WHEEL_DIAMETER;

    double ds = (dsL + dsR) / 2;

    double dTheta = (dsR - dsL) / BASE_WIDTH;

    double dx = ds * cos(pose_.theta + dTheta / 2);
    double dy = ds * sin(pose_.theta + dTheta / 2);

    Matrix3d Fx = jacX(dsL, dsR, dTheta, pose_.theta);
    Matrix<double, 3, 2> Fu = jacU(dsL, dsR, dTheta, pose_.theta);
    Matrix2d Q = q(dsL, dsR);
    uncertainty_ = Fx * uncertainty_*Fx.transpose() + Fu * Q*Fu.transpose();

    increment_ = {dx, dy, dTheta};
    pose_ += increment_;
}


Pose loomo::LoomoOdo::getIncrement()
{
    return increment_;
}

Eigen::Matrix3d loomo::LoomoOdo::getUncertainty()
{
    return uncertainty_;
}

void loomo::LoomoOdo::resetUncertainty()
{
    uncertainty_ = Eigen::Matrix3d::Zero();
}

Pose loomo::LoomoOdo::getPose()
{
    return pose_;
}

void loomo::LoomoOdo::setPose(Pose pose)
{
    pose_ = pose;
}

void loomo::LoomoOdo::reset()
{
    pose_ = { 0.0, 0.0, 0.0 };
    //pose_.x = 0.0;
    //pose_.y = 0.0;
    uncertainty_ = Matrix3d::Zero();
}

Eigen::Matrix3d loomo::LoomoOdo::jacX(double dsL, double dsR, double dTheta, double theta)
{
    Matrix3d jac = Matrix3d::Identity();
    double a = theta - dTheta / 2;
    double b = (dsL + dsR) / 2;
    jac(0, 2) = -sin(a)*b;
    jac(1, 2) = cos(a)*b;
    return jac;
}

Eigen::Matrix<double, 3, 2> loomo::LoomoOdo::jacU(double dsL, double dsR, double dTheta, double theta)
{
    Matrix<double, 3, 2> jac;
    double a = theta - dTheta / 2;
    double b = (dsL + dsR) / (4 * BASE_WIDTH);
    jac(0, 0) = 0.5*cos(a) + b * sin(a);
    jac(1, 0) = 0.5*sin(a) - b * cos(a);
    jac(2, 0) = -1 / BASE_WIDTH;
    jac(0, 1) = 0.5*cos(a) - b * sin(a);
    jac(1, 1) = 0.5*sin(a) + b * cos(a);
    jac(2, 1) = 1 / BASE_WIDTH;
    return jac;
}

Eigen::Matrix2d loomo::LoomoOdo::q(double dsL, double dsR)
{
    Matrix2d q;
    q <<
        K_R * abs(dsR), 0,
        0, K_L * abs(dsL);
    return q;
}

double loomo::normDist(Pose pose1, Pose pose2)
{
    return sqrt(pow((pose1.x - pose2.x), 2) + pow((pose1.y - pose2.y), 2));
}
