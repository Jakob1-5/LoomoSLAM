#pragma once

#include <cmath>
#include <Eigen/Dense>


namespace loomo {

    struct Pose {
        double x;
        double y;
        double theta;
        Pose operator+(Pose rhs) {
            return { this->x + rhs.x, this->y + rhs.y, this->theta + rhs.theta };
        }
        Pose operator+=(Pose dPose) {
            return { this->x += dPose.x, this->y += dPose.y, this->theta += dPose.theta };
        }
        Pose operator-(Pose rhs) {
            return { this->x - rhs.x, this->y - rhs.y, this->theta - rhs.theta };
        }
        double norm() {
            return sqrt(pow(this->x, 2) + pow(this->y, 2));
        }
    };

    double normDist(Pose pose1, Pose pose2);


    static const double BASE_WIDTH = 488.570; // [mm]
    static const double WHEEL_DIAMETER = 269.17; // [mm]
    static const double TICK_PER_REV = 90.0;
    static const double PI = acos(-1.0);
    static const double K_R = 170.0;
    static const double K_L = 170.0;

    class LoomoOdo
    {
    public:
        LoomoOdo(Pose initialPose = { 0.0, 0.0, 0.0 }):
            pose_(initialPose){}

        void incrementPose(int dTickL, int dTickR);
        Pose getIncrement();
        Eigen::Matrix3d getUncertainty();
        Pose getPose();
        void reset();

        void resetUncertainty();
        void setPose(Pose pose);

    private:
        Pose pose_;
        Pose increment_;
        Eigen::Matrix3d uncertainty_ = Eigen::Matrix3d::Zero();

        Eigen::Matrix3d jacX(double dsL, double dsR, double dTheta, double theta);
        Eigen::Matrix<double, 3, 2> jacU(double dsL, double dsR, double dTheta, double theta);
        Eigen::Matrix2d q(double dsL, double dsR);
    };

}
