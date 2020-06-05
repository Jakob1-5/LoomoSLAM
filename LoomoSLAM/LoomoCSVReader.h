#pragma once


#include <string>
#include <vector>
#include <list>
#include <fstream>
#include <iterator>
#include <algorithm>
#include <iostream>
#include <time.h>
#include <cstdint>

enum dataColumn {
    TIME = 0,               // [ms] must be read as uint_64
    IR_LEFT = 1,            // [mm] 
    IR_RIGHT = 2,           // [mm]
    ULTRASONIC = 3,         // [mm]
    POSE_X = 4,             // [m]
    POSE_Y = 5,             // [m]
    POSE_THETA = 6,         // [rad]
    POSE_LIN_VEL = 7,       // [m/s]
    POSE_ANG_VEL = 8,       // [rad/s]
    TICK_LEFT = 9,          // [tics] (90 tics/rev)
    TICK_RIGHT = 10,        // [tics]
    WHEELSPEED_LEFT = 11,   // [rad/s]?
    WHEELSPEED_RIGHT = 12,  // [rad/s]?
    // some more that are left out for now
    HEAD_YAW = 18,          // [rad]
    IMU_ROLL = 19,          // [rad]
    IMU_PITCH = 20,         // [rad]
    IMU_YAW = 21,           // [rad]
    FISHEYETF_TX,           // [m]
    FISHEYE_TF_TY,          // [m]
    FISHEYE_TF_TZ,          // [m]
    FISHEYE_TF_ROLL,        // [rad]
    FISHEYE_TF_PITCH,       // [rad]
    FISHEYE_TF_YAW,         // [rad]
    COLOR_TF_TX,            // [m]
    COLOR_TF_TY,            // [m]
    COLOR_TF_TZ,            // [m]
    COLOR_TF_ROLL,          // [rad]
    COLOR_TF_PITCH,         // [rad]
    COLOR_TF_YAW,           // [rad]
    DEPTH_TF_TX,            // [m]
    DEPTH_TF_TY,            // [m]
    DEPTH_TF_TZ,            // [m]
    DEPTH_TF_ROLL,          // [rad]
    DEPTH_TF_PITCH,         // [rad]
    DEPTH_TF_YAW,           // [rad]
    FISHEYE_IDX,            // [ms], but from a different clock than TIME. Also in uint_64 format
    COLOR_IDX,              // [ms], but from a different clock than TIME. Also in uint_64 format
    DEPTH_IDX               // [ms], but from a different clock than TIME. Also in uint_64 format
};


struct CSVData
{
    int64_t timestamp;
    int ir_left;
    int ir_right;
    int ultrasonic;
    double pose_x;
    double pose_y;
    double pose_theta;
    double pose_lin_vel;
    double pose_ang_vel;
    int tick_left;
    int tick_right;
    double imu_roll;
    double imu_pitch;
    double imu_yaw;
    int64_t fisheye_idx;
    int64_t color_idx;
    int64_t depth_idx;
};

class LoomoCSVReader
{
public:
    static std::list<std::vector<std::string>> getData(std::string fileName);
    static std::vector<CSVData> string2data(std::list<std::vector<std::string>> csvStrings);
};