#pragma once
#include "main.h"
#include "odometry.hpp"
namespace Config{
    //Drivetrain Motor ports
    inline constexpr int8_t frontLeftPort = 1;
    inline constexpr int8_t frontRightPort = 2;
    inline constexpr int8_t backLeftPort = 3;
    inline constexpr int8_t backRightPort = 4;
    inline constexpr int8_t middleLeftPort = 5;
    inline constexpr int8_t middleRightPort = 6;

    //Sensor ports
    inline constexpr int8_t IMUport = 7;
    inline constexpr int8_t rotationRightPort = 8;
    inline constexpr int8_t rotationLeftPort = 9;

    //Constants
    inline constexpr float wheelCircum = 0; // Input later
    inline constexpr int inertialSign = -1;

    //Odometry
    inline constexpr Pose initialPose = {0,0,0}; // (X,Y,THETA)
    inline constexpr float distanceRight = 0; // Distance from right sensor and tracking center
    inline constexpr float distanceLeft = 0; // Distance from left sensor and tracking center
    inline constexpr float tolerance = 1e-5; // tolerance for 0s
} // namespace Config
