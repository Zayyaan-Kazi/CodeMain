#pragma once
#include "main.h"
#include "odometry.hpp"
namespace Config{
    namespace Ports {
        //Drivetrain Motor ports
        inline constexpr int8_t frontLeft = 1;
        inline constexpr int8_t frontRight = 2;
        inline constexpr int8_t rearLeft = 3;
        inline constexpr int8_t rearRight = 4;
        inline constexpr int8_t middleLeft = 5;
        inline constexpr int8_t middleRight = 6;
        
        //Sensor ports
        inline constexpr int8_t IMU = 7;
        inline constexpr int8_t rightRotation = 8;
        inline constexpr int8_t rearRotation = 9;
    }// namespace ports
        
    namespace Chassis{
        inline constexpr float wheelCircum = 0; // Input later
        inline constexpr int inertialSign = 1; // incase we need to change to CCW positive
    }//namespace Chassis

    namespace Odometry{
        inline constexpr Pose initialPose = {0,0,0}; // (X,Y,THETA)
        inline constexpr float distanceRight = 0; // Distance from right sensor to tracking center
        inline constexpr float distanceRear = 0; // Distance from rear sensor to tracking center
        inline constexpr float tolerance = 1e-5; // tolerance for 0s
    }//namespace Odometry
} // namespace Config
