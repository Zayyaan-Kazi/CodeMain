#pragma once
#include "main.h"
#include "odometry.hpp"
namespace Config{
    namespace Defaults {
        namespace moveToPose{
            inline constexpr float turnCompletionDistance = 0; // what point along the line (in) to reach desired heading 
            inline constexpr float scaleToRPM = 0; // RPM the min-max normalization targets
            inline constexpr float captureRadius = 0; // radius (in) from target point to swap to positional PID
            inline constexpr float settleRadius = 0; // radius (in) from target point to consider it reached
            inline constexpr float timeout;

        }//namespace moveToPose
    }// namespace defaultStructValues
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
        inline constexpr float linearVelocity; // linear velocity of x drive (in/s) (not accounting for 1.41x increase)
        inline constexpr float robotRadius; // distance between parallel wheel sets (in)        
    }//namespace Chassis

    namespace Odometry{
        inline constexpr Pose initialPose = {0,0,0}; // (X,Y,THETA)
        inline constexpr float distanceRight = 0; // Distance from right sensor to tracking center
        inline constexpr float distanceRear = 0; // Distance from rear sensor to tracking center
        inline constexpr float tolerance = 1e-5; // tolerance for 0s
    }//namespace Odometry

    namespace velocityControl{
        namespace feedForward{
            inline constexpr float kS = 0; // minimum voltage to overcome friction
            inline constexpr float kV = 0; // voltage to maintain a constant speed
        }//namespace feedforward
        namespace PI{
            inline constexpr float kP = 0; // constant for proportional (distance left)
            inline constexpr float kI = 0; // constant for integral (sum of distances)
            inline constexpr float maxIntegral = 0; // prevents integral from heavily overpowering the motor
            inline constexpr float integrealThreshold = 0; // prevents integral windup on large errors
        }//namespace PI

    }//namespace velocityControl

} // namespace Config
