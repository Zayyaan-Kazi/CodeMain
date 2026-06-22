#pragma once
#include "main.h"
#include "odometry.hpp"
namespace Config{
    namespace Defaults { // default values for structs
        namespace moveToPose{
            inline constexpr float turnCompletionDistance = 0.0f; // what point along the line (in) to reach desired heading 
            inline constexpr float scaleToRPM = 0.0f; // RPM the min-max normalization targets
            inline constexpr float captureRadius = 0.0f; // radius (in) from target point to swap to positional PID
            inline constexpr float settleRadius = 0.0f; // radius (in) from target point to consider it reached
            inline constexpr uint32_t timeout = 0.0; // time in miliseconds until movement ends

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
        inline constexpr float wheelCircum = 0.0f; // Input later
        inline constexpr int inertialSign = 1; // incase we need to change to CCW positive
        inline constexpr float linearVelocity = 0.0f; // linear velocity of x drive (in/s) (not accounting for 1.41x increase)
        inline constexpr float robotRadius = 0.0f; // distance between parallel wheel sets (in)        
    }//namespace Chassis

    namespace Odometry{
        inline constexpr Pose initialPose = {0.0f,0.0f,0.0f}; // (X,Y,THETA)
        inline constexpr float distanceRight = 0.0f; // Distance from right sensor to tracking center
        inline constexpr float distanceRear = 0.0f; // Distance from rear sensor to tracking center
        inline constexpr float tolerance = 1e-5f; // tolerance for 0s
    }//namespace Odometry

    namespace velocityControl{
        namespace feedForward{
            inline constexpr float kS = 0.0f; // minimum voltage to overcome friction
            inline constexpr float kV = 0.0f; // voltage to maintain a constant speed
        }//namespace feedforward
        namespace PI{
            inline constexpr float kP = 0.0f; // constant for proportional (distance left)
            inline constexpr float kI = 0.0f; // constant for integral (sum of distances)
            inline constexpr float maxIntegral = 0.0f; // prevents integral from heavily overpowering the motor
            inline constexpr float integrealThreshold = 0.0f; // prevents integral windup on large errors
        }//namespace PI

    }//namespace velocityControl
    namespace positionalController {
        inline constexpr float dt = 0.01f; // time elapsed in seconds between each call, should be 10 or 20ms
        namespace headingPD {
            inline constexpr float kP = 0.0f; // assuming in/s
            inline constexpr float kD = 0.0f; // assuming in/s
        }// namespace headingPD
        namespace X {
            inline constexpr float kP = 0.0f; // assuming in/s
            inline constexpr float kD = 0.0f; // assuming in/s
        }// namespace X
        namespace Y {
            inline constexpr float kP = 0.0f; // assuming in/s
            inline constexpr float kD = 0.0f; // assuming in/s
        }// namespace Y
    }// namespace positionalController
} // namespace Config
