#pragma once
#include "Config.hpp"
struct velocities{ // stores velocity in the vertical, horizontal, and translational axisesis?
    float xVelocity;
    float yVelocity;
    float thetaVelocity;
}; //this is a stupid name change later -ZK
struct Pose{ //
	float x;
	float y;
	float heading;
};
struct motorCommands{
    float fl = 0; // front left
    float fr = 0; // front right
    float rl = 0; // rear left
    float rr = 0; // rear right
};
struct moveToPoseParam{
    float turnCompletionDistance = Config::Defaults::moveToPose::turnCompletionDistance; // what point along the line (in) to reach desired heading. This function starts at the target point and travels to current pose. Setting it to 0 means it will try complete turn at the same time as reaching point  
    float scaleToRPM; // RPM the min-max normalization targets
    float captureRadius; // radius (in) from target point to swap to positional PD
    float headingCaptureThreshold; // distance (deg) from target theta to swap to heading PD
    float settleRadius; // radius (in) from target point to consider it reached
    float postTurnOmega; // assumed omega (in/s) used to compute translational vector magnitude when turn is being managed by PD
    uint32_t timeout; // time (miliseconds) before move is considered done
};
struct PoseError {
    float deltaX;
    float deltaY;
    float deltaDeg;
    float distanceToTarget;
    bool usingHeadingPD; // is turn being managed by PD
};
