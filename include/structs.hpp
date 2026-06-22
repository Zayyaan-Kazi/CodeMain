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
    float fl; // front left
    float fr; // front right
    float rl; // rear left
    float rr; // rear right
};
struct moveToPoseParam{
    float turnCompletionDistance = Config::Defaults::moveToPose::turnCompletionDistance; // what point along the line (in) to reach desired heading 
    float scaleToRPM; // RPM the min-max normalization targets
    float captureRadius; // radius (in) from target point to swap to positional PD
    float headingCaptureThreshold; // distance (deg) from target theta to swap to heading PD
    float settleRadius; // radius (in) from target point to consider it reached
    float timeout; // time (sec) before move is considered done
    float postTurnOmega; // assumed omega (in/s) used to compute translational vector magnitude when turn is being managed by PD
};
struct PoseError {
    float deltaX;
    float deltaY;
    float deltaDeg;
    float distanceToTarget;
    bool usingHeadingPD; // is turn being managed by PD
};
