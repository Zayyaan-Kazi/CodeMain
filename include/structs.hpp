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
	float theta;
};
struct motorCommands{
    float fl; // front left
    float fr; // front right
    float rl; // rear left
    float rr; // rear right
};
struct moveToPoseParam{
    float finishTurnBy; // what point along the line (in) to reach desired heading 
    float scaleToRPM; // RPM the min-max normalization targets
    float captureRadius; // radius (in) from target point to swap to positional PID
    float settleRadius; // radius (in) from target point to consider it reached
};
