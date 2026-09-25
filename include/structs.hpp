#pragma once
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
// moveToPoseParam is now located under motion
struct PoseError {
    float deltaX;
    float deltaY;
    float deltaDeg;
    float distanceToTarget;
    bool usingHeadingPD; // is turn being managed by PD
};
struct rotationDeltas{
    float vertical;
    float horizontal;
};
