#include "main.h"
#include "motionProfiling.hpp"

motionProfiling::motionProfiling() {}
//TODO: implement control for when turning is managed by PD
velocities motionProfiling::pointToTrajectory(Pose currentPose, Pose targetPose, float turnCompletionDistance){
    //ensure all degrees are converted to radians
    float curHeadingRAD = RoboMath::degToRad(currentPose.theta);
    float targetThetaRAD = RoboMath::degToRad(targetPose.theta);

    //deltas
    float deltaX = targetPose.x - currentPose.x;
    float deltaY = targetPose.y - currentPose.y;
    float deltaThetaRAD = RoboMath::wrapRadian(targetThetaRAD - curHeadingRAD); // ensure radian is between [-pi, pi] for shortest path

    float targetDistance = RoboMath::distance(deltaX,deltaY);
    float turnDistance = targetDistance - turnCompletionDistance;
    float omegaRAD = (linearVelocity*deltaThetaRAD) / turnDistance;  // Turning speed in rad/s
    float omega = robotRadius* fabsf(omegaRAD); // this is the turning speed in in/s, we can use wheel travel to determine RPM

    float vectorMagnitude =  linearVelocity - omega; //hypotenuse of our velocity vector
    float vectorTheta = atan2f(deltaY,deltaX);

    float localTheta = vectorTheta - curHeadingRAD; // get us within local frame of robot
    // when debugging: if velocityX and velocityY have weird behavior try global frame
    float velocityX = vectorMagnitude*cosf(localTheta); // Legs of triangle
    float velocityY = vectorMagnitude*sinf(localTheta);

    velocities returnVelocity = {velocityX,velocityY,omega};
    return returnVelocity;

}

motorCommands motionProfiling::velocityToMotor(velocities velocityRPM){ 
    motorCommands rawCommandTrajectory;
    rawCommandTrajectory.fl = (velocityRPM.yVelocity + velocityRPM.xVelocity) + velocityRPM.thetaVelocity; //X drive motor matrix
    rawCommandTrajectory.fr = (velocityRPM.yVelocity - velocityRPM.xVelocity) - velocityRPM.thetaVelocity;
    rawCommandTrajectory.rl = (velocityRPM.yVelocity - velocityRPM.xVelocity) + velocityRPM.thetaVelocity;
    rawCommandTrajectory.rr = (velocityRPM.yVelocity + velocityRPM.xVelocity) - velocityRPM.thetaVelocity;
    return rawCommandTrajectory;
}

// To anyone reading my code, if you want to know how the math below works search up min max normalization   
motorCommands motionProfiling::scaleRPM(motorCommands rawCommands, float targetRPM){
    float highestMotorRPM = std::max({fabsf(rawCommands.fl),
                                fabsf(rawCommands.fr),
                                fabsf(rawCommands.rl),
                                fabsf(rawCommands.rr)}); // want highest point of distribution to be targetRPM
    
    if (highestMotorRPM < 1e-6) return rawCommands; // Division by zero is a real thing, could happen y'know.

    float scale = targetRPM / highestMotorRPM; 

    return {
        rawCommands.fl * scale,
        rawCommands.fr * scale,
        rawCommands.rl * scale,
        rawCommands.rr * scale
    };
}
