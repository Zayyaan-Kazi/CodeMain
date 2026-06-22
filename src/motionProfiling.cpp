#include "main.h"
#include "motionProfiling.hpp"

motionProfiling::motionProfiling() {}
//TODO: implement control for when turning is managed by PD
velocities motionProfiling::pointToTrajectory(Pose currentPose, Pose targetPose,PoseError deltaPose, moveToPoseParam movementParams){
    //ensure all degrees are converted to radians
    float curHeadingRAD = RoboMath::degToRad(currentPose.heading); 
    float deltaThetaRAD = RoboMath::wrapRadian(RoboMath::degToRad(deltaPose.deltaDeg));

    float omega; // defined here to keep scope
    if(deltaPose.usingHeadingPD){
        omega = movementParams.postTurnOmega;
    }
    else{
        float turnDistance = deltaPose.distanceToTarget - movementParams.turnCompletionDistance;
        float omegaRAD = (linearVelocity*deltaThetaRAD) / turnDistance;  // Turning speed in rad/s
        omega = robotRadius* fabsf(omegaRAD); // this is the turning speed in in/s, we can use wheel travel to determine RPM
    }
    

    float vectorMagnitude =  linearVelocity - omega; //hypotenuse of our velocity vector
    float vectorTheta = atan2f(deltaPose.deltaX,deltaPose.deltaY);

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
motorCommands motionProfiling::targetToMotorRPM(Pose targetPose,Pose currentPose, moveToPoseParam movementParams){
    PoseError deltaPose;
    // pre calculate deltas
    deltaPose.deltaX = targetPose.x - currentPose.x;
    deltaPose.deltaY = targetPose.y - currentPose.y;
    deltaPose.deltaDeg = RoboMath::subDegrees(targetPose.heading,currentPose.heading);    
    deltaPose.distanceToTarget = RoboMath::distance(deltaPose.deltaX,deltaPose.deltaY);

    //threshold checks
    bool useHeadingPD = fabsf(deltaPose.deltaDeg) < movementParams.headingCaptureThreshold;
    bool usePosPD = deltaPose.distanceToTarget < movementParams.captureRadius;

    if(useHeadingPD){
        // Implement code for headingPD
    }
    velocities targetVelocities = pointToTrajectory(currentPose,targetPose,deltaPose,movementParams);
    //convert from in/s to rpm using wheel travel using evil code
    velocities targetVelocitiesRPM = RoboMath::velocitiesToRPM(targetVelocities);
    //convert to motor commands
    motorCommands rawMotorCommands = velocityToMotor(targetVelocitiesRPM);
    //scale motor commands
    motorCommands scaledMotorCommands = scaleRPM(rawMotorCommands, movementParams.scaleToRPM);

    //create loop using velocity controller that exits when close enough
    
    //swap to positional pid and do that exiting when done

    //uhh return unless i forgot something
}