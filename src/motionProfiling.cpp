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
        omega = movementParams.postTurnOmega; // assume low omega value since we're basically already at our desired heading
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
    
    if (highestMotorRPM < 1e-6f) return rawCommands; // Division by zero is a real thing, could happen y'know.

    float scale = targetRPM / highestMotorRPM; 

    return {
        rawCommands.fl * scale,
        rawCommands.fr * scale,
        rawCommands.rl * scale,
        rawCommands.rr * scale
    };
}
motorCommands motionProfiling::targetToMotorRPM(Pose targetPose,Pose currentPose, moveToPoseParam movementParams){
    if (firstCall){
        firstCall = false;
        startTime = pros::millis();
    }
    PoseError deltaPose;
    // pre calculate deltas
    deltaPose.deltaX = targetPose.x - currentPose.x;
    deltaPose.deltaY = targetPose.y - currentPose.y;
    deltaPose.deltaDeg = RoboMath::subDegrees(targetPose.heading,currentPose.heading);    
    deltaPose.distanceToTarget = RoboMath::distance(deltaPose.deltaX,deltaPose.deltaY);
    
    if( (deltaPose.distanceToTarget < movementParams.settleRadius) || (pros::millis() - startTime > movementParams.timeout)){
        settled = true;
        return {}; //return default motorCommands value (hardcoded all 4 at 0)
    }
    //threshold checks
    bool useHeadingPD = fabsf(deltaPose.deltaDeg) < movementParams.headingCaptureThreshold;
    bool usePosPD = deltaPose.distanceToTarget < movementParams.captureRadius;
    velocities targetVelocities;
    if (usePosPD){
        // rotate all to local frame
        float curHeadingRAD = RoboMath::degToRad(currentPose.heading);                                          // note: once i test this on an actual x drive and ensure that the frame is proper
        float localDeltaX = deltaPose.deltaX * cosf(curHeadingRAD) + deltaPose.deltaY * sinf(curHeadingRAD);    // I can have these calculations done before and so they're either used here or in
        float localDeltaY = -deltaPose.deltaX * sinf(curHeadingRAD) + deltaPose.deltaY * cosf(curHeadingRAD);   // pointToTrajectory, since i am unsure i will test and see.

        //calculate values from PD which will be scaled at in/s
        targetVelocities.xVelocity = positionalPDX.update(localDeltaX, Config::positionalController::dt);
        targetVelocities.yVelocity = positionalPDY.update(localDeltaY, Config::positionalController::dt);
        targetVelocities.thetaVelocity = headingPD.update(deltaPose.deltaDeg, Config::positionalController::dt);
        //convert from in/s to rpm using wheel travel using evil code
        velocities targetVelocitiesRPM = RoboMath::velocitiesToRPM(targetVelocities);

        //convert to motor commands
        motorCommands rawMotorCommands = velocityToMotor(targetVelocitiesRPM);
        
        //Do not scale to target RPM because we are trying to correct for tiny amounts of error
        return rawMotorCommands;
    }else{
        targetVelocities = pointToTrajectory(currentPose,targetPose,deltaPose,movementParams);
        if(useHeadingPD){
            targetVelocities.thetaVelocity = headingPD.update(deltaPose.deltaDeg, Config::positionalController::dt);
        }
        //convert from in/s to rpm using wheel travel using evil code
        velocities targetVelocitiesRPM = RoboMath::velocitiesToRPM(targetVelocities);

        //convert to motor commands
        motorCommands rawMotorCommands = velocityToMotor(targetVelocitiesRPM);
        //scale motor commands
        motorCommands scaledMotorCommands = scaleRPM(rawMotorCommands, movementParams.scaleToRPM);
        return scaledMotorCommands;
    }
        
    
}

bool motionProfiling::isSettled() {return settled;}

void motionProfiling::reset() {
    settled = false;
    firstCall = true;
    startTime = 0.0f;
    headingPD.reset();
    positionalPDX.reset();
    positionalPDY.reset();
}