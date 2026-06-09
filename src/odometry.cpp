#include "odometry.hpp"
#include "chassis.hpp"
#include "config.hpp"
#include <cmath>
odometry::odometry(Chassis* _mainChassis) : mainChassis(_mainChassis){}

void odometry::updateOdometry(){
    
    float headingDelta = mainChassis->getHeadingDelta();
    Pose localOffset; // create temporary pose to adjust without affecting the main one
    float headingDeltaRAD = headingDelta * 3.14159 / 180.0; // convert to radians
    auto distanceDeltas = mainChassis->getRotationDeltas(); // first is the right delta and second is the rear delta
    if(fabsf(headingDelta) > tolerance){
        
        localOffset.x = 2.0*sinf(headingDeltaRAD/2.0)* ((distanceDeltas.first / headingDeltaRAD) + disR); // X(forward)
        localOffset.y = 2.0*sinf(headingDeltaRAD/2.0)* ((distanceDeltas.second / headingDeltaRAD) + disB); // X(forward)

    }else{
        return; // skip if rotation change is 0, this may need to be updated as we now using a holonomic
    }
    //Rotate to global via Rotation Matrix (https://en.wikipedia.org/wiki/Rotation_matrix)
    Pose localOffsetRotated;
    float avgHeading = (globalPose.theta * 3.14159 / 180.0f) + headingDeltaRAD / 2.0f; // rotate by midpoint of heading for a more accurate change in heading
    localOffsetRotated.x = localOffset.x * cosf(avgHeading) - localOffset.y * sinf(avgHeading);
    localOffsetRotated.y = localOffset.x * sinf(avgHeading) + localOffset.y * cosf(avgHeading);
    
    globalPose.x += localOffsetRotated.x;
    globalPose.y += localOffsetRotated.y;
    globalPose.theta = mainChassis->getHeading();
}

void odometry::setPose(Pose newPose){
    globalPose = newPose;
}

Pose odometry::getPose(){
    return globalPose;
}