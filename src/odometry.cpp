#include "odometry.hpp"
#include "chassis.hpp"
#include "config.hpp"
#include <cmath>
odometry::odometry(Chassis* _mainChassis) : mainChassis(_mainChassis){}

void odometry::updatePosition(){
    
    float headingDelta = mainChassis->getHeadingDelta();
    Pose localOffset; // create temporary pose to adjust without affecting the main one
    float headingDeltaRAD = headingDelta * 3.14159 / 180.0; // convert to radians
    auto distanceDeltas = mainChassis->getRotationDeltas(); // first is the right delta and second is the rear delta
    
    //Optimization caching, i don't want to mess with the compiler flags because they are scary - Kazi
    float arcChordFactor =  2.0*sinf(headingDeltaRAD/2.0); // compute once and cache
    float invHeadingDeltaRAD =  1.0f / headingDeltaRAD; // multiplication is faster than division on CPU
    
    if(fabsf(headingDelta) > tolerance){
        localOffset.x = arcChordFactor * ((distanceDeltas.first * invHeadingDeltaRAD) + disR); 
        localOffset.y = arcChordFactor * ((distanceDeltas.second * invHeadingDeltaRAD) + disB);
    }else{
        return; // skip if rotation change is 0, this may need to be updated as we now using a holonomic
    }
    
    //Rotate to global via Rotation Matrix (https://en.wikipedia.org/wiki/Rotation_matrix)
    Pose localOffsetRotated;
    float avgHeading = (globalPose.theta * 3.14159 / 180.0f) + headingDeltaRAD / 2.0f; // rotate by midpoint of heading for a more accurate change in heading
    
    float cosAvgHeading = cosf(avgHeading); //compute once and cache
    float sinAvgHeading = sinf(avgHeading); // compute once and cache
    localOffsetRotated.x = localOffset.x * cosAvgHeading - localOffset.y * sinAvgHeading; //rotation matrix X
    localOffsetRotated.y = localOffset.x * sinAvgHeading + localOffset.y * cosAvgHeading; //rotation matrix Y
    
    globalPose.x += localOffsetRotated.x; // add local pose updates to global pose
    globalPose.y += localOffsetRotated.y;
    globalPose.theta = mainChassis->getHeading(); //update heading 
}

void odometry::setPose(Pose newPose){
    globalPose = newPose;
}

Pose odometry::getPose(){
    return globalPose;
}