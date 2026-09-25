#include "odometry.hpp"
#include "chassis.hpp"
#include "config.hpp"
#include "structs.hpp"
#include "robomath.hpp"
#include <cmath>
odometry::odometry(Chassis* _mainChassis) : mainChassis(_mainChassis){}

void odometry::updatePosition(){
    
    float headingDelta = mainChassis->getHeadingDelta();
    auto distanceDeltas = mainChassis->getRotationDeltas();
    
    Pose localOffset; // create temporary pose to adjust without affecting the main one
    float headingDeltaRAD = RoboMath::degToRad(headingDelta); // convert to radians
    //Optimization caching, i don't want to mess with the compiler flags because they are scary - Kazi
    float halfHeadingDeltaRAD = headingDeltaRAD * 0.5f; // cache
    float arcChordFactor =  2.0*sinf(halfHeadingDeltaRAD); // compute once and cache
    
    
    if(fabsf(headingDelta) > tolerance){
        float invHeadingDeltaRAD =  1.0f / headingDeltaRAD; // multiplication is faster than division on CPU
        localOffset.x = arcChordFactor * ((distanceDeltas.horizontal * invHeadingDeltaRAD) + disR); 
        localOffset.y = arcChordFactor * ((distanceDeltas.vertical * invHeadingDeltaRAD) + disB);
    }else{
        localOffset.x = distanceDeltas.horizontal;
        localOffset.y = distanceDeltas.vertical;
    }
    
    //Rotate to global via Rotation Matrix (https://en.wikipedia.org/wiki/Rotation_matrix)
    Pose localOffsetRotated;
    // rotate by midpoint of heading for a more accurate change in heading
    float avgHeading = (RoboMath::degToRad(globalPose.heading)) + halfHeadingDeltaRAD; 
    
    float cosAvgHeading = cosf(avgHeading); //compute once and cache
    float sinAvgHeading = sinf(avgHeading); // compute once and cache
    localOffsetRotated.x = localOffset.x * cosAvgHeading - localOffset.y * sinAvgHeading; //rotation matrix X
    localOffsetRotated.y = localOffset.x * sinAvgHeading + localOffset.y * cosAvgHeading; //rotation matrix Y
    
    globalPose.x += localOffsetRotated.x; // add local pose updates to global pose
    globalPose.y += localOffsetRotated.y;
    globalPose.heading = mainChassis->getHeading(); //update heading 
}

void odometry::setPose(const Pose& newPose){
    globalPose = newPose;
}

Pose odometry::getPose(){
    return globalPose;
}