#pragma once
#include "main.h"
#include "config.hpp"
#include "robomath.hpp"
#include "structs.hpp"
class motionProfiling{ // this should be a namespace, but i coded it late at night, 
                       // no point in changing it now! However, if this now needs
                       // to remember things inbetween calls the backend is already setup for it! -ZK

private:
float linearVelocity{Config::Chassis::linearVelocity}; //linear velocity of x drive (in/s) (not accounting for 1.41x increase)
float robotRadius{Config::Chassis::robotRadius}; // distance between parallel wheel sets (in)


public:
// constructor, write later. most likely will have no inputs
motionProfiling();

/**  * @brief write later you have notes on this -ZK
 * @param currentPose the pose of the robot
 * @param targetPose the pose you want the robot to be in
 * @param thetaD at what point along the line you want to reach theta. Change the variable name later
*/
velocities pointToTrajectory(Pose currentPose, Pose targetPose, float thetaD); //
/** * @brief write later, note that it has to be in RPM */
motorCommands velocityToMotor(velocities velocityRPM);

motorCommands scaleRPM(motorCommands rawCommands, float targetRPM);
};