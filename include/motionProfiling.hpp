#pragma once
#include "main.h"
#include "config.hpp"
#include "robomath.hpp"
#include "structs.hpp"
#include "pd.hpp"
class motionProfiling{ // The name is kinda a misnomer change later -ZK

private:
float linearVelocity{Config::Chassis::linearVelocity}; //linear velocity of x drive (in/s) (not accounting for 1.41x increase)
float robotRadius{Config::Chassis::robotRadius}; // distance between parallel wheel sets (in)
bool settled = false; // Is the robot at its target point
bool firstCall = true; // first call of the movement, starts timer
uint32_t startTime = 0.0; // time elasped since start of movement
PD headingPD{Config::positionalController::headingPD::kP,Config::positionalController::headingPD::kD};
PD positionalPDX{Config::positionalController::X::kP, Config::positionalController::X::kD};
PD positionalPDY{Config::positionalController::Y::kP, Config::positionalController::Y::kD};

public:

// constructor, write later. most likely will have no inputs
motionProfiling();

/**  * @brief write later you have notes on this -ZK
 * @param currentPose the pose of the robot
 * @param targetPose the pose you want the robot to be in
 * @param turnCompletionDistance  at what point along the line you want to reach theta. Change the variable name later
*/
velocities pointToTrajectory(Pose currentPose, Pose targetPose,PoseError deltaPose, moveToPoseParam movementParams ); //
/** * @brief write later, note that it has to be in RPM */
motorCommands velocityToMotor(velocities velocityRPM);

motorCommands scaleRPM(motorCommands rawCommands, float targetRPM);

motorCommands targetToMotorRPM(Pose targetPose,Pose currentPose, moveToPoseParam movementParams);

// returns a boolean depending on if the robot is still actively trying to move toward its target point
bool isSettled();

// resets variables and PD loops back to default state ready for next movement
void reset();
};