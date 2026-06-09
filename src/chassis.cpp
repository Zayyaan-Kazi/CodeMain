#include "chassis.hpp"
#include "main.h"
#include "robomath.hpp"
// initialzier for motors, imu, and controller
Chassis::Chassis() : controllerMaster(pros::E_CONTROLLER_MASTER) {} 


void Chassis::driverUpdate(){
    // Drive control
	// The code below seperates the 3 controller axis' used for driving into variables that are then substituted into each motor
	// output being left stick controls vertical and horizontal and right stick controlling turning
	int32_t powerVertical = controllerMaster.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y); 
	int32_t powerHorizontal = controllerMaster.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X); // these are defined as int32_t because thats what the controller returns
	int32_t powerTurn = controllerMaster.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
	forwardLeft.move( (powerVertical + powerHorizontal) + powerTurn);
	forwardRight.move((powerVertical - powerHorizontal) - powerTurn); // motor.move also expects int_32
	backLeft.move((powerVertical - powerHorizontal) + powerTurn);
	backRight.move( (powerVertical + powerHorizontal) - powerTurn);	
}

// initialzier for rotation sensors / tracking wheels
void Chassis::initalizeRotationSensors(int8_t rotationRightPort, int8_t rotationBackPort){
    rotationRight = std::make_unique<pros::Rotation>(rotationRightPort);
    rotationBack = std::make_unique<pros::Rotation>(rotationRightPort);

    //ensure that the rotation sensors are at 0 and initalize prevPos
    rotationRight->reset_position();
    rotationBack->reset_position();
    rightPrevPos = 0;
    backPrevPos = 0;
    headingLast = 0;
}
std::pair<float,float> Chassis::getRotationDeltas(){
    // Get Current postition of sensors. Multiplied by 100 because sensors return in centidegrees
    float rightRotationPos = rotationRight->get_position() * 100;
    float backRotationPos = rotationBack->get_position() * 100;
    
    // Get the Delta (difference) between the previous and current readings
    float rightDegDelta = (rightRotationPos) - rightPrevPos;
    float backDegDelta = (backRotationPos) - backPrevPos;
    
    //Set the previous variable for next check
    rightPrevPos = rightRotationPos;
    backPrevPos = backRotationPos;
    
    //Convert the delta to inches
    float rightInchDelta = rightDegDelta / 360 * wheelCircumference;
    float backInchDelta = backDegDelta / 360 * wheelCircumference;
    return {rightInchDelta,backInchDelta};
}
float Chassis::getHeadingDelta(){
    float heading = RoboMath::overflowCheck(robotIMU.get_heading()); //pull heading and ensure it is within [0,360)
    heading *= IMUSIGN; // apply CCW positive rotation
    float headingDelta = RoboMath::subDegrees(heading,headingLast); //get the delta within [-180,180]    headingLast = heading;
    return headingDelta;
}
float Chassis::getHeading(){
    return robotIMU.get_heading();
}