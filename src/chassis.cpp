#include "chassis.hpp"
#include "main.h"
// initialzier for motors, imu, and controller
Chassis::Chassis(int8_t MFL, int8_t MBL, int8_t MFR, int8_t MBR, 
    int8_t MML, int8_t MMR, int8_t _robotIMU) : forwardLeft(MFL), 
    backLeft(MBL), forwardRight(MFR), backRight(MBR), middleLeft(MML), 
    middleRight(MMR), robotIMU(_robotIMU), controllerMaster(pros::E_CONTROLLER_MASTER) {} 
//

// initialzier for rotation sensors / tracking wheels
void Chassis::initalizeRotationSensors(int8_t rotationRightPort, int8_t rotationBackPort){
    rotationRight = std::make_unique<pros::Rotation>(rotationRightPort);
    rotationBack = std::make_unique<pros::Rotation>(rotationRightPort);

    //ensure that the rotation sensors are at 0
    rotationRight->reset_position();
    rotationBack->reset_position();
}

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
