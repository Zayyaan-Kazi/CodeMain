#include "chassis.hpp"
#include "main.h"
#include "robomath.hpp"
#include "odometry.hpp"
#include "structs.hpp"
// initialzier for motors, imu, and controller
Chassis::Chassis() : controllerMaster(pros::E_CONTROLLER_MASTER) {} 

void Chassis::setOdometry(odometry* odom) {
    roboOdom = odom;
}

void Chassis::driverUpdate(){
    // Drive control
	// The code below seperates the 3 controller axis' used for driving into variables that are then substituted into each motor
	// output being left stick controls vertical and horizontal and right stick controlling turning
	int32_t powerVertical = controllerMaster.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y); 
	int32_t powerHorizontal = controllerMaster.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X); // these are defined as int32_t because thats what the controller returns
	int32_t powerTurn = controllerMaster.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
	frontLeft.move( (powerVertical + powerHorizontal) + powerTurn); //doesn't use moveXDrive because motor.move has the same bounds as the controller output
	frontRight.move((powerVertical - powerHorizontal) - powerTurn); // motor.move also expects int_32
	rearLeft.move((powerVertical - powerHorizontal) + powerTurn);
	rearRight.move( (powerVertical + powerHorizontal) - powerTurn);	
}

// initialzier for rotation sensors / tracking wheels
void Chassis::initalizeRotationSensors(int8_t rightRotationPort, int8_t rearRotationPort){
    rotationVertical = std::make_unique<pros::Rotation>(rightRotationPort);
    rotationHorizontal = std::make_unique<pros::Rotation>(rearRotationPort);

    //ensure that the rotation sensors are at 0 and initalize prevPos
    rotationVertical->reset_position();
    rotationHorizontal->reset_position();
    verticalPrevPos = 0;
    horizontalPrevPos = 0;
    headingLast = 0;
}
rotationDeltas Chassis::getRotationDeltas(){
    // Get Current postition of sensors. Divided by 100 because sensors return in centidegrees
    float verticalRotationPos = rotationVertical->get_position() / 100;
    float horizontalRotationPos = rotationHorizontal->get_position() /100;
    
    // Get the Delta (difference) between the previous and current readings
    float verticalDegDelta = (verticalRotationPos) - verticalPrevPos;
    float horizontalDegDelta = (horizontalRotationPos) - horizontalPrevPos;
    
    //Set the previous variable for next check
    verticalPrevPos = verticalRotationPos;
    horizontalPrevPos = horizontalRotationPos;
    
    //Convert the delta to inches
    float verticalInchDelta = verticalDegDelta / 360 * wheelCircumference;
    float horizontalInchDelta = horizontalDegDelta / 360 * wheelCircumference;
    return {verticalInchDelta,horizontalInchDelta};
}
float Chassis::getHeadingDelta(){
    float heading = RoboMath::overflowCheck(robotIMU.get_heading()); //pull heading and ensure it is within [0,360)
    heading *= IMUSIGN; // apply CCW positive rotation
    float headingDelta = RoboMath::subDegrees(heading,headingLast); //get the delta within [-180,180]    
    headingLast = heading; // reset heading last before returning
    return headingDelta; 
}
float Chassis::getHeading(){
    return robotIMU.get_heading();
}

void Chassis::moveDrivetrain(motorCommands userCommands){
    int32_t flVoltage = static_cast<int32_t>(fl.calculateVoltage(userCommands.fl, frontLeft.get_actual_velocity()));//cast these to in32t because thats waht the motor takes as a parameter
    int32_t frVoltage = static_cast<int32_t>(fr.calculateVoltage(userCommands.fr, frontRight.get_actual_velocity())); // its probably fine to leave them as floats but i'd like to state intention
    int32_t rlVoltage = static_cast<int32_t>(rl.calculateVoltage(userCommands.rl, rearLeft.get_actual_velocity()));
    int32_t rrVoltage = static_cast<int32_t>(rr.calculateVoltage(userCommands.rr, rearRight.get_actual_velocity()));

    frontLeft.move_voltage(flVoltage);
    frontRight.move_voltage(frVoltage);
    rearLeft.move_voltage(rlVoltage);
    rearRight.move_voltage(rrVoltage);

    /* this one uses the motor's onboard PID which can be... problematic
    frontLeft.move_velocity(userCommands.fl);
    frontRight.move_velocity(userCommands.fr);
    rearLeft.move_velocity(userCommands.rl);
    rearRight.move_velocity(userCommands.rr);*/
}

void Chassis::moveXDrive(float verticalRPM, float horizontalRPM, float turningRPM){
    frontLeft.move_velocity( (verticalRPM + horizontalRPM) + turningRPM);
	frontRight.move_velocity((verticalRPM - horizontalRPM) - turningRPM); 
	rearLeft.move_velocity(  (verticalRPM - horizontalRPM) + turningRPM);
	rearRight.move_velocity( (verticalRPM + horizontalRPM) - turningRPM);
}

void Chassis::moveToPoint(Pose targetPose, moveToPoseParam movementParams){
    motionProfile.reset();
    fl.resetIntegral();
    fr.resetIntegral();
    rl.resetIntegral();
    rr.resetIntegral();
    while(!motionProfile.isSettled()){
        Pose currentPose = roboOdom->getPose();
        motorCommands velocityCommands = motionProfile.targetToMotorRPM(targetPose,currentPose,movementParams); //RPM
        moveDrivetrain(velocityCommands);
        pros::delay(10); //
    }
    
}