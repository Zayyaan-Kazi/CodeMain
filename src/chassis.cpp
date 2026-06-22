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
    rotationRight = std::make_unique<pros::Rotation>(rightRotationPort);
    rotationRear = std::make_unique<pros::Rotation>(rearRotationPort);

    //ensure that the rotation sensors are at 0 and initalize prevPos
    rotationRight->reset_position();
    rotationRear->reset_position();
    rightPrevPos = 0;
    rearPrevPos = 0;
    headingLast = 0;
}
std::pair<float,float> Chassis::getRotationDeltas(){
    // Get Current postition of sensors. Divided by 100 because sensors return in centidegrees
    float rightRotationPos = rotationRight->get_position() / 100;
    float backRotationPos = rotationRear->get_position() /100;
    
    // Get the Delta (difference) between the previous and current readings
    float rightDegDelta = (rightRotationPos) - rightPrevPos;
    float backDegDelta = (backRotationPos) - rearPrevPos;
    
    //Set the previous variable for next check
    rightPrevPos = rightRotationPos;
    rearPrevPos = backRotationPos;
    
    //Convert the delta to inches
    float rightInchDelta = rightDegDelta / 360 * wheelCircumference;
    float backInchDelta = backDegDelta / 360 * wheelCircumference;
    return {rightInchDelta,backInchDelta};
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
    frontLeft.move_velocity(userCommands.fl);
    frontRight.move_velocity(userCommands.fr);
    rearLeft.move_velocity(userCommands.rl);
    rearRight.move_velocity(userCommands.rr);
}

void Chassis::moveXDrive(float verticalRPM, float horizontalRPM, float turningRPM){
    frontLeft.move_velocity( (verticalRPM + horizontalRPM) + turningRPM);
	frontRight.move_velocity((verticalRPM - horizontalRPM) - turningRPM); 
	rearLeft.move_velocity(  (verticalRPM - horizontalRPM) + turningRPM);
	rearRight.move_velocity( (verticalRPM + horizontalRPM) - turningRPM);
}

void Chassis::moveToPose(Pose targetPose, moveToPoseParam parameters){
    if (parameters.turnCompletionDistance == 0.0) return; // the user is an idiot
    Pose currentPose = roboOdom->getPose();
    
    velocities targetVelocities = motionProfile.pointToTrajectory(currentPose,targetPose,parameters.turnCompletionDistance);
    //convert from in/s to rpm using wheel travel using evil code
    velocities targetVelocitiesRPM = RoboMath::velocitiesToRPM(targetVelocities);
    //convert to motor commands
    motorCommands rawMotorCommands = motionProfile.velocityToMotor(targetVelocitiesRPM);
    //scale motor commands
    motorCommands scaledMotorCommands = motionProfile.scaleRPM(rawMotorCommands, parameters.scaleToRPM);

    //create loop using velocity controller that exits when close enough
    
    //swap to positional pid and do that exiting when done

    //uhh return unless i forgot something
}