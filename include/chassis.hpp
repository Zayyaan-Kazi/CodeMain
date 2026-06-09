#pragma once
#include "main.h"
#include "config.hpp"
class Chassis {
private:
    //motors -- Drivetrain
    pros::Motor forwardLeft  {Config::frontLeftPort};
    pros::Motor backLeft {Config::backLeftPort};
    pros::Motor forwardRight {Config::frontRightPort};
    pros::Motor backRight {Config::backRightPort};
    pros::Motor middleLeft {Config::middleLeftPort};
    pros::Motor middleRight {Config::middleRightPort};

    //sensors
    pros::IMU robotIMU {Config::IMUport};
    std::unique_ptr<pros::Rotation> rotationRight; //defined with initalizeRotationSensors
    std::unique_ptr<pros::Rotation> rotationBack; //defined with initalizeRotationSensors
    
    pros::Controller controllerMaster; // defined initially, is this even a sensor?

    //Variables -- Odometry
    float wheelCircumference {Config::wheelCircum};
    float rightPrevPos; // Previous reading, in degrees, of right rotation sensor
    float backPrevPos; // Previous reading, in degrees, of left rotation sensor
    float headingLast; // Previous reading, in Degrees, of the robot heading
    int IMUSIGN {Config::inertialSign}; // this is to flip the heading output because we need counter clockwise rotation to be +
public:
    /// @brief Constructor, initalizes controller.
    Chassis();


    /// @brief updates motors based on controller inputs. Driver control
    void driverUpdate();
    
    /// @brief Creates unique pointers for the right and rear rotation sensors.
    /// @param rotationRightPort 
    /// @param rotationBackPort 
    /// @note When Odometry is added to the robot, this can be defined via config
    void initalizeRotationSensors(int8_t rotationRightPort, int8_t rotationBackPort);
    
    /// @brief Get the distance traveled by the rotation sensors since last check
    /// @returns 2 floats, 1st returns right, 2nd returns back
    std::pair<float,float> getRotationDeltas();
    
    /// @brief Gets the change in heading 
    /// @returns 1 float, in degrees
    float getHeadingDelta();
    /// @brief Gets the heading readout from the IMU
    /// @returns 1 float, in degrees
    float getHeading();
};