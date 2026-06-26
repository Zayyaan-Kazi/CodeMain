#pragma once
#include "main.h"
#include "config.hpp"
#include "velocityController.hpp"
#include "motionProfiling.hpp"
class odometry; // forward declaration for odom object
class Chassis {
private:
    //motors -- Drivetrain
    pros::Motor frontLeft  {Config::Ports::frontLeft};
    pros::Motor rearLeft {Config::Ports::rearLeft};
    pros::Motor frontRight {Config::Ports::frontRight};
    pros::Motor rearRight {Config::Ports::rearRight};
    pros::Motor middleLeft {Config::Ports::middleLeft};
    pros::Motor middleRight {Config::Ports::middleRight};

    //sensors
    pros::IMU robotIMU {Config::Ports::IMU};
    std::unique_ptr<pros::Rotation> rotationVertical; //defined with initalizeRotationSensors
    std::unique_ptr<pros::Rotation> rotationHorizontal; //defined with initalizeRotationSensors
    
    pros::Controller controllerMaster; // defined initially, is this even a sensor?

    //Variables -- Odometry
    float wheelCircumference {Config::Chassis::wheelCircum};
    float verticalPrevPos; // Previous reading, in degrees, of right rotation sensor
    float horizontalPrevPos; // Previous reading, in degrees, of rear rotation sensor
    float headingLast; // Previous reading, in Degrees, of the robot heading
    int IMUSIGN {Config::Chassis::inertialSign}; // this is to flip the heading output because we need counter clockwise rotation to be +
    odometry* roboOdom = nullptr; // odom object to declare later
    
    //Objects Drivetrain
    velocityController fl;
    velocityController rl;
    velocityController fr;
    velocityController rr;

    motionProfiling motionProfile;

public:
    /** @brief Constructor, initalizes controller.
     * ```cpp
     * Chassis roboChassis = Chassis(); ``` 
     * */
    Chassis();

    void setOdometry(odometry* odom);

    /** @brief updates motors based on controller inputs. Driver control
     * Example code:
     * ```cpp
     *  while (true) {
     *      Chassis.driverUpdate();
     *      pros::delay(20);// Prevent CPU overload
     *  }
     * ```    
     * */
    void driverUpdate();
    /** @brief Creates unique pointers for the right and rear rotation sensors.
     * @warning the ports HAVE to map to rotation sensors there is no warning if an incorrect port is mapped
     * @param rightRotationPort the port number of the right rotation sensor
     * @param rearRotationPort the port number of the rear rotation sensor
     * ```cpp
     * Chassis.initalizeRotationSensors(1,2);
     * ```
     * @note When Odometry is added to the robot, this can be depreciated and be initially defined 
     * */
    void initalizeRotationSensors(int8_t rightRotationPort, int8_t rearRotationPort);
    
    /** @brief Get the distance traveled by the rotation sensors since last check
     *  @returns 2 floats
     *  @warning You need to initalize the rotation sensors before using this
     *  ```cpp
     *  auto deltas = chassis->getRotationDeltas();
     *  float rightDelta = deltas.first;
     *  float backDelta = deltas.second;
     * ``` 
     * */
    rotationDeltas getRotationDeltas();
    
    /**  @brief Gets the change in heading 
     * @returns 1 float, in degrees
     * ```cpp 
     * float changeInHeading = chassis->getHeadingDelta(); 
     * ```
     * */
    float getHeadingDelta();
    /**  @brief Gets the heading readout from the IMU
     * @returns 1 float, in degrees
     * ```cpp
     * float roboHeading = chassis->getHeading();
     * ```
     * */
    float getHeading();
    /** @brief Spins the X motors at a target RPM
     *  @param userCommands each motor in the struct will target their rpm
     *  @warning do not call this function whilst calling driverUpdate
     *  ```cpp
     *  //Set all drivetrain motors to 30 RPM
     *  Chassis.moveDrivetrain(30,30,30,30);
     *  ```
     */
    void moveDrivetrain(const motorCommands& userCommands);
    /** @brief Spins the X motors at a target RPM in vertical, horizontal, and turning
     *  @param verticalRPM target RPM for forward/backward movement
     *  @param horizontalRPM target RPM for strafe/sideways movements
     *  @param turningRPM target RPM for turning, as in changing heading
     *  @warning do not call this function whilst calling driverUpdate
     *  ```cpp
     *  //move 50rpm forward, 40 rpm sideways, 20 rpm turning
     *  chassis.moveXDrive(50,40,20);
     *  ```
     */
    void moveXDrive(float verticalRPM, float horizontalRPM, float turningRPM);

    // No comments for now will add comments when functionality is fully done
    void moveToPoint(const Pose& targetPose,const moveToPoseParam& movementParams);
};