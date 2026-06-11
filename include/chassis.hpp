#pragma once
#include "main.h"
#include "config.hpp"
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
    std::unique_ptr<pros::Rotation> rotationRight; //defined with initalizeRotationSensors
    std::unique_ptr<pros::Rotation> rotationRear; //defined with initalizeRotationSensors
    
    pros::Controller controllerMaster; // defined initially, is this even a sensor?

    //Variables -- Odometry
    float wheelCircumference {Config::Chassis::wheelCircum};
    float rightPrevPos; // Previous reading, in degrees, of right rotation sensor
    float rearPrevPos; // Previous reading, in degrees, of rear rotation sensor
    float headingLast; // Previous reading, in Degrees, of the robot heading
    int IMUSIGN {Config::Chassis::inertialSign}; // this is to flip the heading output because we need counter clockwise rotation to be +
public:
    /** @brief Constructor, initalizes controller.
     * ```cpp
     * Chassis roboChassis = Chassis(); ``` 
     * */
    Chassis();

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
     *  @returns 2 floats, 1st returns right, 2nd returns back
     *  @warning You need to initalize the rotation sensors before using this
     *  ```cpp
     *  auto deltas = chassis->getRotationDeltas();
     *  float rightDelta = deltas.first;
     *  float backDelta = deltas.second;
     * ``` 
     * */
    std::pair<float,float> getRotationDeltas();
    
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
};