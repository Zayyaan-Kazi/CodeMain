#include "main.h"
class Chassis {
private:
    //motors -- Drivetrain
    pros::Motor forwardLeft; //defined initially
    pros::Motor backLeft; //defined initially
    pros::Motor forwardRight; //defined initially
    pros::Motor backRight; //defined initially
    pros::Motor middleLeft; //defined initially
    pros::Motor middleRight; //defined initially

    //sensors
    pros::IMU robotIMU; //defined initially
    std::unique_ptr<pros::Rotation> rotationRight; //defined with initalizeRotationSensors
    std::unique_ptr<pros::Rotation> rotationBack; //defined with initalizeRotationSensors
    
    
    pros::Controller controllerMaster; // defined initially
public:
    /// @brief Constructor for all drivetrain motors and IMU
    /// @param MFL Motor, Front Left
    /// @param MBL Motor, Back Left
    /// @param MFR Motor, Front Right
    /// @param MBR Motor, Back Right
    /// @param MML Motor, Middle Left
    /// @param MMR Motor, Middle Right
    /// @param _robotIMU Inerital, initalizer variable for robot imu
    Chassis(int8_t MFL, int8_t MBL, int8_t MFR, int8_t MBR, int8_t MML, int8_t MMR, int8_t _robotIMU);
    
    /// @brief Creates unique pointers for the right and rear rotation sensors.
    /// @param rotationRightPort 
    /// @param rotationBackPort 
    void initalizeRotationSensors(int8_t rotationRightPort, int8_t rotationBackPort);

    /// @brief updates motors based on controller inputs. Driver control
    void driverUpdate();
};