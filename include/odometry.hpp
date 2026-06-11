#pragma once
#ifndef odometry_H
#define odometry_H
#include <cmath>
#include <iostream>
#include <array>
#include <vector>
#include <string>
#include <thread>
#include "main.h"
#include "api.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "config.hpp"
#include "chassis.hpp"
//structs
struct Pose{
	float x;
	float y;
	float theta;
};

//inline Pose globalPose {Config::initialPose}; // GLOBAL VERSION This is the pose that other classes will interact with 

class odometry {
private:
    const float disR {Config::Odometry::distanceRight}; //Distance from right sensor and tracking center 
    const float disB {Config::Odometry::distanceRear}; // Distance from left sensor and tracking center
    Pose globalPose {Config::Odometry::initialPose}; // This is the pose that other classes will interact with 
    float tolerance {Config::Odometry::tolerance}; //Tolerance before considering a number as 0
    Chassis* mainChassis;

    

public:

    /**  @brief Odometry Class Constructor
     * Odometry is the robot pose tracking algorithim, this one specifically is arc based odometry
     * ```cpp
     * // pass a pointer from your chassis into here
     * odometry roboOdom(&myChassis)
     * ```
     */
    odometry(Chassis* _mainChassis);

    /** @brief updates position using arc odometry 
     * @warning you NEED to have your rotation sensors initalized in your chassis for this to work
     * ```cpp
     * while(true){ //needs to be constantly called to keep track of position
     *  odometry.updatePosition();
     *  pros::delay(20);
     * }
     * ```
    */
    void updatePosition();

    /** @brief Sets new pose for robot
	 *  @param newPose new position vector
     *  ```cpp
     *  //set the robot pose to X:10, Y:20, Theta:30 degrees
     *  odometry.setPose({10.0f,20.0f,30.0f});
     * ```
    */
    void setPose(Pose newPose);

    /** @brief getter for the pose of the robot
    *  @returns a pose object of the robot
    *  ```cpp
    *  Pose currentPose = odometry.getPose();
    *  ```
    */
    Pose getPose();
  


};




#endif