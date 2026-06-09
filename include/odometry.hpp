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
    const float disR {Config::distanceRight}; //Distance from right sensor and tracking center 
    const float disB {Config::distanceLeft}; // Distance from left sensor and tracking center
    Pose globalPose {Config::initialPose}; // This is the pose that other classes will interact with 
    Chassis* mainChassis;
    float tolerance {Config::tolerance};

    /// @brief Fixes the rotation offset and updates the global position values */
    void updateOdometry();

public:

    /// @brief Odometry Class Constructor
    /// Odometry is the robot pose tracking algorithim, this one specifically is arc based odometry
    odometry(Chassis* _mainChassis);


    /// @brief Sets new pose for robot
	/// @param newPose new position vector
    void setPose(Pose newPose);
    
    /// @brief sets pose without having to create a pose object
    /// @param x new x value
    /// @param y new y value
    /// @param theta new theta/heading value
    void setPose(float x, float y, float theta);

    /// @brief getter for the pose of the robot
    /// @returns a pose object of the robot
    Pose getPose();
  


};




#endif