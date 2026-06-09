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

inline Pose globalPose {Config::initialPose}; // This is the pose that other classes will interact with 

class odometry {
private:
    const float disR {Config::distanceRight}; //Distance from right sensor and tracking center 
    const float disB {Config::distanceLeft}; // Distance from left sensor and tracking center
    
    Chassis* mainChassis;
    float tolerance {Config::tolerance};

    /// @brief Fixes the rotation offset and updates the global position values */
    void updateOdometry();

public:

    /// @brief Odometry Class Constructor
    /// Odometry is the robot pose tracking algorithim, this one specifically is arc based odometry
    odometry(Chassis* _mainChassis);


    /// @brief Updates Orientation and position values w/ changing ram values using a vector
	/// @param newPose new position vector
    void updatePosition(Pose newPose);
    
    /// @brief Gets the pointer to the position vector
	/// @returns pointer to the position vector */
    std::vector<float>* getPositionPointer();


    /// @brief updates encoder values
    /// @note also automatically calls the calculate local offset, may need to do some c++ trickery */
    void updateDistances();

};




#endif