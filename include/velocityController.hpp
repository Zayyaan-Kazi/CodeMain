#pragma once
#include "main.h"
#include "config.hpp"
class velocityController {
private:
    //feed forward constants
    float kS{Config::velocityControl::feedForward::kS}; // minimum voltage to overcome friction
    float kV{Config::velocityControl::feedForward::kV}; // voltage to maintain a constant speed

    //PI constants
    float kP{Config::velocityControl::PI::kP}; // constant for proportional (distance left)
    float kI{Config::velocityControl::PI::kI}; // constant for integral (sum of distances)

    //general variables
     // as of right now i dont think these 2 really need any
     // but if that were to change here are where to put them
    //feed forward specific variables

    //PI specific variables
    float prevError{0.0}; // error from previous loop
    float integral{0.0}; //stores the error over time
    float maxIntegral{Config::velocityControl::PI::maxIntegral}; //prevents integral from heavily overpowering the motor
    float integralThreshold{Config::velocityControl::PI::integrealThreshold}; //prevents integral windup on large errors
public:
    /** * @brief Constructor all values are defined in config */
    velocityController();
    /** * @brief resets the integral variable
     */
    void resetIntegral();

    /** * @brief Calculates the voltage to move the motor at the desired RPM
     * @param targetRPM how fast you want the motor to spin, in RPM
     * @param currentRPM how fast the motor is currently spinning at, in RPM
     * @return a the voltage in milivolts the motor needs to spin at to reach it
     * @note This function runs once so you need to put it in a loop when using it
     */
    float calculateVoltage(float targetRPM, float currentRPM);

};