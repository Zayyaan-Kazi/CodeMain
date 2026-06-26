#pragma once
#include "main.h"
#include "config.hpp"
#include "robomath.hpp"
#include "structs.hpp"

class PD {
private:
    float prevError = 0.0f;
    float kP = 0.0f;
    float kD = 0.0f;
public:
    
    /** @brief constructor for PD clss
     *  @param kP proportional constant
     *  @param kD derivative constant
     */
    PD(float kP, float kD);
    /** *@brief PD output
     * @param error difference between current and target
     * @param dt delta time (seconds)
     * ```cpp
     * PD headingPD(1,0.03);
     * float headingError = targetHeading - currentHeading;
     * while(headingError > 1){
     *  headingError = targetHeading - currentHeading;
     *  headingPD.update(headingError, .01); // .01 for 10ms delay
     *  pros::delay(10);
     * }
     * ```
     */
    float update(float error, float dt);

    /** @brief resets prevError variable, run when changing target
     * ```cpp
     * PD headingPD(1,0.03);
     * float headingError = targetHeading - currentHeading;
     * while(headingError > 1){
     *  // update code
     * }
     * headingPD.reset();
     * ```
     */
    void reset();

};