#include "velocityController.hpp"
#include "config.hpp"
#include "robomath.hpp"
velocityController::velocityController() {}

void velocityController::resetIntegral(){
    integral = 0.0;
}
float velocityController::calculateVoltage(float targetRPM, float currentRPM){
    //get values for PI
    float error = targetRPM - currentRPM;
    
    int errorSign = RoboMath::sign(error);
    if(errorSign != RoboMath::sign(prevError)){ // prevent integral from misbehaving by resetting when we have to swap direction
        integral = 0.0;
    }
    
    if(fabs(error) > integralThreshold){ // add to integral when below threshold
        integral += error;
    }
    integral = std::clamp(integral, -maxIntegral, maxIntegral);

    // seperate all terms into variables
    float pTerm = kP * error;
    float iTerm = kI * integral;
    float staticTerm = kS * errorSign;
    float voltageTerm = kV * targetRPM;

    prevError = error; // quickly update prevError before returning
    return pTerm+iTerm+staticTerm+voltageTerm;

}