#include "pd.hpp"

PD::PD(float kP, float kD) : kP(kP), kD(kD) {}

float PD::update(float error, float dt) {
    float derivative = (error - prevError) / dt;
    prevError = error;
    return kP * error + kD * derivative;
}

void PD::reset() {
    prevError = 0.0f;
}