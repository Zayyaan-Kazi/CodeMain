#pragma once
#include "main.h"
#include "config.hpp"
#include "robomath.hpp"
#include "structs.hpp"


#pragma once

class PD {
private:
    float prevError = 0.0f;
    float kP = 0.0f;
    float kD = 0.0f;
public:
    

    PD(float kP, float kD);
    float update(float error, float dt);
    void reset();

};