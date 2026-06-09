#pragma once
#include <cmath>
namespace RoboMath{
    float overflowCheck(float deg) {
	    // Normalize to [0, 360)
	    float v = fmod(deg, 360.0);
	    if (v < 0) v += 360.0;
	    return v;
    }
    float subDegrees(float deg1, float deg2) {
	    float delta = deg1 - deg2;
	    // Normalize to [-180, 180]
	    delta = fmodf(delta + 180.0f, 360.0f);
	    if (delta < 0) delta += 360.0f;
	    delta -= 180.0f;
	    return delta;
    }
};