#pragma once
#include <cmath>
#include "structs.hpp"
namespace RoboMath{
	float pi = 3.1415926f;
	float tau = pi*2;

	float distance(float a,float b){
		return sqrtf(a*a+b*b);
	}
    float overflowCheck(float deg) {
	    // Normalize to [0, 360)
	    float v = fmod(deg, 360.0);
	    if (v < 0) v += 360.0;
	    return v;
    }
	//deg1 - deg2 normalized to [-180,180]
    float subDegrees(float deg1, float deg2){
		float delta = deg1-deg2;
		return std::remainderf(delta,360); // normalize to [-180,180] before returning
	}
	float wrapRadian(float rad){
		return std::remainderf(rad,tau); // normalize to [-pi,pi]
	}
	float degToRad(float deg){
		static const float conversion = pi / 180.0f;
		return deg*conversion;
	}
	/** * @brief returns 1 if number is positive -1 if negative
	 */
	int sign(float number){
		if (number > 0){
			return 1; 
		}
		return -1;
	}
	/** * @brief converts in/s to motor RPM 
	 * This assumes in/s 
	 */
	velocities velocitiesToRPM(velocities initVelocity){
		velocities velocityRPM = initVelocity; // create object to apply changes to
		float rpmConversion = 60.0 / Config::Chassis::wheelCircum; // combine it all into 1 math thing (speed*60) / circum = RPM
		velocityRPM.xVelocity *= rpmConversion;
		velocityRPM.yVelocity *= rpmConversion;
		velocityRPM.thetaVelocity *= rpmConversion;
		return velocityRPM;
	}
	
	/*
	float subDegrees(float deg1, float deg2) { // old replaced with remainder function
	float delta = deg1 - deg2;
	// Normalize to [-180, 180]
	delta = fmodf(delta + 180.0f, 360.0f);
	if (delta < 0) delta += 360.0f;
	delta -= 180.0f;
	return delta;
	}
	*/
};