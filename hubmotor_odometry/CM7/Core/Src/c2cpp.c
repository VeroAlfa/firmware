/*
 * c2cpp.c
 *
 *  Created on: Sep 29, 2022
 *      Author: natta
 */

#include "c2cpp.h"

void UserCodeSetup();
void UserCodeLoop();
float UserCodeUpdateRightWheel(float position, float velocity);
float UserCodeUpdateLeftWheel(float position, float velocity);
float UserGetRWSpeedVariance();
float UserGetLWSpeedVariance();
float UserGetLinVelVariance();
float UserGetAngVelVariance();

void setup()
{
	// Call c++ domain
	UserCodeSetup();
}

void loop()
{
	// Call c++ domain
	UserCodeLoop();
}

float update_rightwheel(float position, float velocity)
{
	// Call c++ domain
	return UserCodeUpdateRightWheel(position, velocity);
}

float update_leftwheel(float position, float velocity)
{
	return UserCodeUpdateLeftWheel(position, velocity);
}

float get_rw_speed_variance()
{
	return UserGetRWSpeedVariance();
}

float get_lw_speed_variance()
{
	return UserGetLWSpeedVariance();
}

float get_lin_vel_variance()
{
	return UserGetLinVelVariance();
}

float get_ang_vel_variance()
{
	return UserGetAngVelVariance();
}
