/*
 * c2cpp.h
 *
 *  Created on: Sep 29, 2022
 *      Author: natta
 */

#ifndef INC_C2CPP_H_
#define INC_C2CPP_H_

void setup();
void loop();
float update_rightwheel(float position, float velocity);
float update_leftwheel(float position, float velocity);
float get_rw_speed_variance();
float get_lw_speed_variance();
float get_lin_vel_variance();
float get_ang_vel_variance();

#endif /* INC_C2CPP_H_ */
