/*
 * c2cpp.h
 *
 *  Created on: Oct 23, 2022
 *      Author: veroalfa
 */

#ifndef INC_C2CPP_H_
#define INC_C2CPP_H_
#include "stm32h7xx_hal.h"

void xicro_begin(UART_HandleTypeDef* huart);
void xicro_publish_nav(float pose__of__position__of__x ,float pose__of__position__of__y ,float pose__of__position__of__z ,float pose__of__orientation__of__x ,float pose__of__orientation__of__y ,float pose__of__orientation__of__z ,float pose__of__orientation__of__w ,float twist__of__linear__of__x ,float twist__of__linear__of__y ,float twist__of__linear__of__z ,float twist__of__angular__of__x ,float twist__of__angular__of__y ,float twist__of__angular__of__z);
void xicro_publish_imu(float orientation__of__x ,float orientation__of__y ,float orientation__of__z ,float orientation__of__w ,float angular_velocity__of__x ,float angular_velocity__of__y ,float angular_velocity__of__z ,float linear_acceleration__of__x ,float linear_acceleration__of__y ,float linear_acceleration__of__z );
void xicro_spin_node();
float xicro_get_linvel();
float xicro_get_angvel();



#endif /* INC_C2CPP_H_ */
