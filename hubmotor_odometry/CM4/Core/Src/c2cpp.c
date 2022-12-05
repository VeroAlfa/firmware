/*
 * c2cpp.c
 *
 *  Created on: Oct 23, 2022
 *      Author: veroalfa
 */

#include "c2cpp.h"

void UserXicroBegin(UART_HandleTypeDef* huart);
void UserXicroPublishNAV(float pose__of__position__of__x ,float pose__of__position__of__y ,float pose__of__position__of__z ,float pose__of__orientation__of__x ,float pose__of__orientation__of__y ,float pose__of__orientation__of__z ,float pose__of__orientation__of__w ,float twist__of__linear__of__x ,float twist__of__linear__of__y ,float twist__of__linear__of__z ,float twist__of__angular__of__x ,float twist__of__angular__of__y ,float twist__of__angular__of__z);
void UserXicroPublishIMU(float orientation__of__x ,float orientation__of__y ,float orientation__of__z ,float orientation__of__w ,float angular_velocity__of__x ,float angular_velocity__of__y ,float angular_velocity__of__z ,float linear_acceleration__of__x ,float linear_acceleration__of__y ,float linear_acceleration__of__z );
void UserXicroSpin();
float UserXicroGetLinVel();
float UserXicroGetAngVel();

void xicro_begin(UART_HandleTypeDef* huart)
{
	UserXicroBegin(huart);
}

void xicro_publish_nav(float pose__of__position__of__x ,float pose__of__position__of__y ,float pose__of__position__of__z ,float pose__of__orientation__of__x ,float pose__of__orientation__of__y ,float pose__of__orientation__of__z ,float pose__of__orientation__of__w ,float twist__of__linear__of__x ,float twist__of__linear__of__y ,float twist__of__linear__of__z ,float twist__of__angular__of__x ,float twist__of__angular__of__y ,float twist__of__angular__of__z)
{
	UserXicroPublishNAV(pose__of__position__of__x, pose__of__position__of__y, pose__of__position__of__z, pose__of__orientation__of__x, pose__of__orientation__of__y, pose__of__orientation__of__z, pose__of__orientation__of__w, twist__of__linear__of__x, twist__of__linear__of__y, twist__of__linear__of__z, twist__of__angular__of__x, twist__of__angular__of__y, twist__of__angular__of__z);
}

void xicro_publish_imu(float orientation__of__x ,float orientation__of__y ,float orientation__of__z ,float orientation__of__w ,float angular_velocity__of__x ,float angular_velocity__of__y ,float angular_velocity__of__z ,float linear_acceleration__of__x ,float linear_acceleration__of__y ,float linear_acceleration__of__z )
{
	UserXicroPublishIMU(orientation__of__x, orientation__of__y, orientation__of__z, orientation__of__w, angular_velocity__of__x, angular_velocity__of__y, angular_velocity__of__z, linear_acceleration__of__x, linear_acceleration__of__y, linear_acceleration__of__z);
}

void xicro_spin_node()
{
	UserXicroSpin();
}

float xicro_get_linvel()
{
	return UserXicroGetLinVel();
}

float xicro_get_angvel()
{
	return UserXicroGetAngVel();
}


