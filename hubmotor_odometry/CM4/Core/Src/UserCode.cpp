/*
 * UserCode.cpp
 *
 *  Created on: Oct 23, 2022
 *      Author: veroalfa
 */

#include "Xicro_sub_N_pub_ID_3.h"

/* Create Xicro object */
Xicro xicro;

extern "C" void UserXicroBegin(UART_HandleTypeDef* huart);
extern "C" void UserXicroPublishNAV(float pose__of__position__of__x ,float pose__of__position__of__y ,float pose__of__position__of__z ,float pose__of__orientation__of__x ,float pose__of__orientation__of__y ,float pose__of__orientation__of__z ,float pose__of__orientation__of__w ,float twist__of__linear__of__x ,float twist__of__linear__of__y ,float twist__of__linear__of__z ,float twist__of__angular__of__x ,float twist__of__angular__of__y ,float twist__of__angular__of__z);
extern "C" void UserXicroPublishIMU(float orientation__of__x ,float orientation__of__y ,float orientation__of__z ,float orientation__of__w ,float angular_velocity__of__x ,float angular_velocity__of__y ,float angular_velocity__of__z ,float linear_acceleration__of__x ,float linear_acceleration__of__y ,float linear_acceleration__of__z );
extern "C" void UserXicroSpin();
extern "C" float UserXicroGetLinVel();
extern "C" float UserXicroGetAngVel();

void UserXicroBegin(UART_HandleTypeDef* huart)
{
	xicro.begin(huart);
}

void UserXicroPublishNAV(float pose__of__position__of__x ,float pose__of__position__of__y ,float pose__of__position__of__z ,float pose__of__orientation__of__x ,float pose__of__orientation__of__y ,float pose__of__orientation__of__z ,float pose__of__orientation__of__w ,float twist__of__linear__of__x ,float twist__of__linear__of__y ,float twist__of__linear__of__z ,float twist__of__angular__of__x ,float twist__of__angular__of__y ,float twist__of__angular__of__z)
{
	xicro.publish_nav_stm32(pose__of__position__of__x, pose__of__position__of__y, pose__of__position__of__z, pose__of__orientation__of__x, pose__of__orientation__of__y, pose__of__orientation__of__z, pose__of__orientation__of__w, twist__of__linear__of__x, twist__of__linear__of__y, twist__of__linear__of__z, twist__of__angular__of__x, twist__of__angular__of__y, twist__of__angular__of__z);
}

void UserXicroPublishIMU(float orientation__of__x ,float orientation__of__y ,float orientation__of__z ,float orientation__of__w ,float angular_velocity__of__x ,float angular_velocity__of__y ,float angular_velocity__of__z ,float linear_acceleration__of__x ,float linear_acceleration__of__y ,float linear_acceleration__of__z )
{
	xicro.publish_imu_stm32(orientation__of__x, orientation__of__y, orientation__of__z, orientation__of__w, angular_velocity__of__x, angular_velocity__of__y, angular_velocity__of__z, linear_acceleration__of__x, linear_acceleration__of__y, linear_acceleration__of__z);
}

void UserXicroSpin()
{
	xicro.Spin_node();
}

float UserXicroGetLinVel()
{
	return xicro.Sub_cmd_vel_stm32.linear;
}

float UserXicroGetAngVel()
{
	return xicro.Sub_cmd_vel_stm32.angular;
}
