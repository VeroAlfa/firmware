/*
 * SRAM4.h
 *
 *  Created on: Oct 27, 2022
 *      Author: veroalfa
 */

#ifndef SRC_SRAM4_H_
#define SRC_SRAM4_H_

#define SHARED_MEMORY_ADDRESS 0x38000000

struct shared_data {
    /* shared data goes here */
	float robot_x;
	float robot_y;
	float robot_qx;
	float robot_qy;
	float robot_qz;
	float robot_qw;
	float robot_linvel;
	float robot_angvel;
	float imu_qx;
	float imu_qy;
	float imu_qz;
	float imu_qw;
	float imu_gx;
	float imu_gy;
	float imu_gz;
	float imu_ax;
	float imu_ay;
	float imu_az;
	float cmd_vel_linear;
	float cmd_vel_angular;
	uint8_t nav_pub_flag;
	uint8_t imu_pub_flag;
};
volatile struct shared_data * const shared_ptr = (struct shared_data *)(SHARED_MEMORY_ADDRESS);

#endif /* SRC_SRAM4_H_ */
