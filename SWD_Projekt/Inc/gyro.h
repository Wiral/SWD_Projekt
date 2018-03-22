/*
 * gyro.h
 *
 *  Created on: Mar 21, 2018
 *      Author: Jakub
 */

#ifndef GYRO_H_
#define GYRO_H_

#include "i2c.h"
#include "usart.h"

#define L3GD20_GYRO_ADRESS (0x35 << 1) //110101x
#define L3GD20_GYRO_CTRL_REG1 0x20
#define L3GD20_GYRO_TURN_ON 0x0F		//turn ON power and axis reading
#define L3GD20_GYRO_OUT_X_L 0x28
#define L3GD20_GYRO_MULTI_READ 0x80

uint8_t gyro_tmp[6]; 	// Zmienna do bezposredniego odczytu z akcelerometru
int16_t gyro_data[3];

void LSM303_Init() {
	uint8_t Settings = L3GD20_GYRO_TURN_ON;
	HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG1_A, 1, &Settings, 1, 100);
}

void LSM303_ReadAxis() {
	HAL_I2C_Mem_Read(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_X_L_A_MULTI_READ, 1, gyro_tmp, 6, 100);
	acc_data[0] = ((gyro_tmp[1] << 8) | gyro_tmp[0]);	//copy gyro_tmp data to acc data
	acc_data[1] = ((gyro_tmp[3] << 8) | gyro_tmp[2]);	//copy gyro_tmp data to acc data
	acc_data[2] = ((gyro_tmp[5] << 8) | gyro_tmp[4]);	//copy gyro_tmp data to acc data
}


#endif /* GYRO_H_ */
