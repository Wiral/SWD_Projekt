/*
 * acc.h
 *
 *  Created on: Mar 21, 2018
 *      Author: Jakub
 */

#ifndef ACC_H_
#define ACC_H_

#include "i2c.h"
#include "usart.h"


#define LSM303_ACC_ADDRESS (0x19 << 1) // adres akcelerometru: 0011 001x
#define LSM303_ACC_CTRL_REG1_A 0x20 // rejestr ustawien 1
#define LSM303_ACC_XYZ_ENABLE 0x07 // 0000 0111
#define LSM303_ACC_100HZ 0x50 // 0101 0000
#define LSM303_ACC_RESOLUTION 2.0 // Maksymalna wartosc przyspieszenia [g]
#define LSM303_ACC_X_L_A_MULTI_READ 0x28 | 0x80 // nizszy bajt danych osi X, multi read

uint8_t acc_tmp[6]; 	// Zmienna do bezposredniego odczytu z akcelerometru
int16_t acc_data[3];

void LSM303_Init() {
	uint8_t Settings = LSM303_ACC_XYZ_ENABLE | LSM303_ACC_100HZ;
	HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG1_A, 1, &Settings, 1, 100);
}

void LSM303_ReadAxis() {
	HAL_I2C_Mem_Read(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_X_L_A_MULTI_READ, 1,
			acc_tmp, 6, 100);
	acc_data[0] = ((acc_tmp[1] << 8) | acc_tmp[0]);	//copy acc_tmp data to acc data
	acc_data[1] = ((acc_tmp[3] << 8) | acc_tmp[2]);	//copy acc_tmp data to acc data
	acc_data[2] = ((acc_tmp[5] << 8) | acc_tmp[4]);	//copy acc_tmp data to acc data
}




#endif /* ACC_H_ */
