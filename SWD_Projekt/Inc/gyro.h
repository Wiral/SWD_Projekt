/*
 * gyro.h
 *
 *  Created on: Mar 21, 2018
 *      Author: Jakub
 */

#ifndef GYRO_H_
#define GYRO_H_
#include "spi.h"
#include "usart.h"
#include "gpio.h"
//#define L3GD20_GYRO_ADRESS (0x35 << 1) //110101x
#define L3GD20_GYRO_CTRL_REG1 0x20
#define L3GD20_GYRO_TURN_ON 0x0F		//turn ON power and axis reading
#define L3GD20_GYRO_OUT_X_L 0x28
#define L3GD20_READ_ENABLE 0x80
#define L3GD20_GYRO_MULTI_READ 0x40

uint8_t gyro_tmp[6]; 	// Zmienna do bezposredniego odczytu z akcelerometru
int16_t gyro_data[3];

void L3GD20_Init() {
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, DISABLE); //set CS pin
	uint8_t adress = L3GD20_GYRO_CTRL_REG1;
	HAL_SPI_Transmit(&hspi1, &adress , 1, 100);
	adress = L3GD20_GYRO_TURN_ON;
	HAL_SPI_Transmit(&hspi1,&adress, 1, 100);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, ENABLE);
}

void L3GD20_ReadAxis() {
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, DISABLE);
	uint8_t adress = L3GD20_GYRO_OUT_X_L | L3GD20_GYRO_MULTI_READ | L3GD20_READ_ENABLE;
	HAL_SPI_Transmit(&hspi1,&adress,1,100);
	HAL_SPI_Receive(&hspi1,gyro_tmp,6,100);
	gyro_data[0] = ((gyro_tmp[1] << 8) | gyro_tmp[0]);	//copy gyro_tmp data to acc data
	gyro_data[1] = ((gyro_tmp[3] << 8) | gyro_tmp[2]);	//copy gyro_tmp data to acc data
	gyro_data[2] = ((gyro_tmp[5] << 8) | gyro_tmp[4]);	//copy gyro_tmp data to acc data
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, ENABLE);
}




#endif /* GYRO_H_ */
