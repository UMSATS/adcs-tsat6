/*
 * GYRO_A3G4250DTR_driver.h
 *
 *  Created on: Jan 19, 2024
 *      Author: Alexandr Yermakov, Andrii Kvasnytsia
 */

/*

Datasheet URL:
https://www.st.com/resource/en/datasheet/a3g4250d.pdf

*/

#ifndef HARDWARE_PERIPHERALS_INC_GYRO_A3G4250DTR_DRIVER_H_
#define HARDWARE_PERIPHERALS_INC_GYRO_A3G4250DTR_DRIVER_H_

#include "stm32l4xx_hal.h"

// A3G4250D register addresses
#define GYRO_WHO_AM_I		0x0F
#define GYRO_CTRL_REG1		0x20
#define GYRO_OUT_X_L		0x28
#define GYRO_OUT_X_H		0x29
#define GYRO_OUT_Y_L		0x2A
#define GYRO_OUT_Y_H		0x2B
#define GYRO_OUT_Z_L		0x2C
#define GYRO_OUT_Z_H		0x2D

// Function to write to A3G4250D register
void GYRO_WriteReg(uint8_t reg, uint8_t data);

// Function to read from A3G4250D register
uint8_t GYRO_ReadReg(uint8_t reg);

// Function to read gyroscope data
void GYRO_ReadAngRate(int16_t *pData);

// Function to initialize A3G4250D
void GYRO_Init(void);

#endif /* HARDWARE_PERIPHERALS_INC_GYRO_A3G4250DTR_DRIVER_H_ */


