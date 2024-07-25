#include <magnetometer_driver.h>
#include "stm32l4xx_hal.h"
#include <stdio.h>
#include "main.h"

extern SPI_HandleTypeDef hspi3;


// Function to write to MAGNETOMETER register
void MAG_WriteReg(uint8_t reg, uint8_t data) {
	HAL_GPIO_WritePin(MAG2_nCS_GPIO_Port, MAG2_nCS_Pin, GPIO_PIN_RESET);
	uint8_t txData[2] = {reg, data};
	HAL_SPI_Transmit(&hspi3, txData, 2, 100);
	HAL_GPIO_WritePin(MAG2_nCS_GPIO_Port, MAG2_nCS_Pin, GPIO_PIN_SET);
}

// Function to read from MAGNETOMETER register
uint8_t MAG_ReadReg(uint8_t reg) {
	HAL_GPIO_WritePin(MAG2_nCS_GPIO_Port, MAG2_nCS_Pin, GPIO_PIN_RESET);
	uint8_t txData = 0x80 | reg; // MSB must be set for read
	uint8_t rxData;
	HAL_SPI_Transmit(&hspi3, &txData, 1, 100);
	HAL_SPI_Receive(&hspi3, &rxData, 1, 100);
	HAL_GPIO_WritePin(MAG2_nCS_GPIO_Port, MAG2_nCS_Pin, GPIO_PIN_SET);
	return rxData;
}

// Function to read magnetic field data
void MAG_ReadMagneticField(int16_t *pData) {
	uint8_t tmpbuffer[6] = {0};

	tmpbuffer[0] = MAG_ReadReg(MAG_XOUT_L);
	tmpbuffer[1] = MAG_ReadReg(MAG_XOUT_H);
	tmpbuffer[2] = MAG_ReadReg(MAG_YOUT_L);
	tmpbuffer[3] = MAG_ReadReg(MAG_YOUT_H);
	tmpbuffer[4] = MAG_ReadReg(MAG_ZOUT_L);
	tmpbuffer[5] = MAG_ReadReg(MAG_ZOUT_H);

	pData[0] = (int16_t)((tmpbuffer[1] << 8) | tmpbuffer[0]);
	pData[1] = (int16_t)((tmpbuffer[3] << 8) | tmpbuffer[2]);
	pData[2] = (int16_t)((tmpbuffer[5] << 8) | tmpbuffer[4]);
}

// Function to initialize MAGNETOMETER
void MAG_Init(void) {
  // Set CS pin high
  HAL_GPIO_WritePin(MAG1_nCS_GPIO_Port, MAG1_nCS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MAG2_nCS_GPIO_Port, MAG2_nCS_Pin, GPIO_PIN_SET);
  // Configure magnetic sensor settings
  MAG_WriteReg(MAG_CONTROL_0, 0x00); // Reset sensor
  HAL_Delay(1); // Wait for reset to complete
}
