#include "gyroscope.h"
#include "stm32l4xx_hal.h"
#include "main.h"

extern SPI_HandleTypeDef hspi2;

// Function to write to A3G4250D register
void A3G4250D_WriteReg(uint8_t reg, uint8_t data) {
    HAL_GPIO_WritePin(GYR1_nCS_GPIO_Port, GYR1_nCS_Pin, GPIO_PIN_RESET);
    uint8_t txData[2] = {reg, data};
    HAL_SPI_Transmit(&hspi2, txData, 2, 100);
    HAL_GPIO_WritePin(GYR1_nCS_GPIO_Port, GYR1_nCS_Pin, GPIO_PIN_SET);
}

// Function to read from A3G4250D register
uint8_t A3G4250D_ReadReg(uint8_t reg) {
	HAL_GPIO_WritePin(GYR1_nCS_GPIO_Port, GYR1_nCS_Pin, GPIO_PIN_RESET);
    uint8_t txData = 0x80 | reg; // MSB must be set for read
	uint8_t rxData;
	HAL_SPI_TransmitReceive(&hspi2, &txData, &rxData, 1, 100);
    HAL_GPIO_WritePin(GYR1_nCS_GPIO_Port, GYR1_nCS_Pin, GPIO_PIN_SET);
    return rxData;
}

// Function to read gyroscope data
void A3G4250D_ReadGyro(float *pfData){
	uint8_t tmpbuffer[6] = {0};
	int16_t RawData[3] = {0};
	uint8_t tmpreg = 0;
	float sensitivity = GYRO_SENSITIVITY_245DPS;
	int i = 0;

	tmpbuffer[0] = A3G4250D_ReadReg(A3G4250D_OUT_X_L);
	tmpbuffer[1] = A3G4250D_ReadReg(A3G4250D_OUT_X_H);
	tmpbuffer[2] = A3G4250D_ReadReg(A3G4250D_OUT_Y_L);
	tmpbuffer[3] = A3G4250D_ReadReg(A3G4250D_OUT_Y_H);
	tmpbuffer[4] = A3G4250D_ReadReg(A3G4250D_OUT_Z_L);
	tmpbuffer[5] = A3G4250D_ReadReg(A3G4250D_OUT_Z_H);

	tmpreg = A3G4250D_ReadReg(GYRO_CTRL_REG4);

	if (!(tmpreg & GYRO_BLE_MSB)){
		for (i = 0; i < 3; i++){
		  RawData[i] = (int16_t)(((uint16_t)tmpbuffer[2 * i + 1] << 8) + tmpbuffer[2 * i]);
		}
	}
	else{
		for (i = 0; i < 3; i++){
		  RawData[i] = (int16_t)(((uint16_t)tmpbuffer[2 * i] << 8) + tmpbuffer[2 * i + 1]);
		}
	}

	/* Multiplied by sensitivity */
	for (i = 0; i < 3; i++){
		pfData[i] = (float)(RawData[i] * sensitivity);
	}
}

// Function to initialize A3G4250D
void A3G4250D_Init(void) {
	// Enable SPI interface
	__HAL_RCC_SPI1_CLK_ENABLE();
	// Configure CS pin as output
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GYR1_nCS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GYR1_nCS_GPIO_Port, &GPIO_InitStruct);
	// Set CS pin high
	HAL_GPIO_WritePin(GYR1_nCS_GPIO_Port, GYR1_nCS_Pin, GPIO_PIN_SET);
	// Configure gyroscope settings
	A3G4250D_WriteReg(A3G4250D_CTRL_REG1, 0x0F); // Enable all axes, normal mode, 100Hz data rate
}
