#ifndef __GYROSCOPE_H__
#define __GYROSCOPE_H__

#include "stm32l4xx_hal.h"

//#define A3G4250D_SPI &hspi2 // SPI interface
//#define A3G4250D_CS_PIN GPIO_PIN_4 // Chip select pin
//#define A3G4250D_CS_GPIO_PORT GPIOA // Chip select port

// A3G4250D register addresses
#define A3G4250D_WHO_AM_I 0x0F
#define A3G4250D_CTRL_REG1 0x20
#define A3G4250D_OUT_X_L 0x28
#define A3G4250D_OUT_X_H 0x29
#define A3G4250D_OUT_Y_L 0x2A
#define A3G4250D_OUT_Y_H 0x2B
#define A3G4250D_OUT_Z_L 0x2C
#define A3G4250D_OUT_Z_H 0x2D
#define GYRO_CTRL_REG4	 0x23

/* Full Scale Sensitivity */
#define GYRO_SENSITIVITY_245DPS  ((float)8.75f)

/* Endian Data selection */
#define GYRO_BLE_MSB			((uint8_t)0x40)

// Function to write to A3G4250D register
void A3G4250D_WriteReg(uint8_t reg, uint8_t data);

// Function to read from A3G4250D register
uint8_t A3G4250D_ReadReg(uint8_t reg);

// Function to read gyroscope data
void A3G4250D_ReadGyro(float *pfData);

// Function to initialize A3G4250D
void A3G4250D_Init(void);

#endif // __GYROSCOPE_H__
