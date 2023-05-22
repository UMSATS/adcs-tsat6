/*
 * FILENAME: BNO085_driver.h
 *
 * DESCRIPTION: STM32L4 driver header file for the BNO085 IMU.
 *
 * AUTHORS:
 *  - Daigh Burgess (daigh.burgess@umsats.ca)
 *
 * CREATED ON: May 20, 2022
 */

#ifndef HARDWARE_PERIPHERALS_INC_BNO085_DRIVER_H_
#define HARDWARE_PERIPHERALS_INC_BNO085_DRIVER_H_

//###############################################################################################
//Include Directives
//###############################################################################################
#include <stdint.h>

#include "stm32l4xx_hal.h"

//###############################################################################################
//Public Define Directives
//###############################################################################################
#define BNO085_SPI              hspi2

#define BNO085_H_INTN_GPIO      GPIOC
#define BNO085_H_INTN_PIN       GPIO_PIN_9

#define BNO085_NRST_GPIO        GPIOC
#define BNO085_NRST_PIN         GPIO_PIN_8

#define BNO085_PS0_GPIO         GPIOC
#define BNO085_PS0_PIN          GPIO_PIN_7

#define BNO085_BOOTN_GPIO       GPIOC
#define BNO085_BOOTN_PIN        GPIO_PIN_6

#define BNO085_H_CSN_GPIO       GPIOB
#define BNO085_H_CSN_PIN        GPIO_PIN_12

#define BNO085_SPI_DELAY        HAL_MAX_DELAY

//###############################################################################################
//Public struct
//###############################################################################################
struct Rotation_Vector
{
    uint16_t q_i;       //unit quaternion i component
    uint16_t q_j;       //unit quaternion j component
    uint16_t q_k;       //unit quaternion k component
    uint16_t q_real;    //unit quaternion real component
    uint16_t accuracy;  //measurement accuracy estimate
};

//###############################################################################################
//Global Variable Declarations
//###############################################################################################
extern struct Rotation_Vector rotation_vector;

//###############################################################################################
//Public Driver Function Prototypes
//###############################################################################################
/*
 * FUNCTION: BNO085_Init
 *
 * DESCRIPTION: Resets the BNO085, waits for initialization to complete, then configures the
 *              BNO085 to generate rotation vector reports every minute.
 *
 * NOTES:
 *  - This function clears global variables which the BNO085_driver.c file uses.
 */
HAL_StatusTypeDef BNO085_Init();

/*
 * FUNCTION: BNO085_Reset
 *
 * DESCRIPTION: Resets the BNO085.
 *
 * NOTES:
 *  - This function clears global variables which the BNO085_driver.c file uses.
 *  - The BNO085 must be reconfigured to generate sensor reports after this command is executed.
 */
void BNO085_Reset();

/*
 * FUNCTION: BNO085_Interrupt_Handler
 *
 * DESCRIPTION: Handles the interrupt generated by the BNO085 on the H_INTN pin.
 *
 * NOTES:
 *  - This function clears the interrupt by reading data from the BNO085.
 *  - The read data is processed. If the data is a rotation vector report, the current rotation
 *    vector is updated accordingly & the BNO085_Send_Rotation_Vector_Telemetry function is
 *    called.
 */
HAL_StatusTypeDef BNO085_Interrupt_Handler();

/*
 * FUNCTION: BNO085_Send_Rotation_Vector_Telemetry
 *
 * DESCRIPTION: Sends the current rotation vector telemetry over the CAN bus to CDH.
 *
 * NOTES:
 *  - The telemetry data is split and sent over 2 CAN messages.
 */
HAL_StatusTypeDef BNO085_Send_Rotation_Vector_Telemetry();

#endif /* HARDWARE_PERIPHERALS_INC_BNO085_DRIVER_H_ */