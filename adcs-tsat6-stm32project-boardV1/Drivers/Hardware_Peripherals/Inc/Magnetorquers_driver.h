/*
 * FILENAME: Magnetorquers_driver.h
 *
 * DESCRIPTION: STM32L4 driver header file for the 3 Magnetorquers.
 *
 * AUTHORS:
 *  - Daigh Burgess (daigh.burgess@umsats.ca)
 *
 * CREATED ON: May 18, 2023
 */

#ifndef HARDWARE_PERIPHERALS_INC_MAGNETORQUERS_DRIVER_H_
#define HARDWARE_PERIPHERALS_INC_MAGNETORQUERS_DRIVER_H_

//###############################################################################################
//Include Directives
//###############################################################################################
#include "stm32l4xx_hal.h"

//###############################################################################################
//Public Define Directives
//###############################################################################################
#define PWM1_GPIO       GPIOA
#define PWM1_PIN        GPIO_PIN_1

#define PWMSW1_GPIO     GPIOA
#define PWMSW1_PIN      GPIO_PIN_0

#define PWM2_GPIO       GPIOA
#define PWM2_PIN        GPIO_PIN_7

#define PWMSW2_GPIO     GPIOA
#define PWMSW2_PIN      GPIO_PIN_2

#define PWM3_GPIO       GPIOC
#define PWM3_PIN        GPIO_PIN_5

#define PWMSW3_GPIO     GPIOC
#define PWMSW3_PIN      GPIO_PIN_4

//###############################################################################################
//Public Driver Function Prototypes
//###############################################################################################
/*
 * FUNCTION: Magnetorquers_Init
 *
 * DESCRIPTION: Turn all 3 Magnetorquers off. Set all 3 Magnetorquers to reverse direction.
 */
void Magnetorquers_Init();

/*
 * FUNCTION: Magnetorquer1_Forward
 *
 * DESCRIPTION: Set Magnetorquer1 to forward direction.
 */
void Magnetorquer1_Forward();

/*
 * FUNCTION: Magnetorquer2_Forward
 *
 * DESCRIPTION: Set Magnetorquer2 to forward direction.
 */
void Magnetorquer2_Forward();

/*
 * FUNCTION: Magnetorquer3_Forward
 *
 * DESCRIPTION: Set Magnetorquer3 to forward direction.
 */
void Magnetorquer3_Forward();

/*
 * FUNCTION: Magnetorquer1_Reverse
 *
 * DESCRIPTION: Set Magnetorquer1 to reverse direction.
 */
void Magnetorquer1_Reverse();

/*
 * FUNCTION: Magnetorquer2_Reverse
 *
 * DESCRIPTION: Set Magnetorquer2 to reverse direction.
 */
void Magnetorquer2_Reverse();

/*
 * FUNCTION: Magnetorquer3_Reverse
 *
 * DESCRIPTION: Set Magnetorquer3 to reverse direction.
 */
void Magnetorquer3_Reverse();

/*
 * FUNCTION: Magnetorquer1_Full_Strength
 *
 * DESCRIPTION: Turn Magnetorquer1 on with full strength output. This is accomplished by using a
 *              constant HIGH GPIO signal instead of a PWM signal.
 */
void Magnetorquer1_Full_Strength();

/*
 * FUNCTION: Magnetorquer2_Full_Strength
 *
 * DESCRIPTION: Turn Magnetorquer2 on with full strength output. This is accomplished by using a
 *              constant HIGH GPIO signal instead of a PWM signal.
 */
void Magnetorquer2_Full_Strength();

/*
 * FUNCTION: Magnetorquer3_Full_Strength
 *
 * DESCRIPTION: Turn Magnetorquer3 on with full strength output. This is accomplished by using a
 *              constant HIGH GPIO signal instead of a PWM signal.
 */
void Magnetorquer3_Full_Strength();

/*
 * FUNCTION: Magnetorquer1_Off
 *
 * DESCRIPTION: Turn Magnetorquer1 off.
 */
void Magnetorquer1_Off();

/*
 * FUNCTION: Magnetorquer2_Off
 *
 * DESCRIPTION: Turn Magnetorquer2 off.
 */
void Magnetorquer2_Off();

/*
 * FUNCTION: Magnetorquer3_Off
 *
 * DESCRIPTION: Turn Magnetorquer3 off.
 */
void Magnetorquer3_Off();

#endif /* HARDWARE_PERIPHERALS_INC_MAGNETORQUERS_DRIVER_H_ */
