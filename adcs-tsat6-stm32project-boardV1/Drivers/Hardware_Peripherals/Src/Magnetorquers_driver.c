/*
 * FILENAME: Magnetorquers_driver.c
 *
 * DESCRIPTION: STM32L4 driver source file for the 3 Magnetorquers.
 *
 * AUTHORS:
 *  - Daigh Burgess (daigh.burgess@umsats.ca)
 *
 * CREATED ON: May 18, 2023
 */

//###############################################################################################
//Include Directives
//###############################################################################################
#include "stm32l4xx_hal.h"
#include "Magnetorquers_driver.h"

//###############################################################################################
//Public Driver Functions
//###############################################################################################
void Magnetorquers_Init()
{
    HAL_GPIO_WritePin(PWM1_GPIO, PWM1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PWMSW1_GPIO, PWMSW1_PIN, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(PWM2_GPIO, PWM2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PWMSW2_GPIO, PWMSW2_PIN, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(PWM3_GPIO, PWM3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PWMSW3_GPIO, PWMSW3_PIN, GPIO_PIN_RESET);
}

void Magnetorquer1_Forward()
{
    HAL_GPIO_WritePin(PWMSW1_GPIO, PWMSW1_PIN, GPIO_PIN_SET);
}

void Magnetorquer2_Forward()
{
    HAL_GPIO_WritePin(PWMSW2_GPIO, PWMSW2_PIN, GPIO_PIN_SET);
}

void Magnetorquer3_Forward()
{
    HAL_GPIO_WritePin(PWMSW3_GPIO, PWMSW3_PIN, GPIO_PIN_SET);
}

void Magnetorquer1_Reverse()
{
    HAL_GPIO_WritePin(PWMSW1_GPIO, PWMSW1_PIN, GPIO_PIN_RESET);
}

void Magnetorquer2_Reverse()
{
    HAL_GPIO_WritePin(PWMSW2_GPIO, PWMSW2_PIN, GPIO_PIN_RESET);
}

void Magnetorquer3_Reverse()
{
    HAL_GPIO_WritePin(PWMSW3_GPIO, PWMSW3_PIN, GPIO_PIN_RESET);
}

void Magnetorquer1_Full_Strength()
{
    HAL_GPIO_WritePin(PWM1_GPIO, PWM1_PIN, GPIO_PIN_SET);
}

void Magnetorquer2_Full_Strength()
{
    HAL_GPIO_WritePin(PWM2_GPIO, PWM2_PIN, GPIO_PIN_SET);
}

void Magnetorquer3_Full_Strength()
{
    HAL_GPIO_WritePin(PWM3_GPIO, PWM3_PIN, GPIO_PIN_SET);
}

void Magnetorquer1_Off()
{
    HAL_GPIO_WritePin(PWM1_GPIO, PWM1_PIN, GPIO_PIN_RESET);
}

void Magnetorquer2_Off()
{
    HAL_GPIO_WritePin(PWM2_GPIO, PWM2_PIN, GPIO_PIN_RESET);
}

void Magnetorquer3_Off()
{
    HAL_GPIO_WritePin(PWM3_GPIO, PWM3_PIN, GPIO_PIN_RESET);
}
