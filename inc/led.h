/*
 * led.h
 *
 *  Created on: 28-Dec-2019
 *      Author: nieki
 */

#ifndef LED_H_
#define LED_H_

#include<stdint.h>
#include "gpio_driver.h"
#include "STM32F407VGT6.h"


/** @brief The funciton initializes the configuration of the input GPIO pin controlling
 *         one of the 4 LEDs on the board 
 *
 *  @param[in] led_pin_nb  
 *  @return void
 */
void LedConfInit(GPIO_PinNumber led_pin_nb);


/** @brief The funciton initializes the configuration of the 4 GPIO pin controlling
 *         the LEDs
 *
 *  @return void
 */
void AllLedConfInit(void);


/** @brief The function toggles the input GPIO pin controlling one of the 4 LEDs
 *         on the board 
 *
 *  @param[in] led_pin_nb
 *  @return void
 */
void LedToggle(GPIO_PinNumber led_pin_nb); 


#endif /* LED_H_ */
