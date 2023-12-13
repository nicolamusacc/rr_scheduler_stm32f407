#include "led.h"


void LedConfInit(GPIO_PinNumber led_pin_nb)
{
	GPIO_handler_t GPIO_Led_Handler;

    GPIO_Led_Handler.pGPIO = GPIOD;
    GPIO_Led_Handler.GPIO_PinConf.PinNumber = led_pin_nb;
    GPIO_Led_Handler.GPIO_PinConf.PinMode = GPIO_OUT_MODE;
    GPIO_Led_Handler.GPIO_PinConf.PinSpeed = GPIO_SPEED_HIGH;
    GPIO_Led_Handler.GPIO_PinConf.PinPuPdControl = GPIO_PIN_NOPUPD;
    GPIO_Led_Handler.GPIO_PinConf.PinOutType = GPIO_OUT_PUSHPULL;

    GPIO_PeripheralClkControl(GPIO_Led_Handler.pGPIO, ENABLED);
    GPIO_Init(&GPIO_Led_Handler);
}


void AllLedConfInit(void)
{
	LedConfInit(GPIO_PIN_NB_12);
	LedConfInit(GPIO_PIN_NB_13);
	LedConfInit(GPIO_PIN_NB_14);
	LedConfInit(GPIO_PIN_NB_15);
	
}


void LedToggle(GPIO_PinNumber led_pin_nb)
{   

	GPIO_ToggleOutputPin(GPIOD, led_pin_nb);

}

