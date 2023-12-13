#include "gpio_driver.h"


void GPIO_PeripheralClkControl(GPIO_registers_t *pGPIOx, uint8_t is_clk_enabled)
{
    if (is_clk_enabled == ENABLED){
        if(pGPIOx == GPIOA){
            GPIOA_CLK_EN();
        }else if (pGPIOx == GPIOB){
            GPIOB_CLK_EN();
        }else if (pGPIOx == GPIOC){
            GPIOC_CLK_EN();
        }else if (pGPIOx == GPIOD){
            GPIOD_CLK_EN();
        }else if (pGPIOx == GPIOE){
            GPIOE_CLK_EN();
        }else if (pGPIOx == GPIOF){
            GPIOF_CLK_EN();
        }else if (pGPIOx == GPIOG){
            GPIOG_CLK_EN();
        }else if (pGPIOx == GPIOH){
            GPIOH_CLK_EN();
        }else if (pGPIOx == GPIOI){
            GPIOI_CLK_EN();
        }
    }
    else{
        if(pGPIOx == GPIOA){
            GPIOA_CLK_DIS();
        }else if (pGPIOx == GPIOB){
            GPIOB_CLK_DIS();
        }else if (pGPIOx == GPIOC){
            GPIOC_CLK_DIS();
        }else if (pGPIOx == GPIOD){
            GPIOD_CLK_DIS();
        }else if (pGPIOx == GPIOE){
            GPIOE_CLK_DIS();
        }else if (pGPIOx == GPIOF){
            GPIOF_CLK_DIS();
        }else if (pGPIOx == GPIOG){
            GPIOG_CLK_DIS();
        }else if (pGPIOx == GPIOH){
            GPIOH_CLK_DIS();
        }else if (pGPIOx == GPIOI){
            GPIOI_CLK_DIS();
        }
    }
}



void GPIO_Init(GPIO_handler_t *pGPIOHandler)
{   
    GPIO_PinNumber pin_nb;
    uint32_t pin_mode;
    uint32_t pin_speed;
    uint32_t pin_out_type;
    uint32_t pin_pupd_ctrl;
    uint32_t pin_altfun;
    uint8_t altfun_reg_selector;

    pin_nb = pGPIOHandler->GPIO_PinConf.PinNumber;

    if (pGPIOHandler->GPIO_PinConf.PinMode <= GPIO_ANALOG_MODE)
    { 
        // Configure pin mode
        pin_mode = ( pGPIOHandler->GPIO_PinConf.PinMode << (2*pin_nb) );
        pGPIOHandler->pGPIO->MODER &= ~( 0x3 << (2*pin_nb) );  //clearing
        pGPIOHandler->pGPIO->MODER |= pin_mode;
    }
    else {
        if (pGPIOHandler->GPIO_PinConf.PinMode == GPIO_FT_IT_MODE)
        {
            // configure EXTI_FTSR register
            EXTI->FTSR |= (1 << pin_nb);
            EXTI->RTSR &= ~(1 << pin_nb);   // clear RTSR

        } else if (pGPIOHandler->GPIO_PinConf.PinMode == GPIO_RT_IT_MODE) {
            
            // configure EXTI_RTSR register 
            EXTI->RTSR |= (1 << pin_nb);
            EXTI->FTSR &= ~(1 << pin_nb);   // clear FTSR

        } else if (pGPIOHandler->GPIO_PinConf.PinMode == GPIO_RFT_IT_MODE) {
            // configue both EXTI_FTSR and EXTI_RTSR
            EXTI->FTSR |= (1 << pin_nb);
            EXTI->RTSR |= (1 << pin_nb);
        }

        // configure the GPIO port selection in SYSCFG_EXTICRx register
        SYSCFG_CLK_EN();
        uint8_t exti_reg = pin_nb / 4;
        uint8_t exti_pin = pin_nb % 4;
        uint8_t port_code = GPIOADDR_TO_SYSCFG_PCODE(pGPIOHandler->pGPIO);
        
        SYSCFG->EXTICR[exti_reg] |= (port_code << 4*exti_pin);

        // configure the EXTI_IMR register to enable the EXTI IT delivery
        EXTI->IMR |= (1 << pin_nb);
    } 

    // Configure pin speed
    pin_speed = ( pGPIOHandler->GPIO_PinConf.PinSpeed << (2*pin_nb) );
    pGPIOHandler->pGPIO->OSPEEDR &= ~( 0x3 << (2*pin_nb) );  //clearing
    pGPIOHandler->pGPIO->OSPEEDR |= pin_speed;

    // Configure pull-up pull-down state
    pin_pupd_ctrl = ( pGPIOHandler->GPIO_PinConf.PinPuPdControl << (2*pin_nb) );
    pGPIOHandler->pGPIO->PUPDR &= ~( 0x3 << (2*pin_nb) );  //clearing
    pGPIOHandler->pGPIO->PUPDR |= pin_pupd_ctrl;

    // Configure output type
    pin_out_type = ( pGPIOHandler->GPIO_PinConf.PinOutType << (pin_nb) );
    pGPIOHandler->pGPIO->OTYPER &= ~( 0x1 << (pin_nb) );  //clearing
    pGPIOHandler->pGPIO->OTYPER |= pin_out_type;

    // alt function
    if(pGPIOHandler->GPIO_PinConf.PinMode == GPIO_ALTFN_MODE)
    {
        pin_altfun = ( pGPIOHandler->GPIO_PinConf.PinAltFunSelect << ((4*pin_nb) % 8) );
        // select LOW [pin 0-7] or HIGH [pin 8-15] register
        // TODO create a macro for 8 as MAX_PIN_NUMB_AFR
        altfun_reg_selector = pGPIOHandler->GPIO_PinConf.PinNumber / 8;
        if (altfun_reg_selector == AFR_LOW)
        {
            pGPIOHandler->pGPIO->AFRL &= ~( 0xF << ((4*pin_nb) % 8) );  //clearing
            pGPIOHandler->pGPIO->AFRL |= pin_altfun;
        }
        else{
            pGPIOHandler->pGPIO->AFRH &= ~( 0xF << ((4*pin_nb) % 8) );  //clearing
            pGPIOHandler->pGPIO->AFRH |= pin_altfun;
        }
        
    }
}



void GPIO_Reset(GPIO_registers_t *pGPIOx)
{
    if(pGPIOx == GPIOA){
        GPIOA_RST();
    }else if (pGPIOx == GPIOB){
        GPIOB_RST();
    }else if (pGPIOx == GPIOC){
        GPIOC_RST();
    }else if (pGPIOx == GPIOD){
        GPIOD_RST();
    }else if (pGPIOx == GPIOE){
        GPIOE_RST();
    }else if (pGPIOx == GPIOF){
        GPIOF_RST();
    }else if (pGPIOx == GPIOG){
        GPIOG_RST();
    }else if (pGPIOx == GPIOH){
        GPIOH_RST();
    }else if (pGPIOx == GPIOI){
        GPIOI_RST();
    }
};



uint8_t GPIO_ReadFromInputPin(GPIO_registers_t *pGPIOx, uint8_t PinNumber)
{   
    uint8_t pin_value;
    pin_value = (uint8_t)( (pGPIOx->IDR >> PinNumber) & 0x00000001 );

    return pin_value;
}



uint16_t GPIO_ReadFromInputPort(GPIO_registers_t *pGPIOx)
{
    uint16_t port_value;
    port_value = (uint16_t)(pGPIOx->IDR);

    return port_value;
}



void GPIO_WriteToOutputPin(GPIO_registers_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
    if (value == GPIO_PIN_SET)
    {
        pGPIOx->ODR |= (uint32_t)(value << PinNumber); 
    }
    else {
        pGPIOx->ODR &= ~(uint32_t)(value << PinNumber); 
    }
}



void GPIO_WriteToOutputPort(GPIO_registers_t *pGPIOx, uint16_t value)
{
    pGPIOx->ODR = (uint32_t) value;
}



void GPIO_ToggleOutputPin(GPIO_registers_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}



/*
* IRQ config and ISR handling
*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQEnable)
{
    if (IRQEnable == IRQ_ENABLE)
    {
        if (IRQNumber <= 31)
        {
            // Enable interrupts in NVIC_ISER0 register (IRQ0 to IRQ31)
            *NVIC_ISER0_ADDR |= (1 << IRQNumber);
        }else if (IRQNumber > 31 && IRQNumber <= 63)
        {
            // Enable interrupts in NVIC_ISER1 register (IRQ32 to IRQ63)
            *NVIC_ISER1_ADDR |= (1 << (IRQNumber % 32) );
        }else if (IRQNumber > 64 && IRQNumber <= 95)
        {
            // Enable interrupts in NVIC_ISER2 register (IRQ64 to IRQ95)
            *NVIC_ISER2_ADDR |= (1 << (IRQNumber % 64) );
        }
    }
    else
    {
        if (IRQNumber <= 31)
        {
            // Disable interrupts in NVIC_ICER0 register (IRQ0 to IRQ31)
            *NVIC_ICER0_ADDR |= (1 << IRQNumber);
        }else if (IRQNumber > 31 && IRQNumber <= 63)
        {
            // Disable interrupts in NVIC_ICER1 register (IRQ32 to IRQ63)
            *NVIC_ICER1_ADDR |= (1 << (IRQNumber % 32) );
        }else if (IRQNumber > 64 && IRQNumber <= 95)
        {
            // Disable interrupts in NVIC_ICER2 register (IRQ64 to IRQ95)
            *NVIC_ICER2_ADDR |= (1 << (IRQNumber % 64) );
        }
    }
}



void GPIO_IRQ_PriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
    uint8_t iprx_reg_idx = IRQNumber / 4;
    uint8_t iprx_field = (IRQNumber % 4);
    NVIC_IPR_BASE_ADDR->IPR[iprx_reg_idx] |= (IRQPriority << (8 * iprx_field) ); 
}



void GPIO_IRQHandler(uint8_t PinNumber);

