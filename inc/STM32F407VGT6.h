/* 
 * STM32F407VGT6.h contains 
 *  - peripheral and memory base addresses specific
 *    to the STM32F407VGT6 microcontroller.
 *  - peripheral registers types and macros to manipulate them
 */

#ifndef INC_STM32F407VGT6_H_
#define INC_STM32F407VGT6_H_

#include<stdint.h>

/* Resources used for scheduling project */
#define HSI_CLK 			16000000U
#define SYSTICK_TIM_CLK 	HSI_CLK
#define TICK_HZ				1000U


#define INTERRUPT_ENABLE() do{__asm volatile("MOV R0, #0x1"); __asm volatile("MSR PRIMASK, R0"); }while(0)
#define INTERRUPT_DISABLE() do{__asm volatile("MOV R0, #0x0"); __asm volatile("MSR PRIMASK, R0"); }while(0)

/* ARM Cortex-Mx Processor NVIC ISERx register addresses */
#define NVIC_ISER_BASE_ADDR ((volatile uint32_t*)0xE000E100)
#define NVIC_ISER0_ADDR     (NVIC_ISER_BASE_ADDR)
#define NVIC_ISER1_ADDR     (NVIC_ISER_BASE_ADDR + 0x04)
#define NVIC_ISER2_ADDR     (NVIC_ISER_BASE_ADDR + 0x08)

/* ARM Cortex-Mx Processor NVIC ICERx register addresses */
#define NVIC_ICER_BASE_ADDR ((volatile uint32_t*)0XE000E180)
#define NVIC_ICER0_ADDR     (NVIC_ICER_BASE_ADDR)
#define NVIC_ICER1_ADDR     (NVIC_ICER_BASE_ADDR + 0x04)
#define NVIC_ICER2_ADDR     (NVIC_ICER_BASE_ADDR + 0x08)

/* ARM Cortex-Mx Processor NVIC IPRx register addresses */
#define NVIC_IPR_BASE_ADDR ((NVIC_IPR_registers_t*)0XE000E400)

/* ARM Cortex-Mx Processor NVIC IPRx registers definition */
typedef struct 
{
    uint32_t volatile IPR[60];
} NVIC_IPR_registers_t;


/* SRAM and Flash base addresses */
#define FLASH_BASE_ADDR     (0x08000000U)
#define SRAM1_BASE_ADDR     (0x20000000U)
#define SRAM2_BASE_ADDR     (0x20001C00U) 
#define ROM                 (0x1FFF0000U)
#define SRAM_BASE_ADDR      (SRAM1_BASE_ADDR)

/* AHBx and APBx Bus Peripherals base addresses */
#define PERIPH_BASE_ADDR    (0x40000000U)
#define APB1_BASE_ADDR      (PERIPH_BASE_ADDR)
#define APB2_BASE_ADDR      (0x40010000U)
#define AHB1_BASE_ADDR      (0x40020000U)
#define AHB2_BASE_ADDR      (0x50000000U)

/* Peripheral hanging on AHB1 bus */
#define GPIOA_BASE_ADDR      (AHB1_BASE_ADDR)
#define GPIOB_BASE_ADDR      (AHB1_BASE_ADDR + 0x0400)
#define GPIOC_BASE_ADDR      (AHB1_BASE_ADDR + 0x0800)
#define GPIOD_BASE_ADDR      (AHB1_BASE_ADDR + 0x0C00)
#define GPIOE_BASE_ADDR      (AHB1_BASE_ADDR + 0x1000)
#define GPIOF_BASE_ADDR      (AHB1_BASE_ADDR + 0x1400)
#define GPIOG_BASE_ADDR      (AHB1_BASE_ADDR + 0x1800)
#define GPIOH_BASE_ADDR      (AHB1_BASE_ADDR + 0x1C00)
#define GPIOI_BASE_ADDR      (AHB1_BASE_ADDR + 0x2000)
#define RCC_BASE_ADDR        (AHB1_BASE_ADDR + 0x3800)

/* Peripheral hanging on APB1 bus */
#define I2C1_BASE_ADDR      (APB1_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR      (APB1_BASE_ADDR + 0x5800)
#define I2C3_BASE_ADDR      (APB1_BASE_ADDR + 0x5C00)

#define SPI2_BASE_ADDR      (APB1_BASE_ADDR + 0x3800)
#define SPI3_BASE_ADDR      (APB1_BASE_ADDR + 0x3C00)

#define USART2_BASE_ADDR    (APB1_BASE_ADDR + 0x4400)
#define USART3_BASE_ADDR    (APB1_BASE_ADDR + 0x4800)
#define UART4_BASE_ADDR     (APB1_BASE_ADDR + 0x4C00)
#define UART5_BASE_ADDR     (APB1_BASE_ADDR + 0x5000)

/* Peripheral hanging on APB2 bus */
#define EXTI_BASE_ADDR      (APB2_BASE_ADDR + 0x3C00)
#define SPI1_BASE_ADDR      (APB2_BASE_ADDR + 0x3000)
#define SYSCFG_BASE_ADDR    (APB2_BASE_ADDR + 0x3800)
#define USART1_BASE_ADDR    (APB2_BASE_ADDR + 0x1000)
#define USART6_BASE_ADDR    (APB2_BASE_ADDR + 0x1400)

/* Peripheral registers data structures */


typedef struct
{
    uint32_t volatile MODER;
    uint32_t volatile OTYPER;
    uint32_t volatile OSPEEDR;
    uint32_t volatile PUPDR;
    uint32_t volatile IDR;
    uint32_t volatile ODR;
    uint32_t volatile BSRR;
    uint32_t volatile LCKR;
    uint32_t volatile AFRL;
    uint32_t volatile AFRH;
} GPIO_registers_t;

typedef struct
{
    uint32_t volatile CR;
    uint32_t volatile PLLCFGR;
    uint32_t volatile CFGR;
    uint32_t volatile CIR;
    uint32_t volatile AHB1RSTR;
    uint32_t volatile AHB2RSTR;
    uint32_t volatile AHB3RSTR;
    uint32_t volatile RESERVED0;
    uint32_t volatile APB1RSTR;
    uint32_t volatile APB2RSTR;
    uint32_t volatile RESERVED1;
    uint32_t volatile RESERVED2;
    uint32_t volatile AHB1ENR;
    uint32_t volatile AHB2ENR;
    uint32_t volatile AHB3ENR;
    uint32_t volatile RESERVED3;
    uint32_t volatile APB1ENR;
    uint32_t volatile APB2ENR;
    uint32_t volatile RESERVED4;
    uint32_t volatile RESERVED5;
    uint32_t volatile AHB1PENR;
    uint32_t volatile AHB2PENR;
    uint32_t volatile AHB3PENR;
    uint32_t volatile RESERVED6;
    uint32_t volatile RESERVED7;
    uint32_t volatile BDCR;
    uint32_t volatile CSR;
    uint32_t volatile RESERVED8;
    uint32_t volatile RESERVED9;
    uint32_t volatile PLLI2SCFGR;
    uint32_t volatile PLLSAICFGR;
    uint32_t volatile DCKCFGR;
} RCC_registers_t;

typedef struct 
{
    uint32_t volatile IMR;
    uint32_t volatile EMR;
    uint32_t volatile RTSR;
    uint32_t volatile FTSR;
    uint32_t volatile SWIER;
    uint32_t volatile PR;
} EXTI_registers_t;

typedef struct 
{
    uint32_t volatile MEMRMP;
    uint32_t volatile PMC;
    uint32_t volatile EXTICR[4];
    uint32_t RESERVED1;
    uint32_t RESERVED2;
    uint32_t volatile CMPCR;
}SYSCFG_registers_t;


/* Peripheral definition: pointers to the gpio register data structure */
#define GPIOA   ( (GPIO_registers_t*)GPIOA_BASE_ADDR)
#define GPIOB   ( (GPIO_registers_t*)GPIOB_BASE_ADDR)
#define GPIOC   ( (GPIO_registers_t*)GPIOC_BASE_ADDR)
#define GPIOD   ( (GPIO_registers_t*)GPIOD_BASE_ADDR)
#define GPIOE   ( (GPIO_registers_t*)GPIOE_BASE_ADDR)
#define GPIOF   ( (GPIO_registers_t*)GPIOF_BASE_ADDR)
#define GPIOG   ( (GPIO_registers_t*)GPIOG_BASE_ADDR)
#define GPIOH   ( (GPIO_registers_t*)GPIOH_BASE_ADDR)
#define GPIOI   ( (GPIO_registers_t*)GPIOI_BASE_ADDR)

#define RCC     ( (RCC_registers_t*)RCC_BASE_ADDR)
#define EXTI    ( (EXTI_registers_t*)EXTI_BASE_ADDR)
#define SYSCFG  ( (SYSCFG_registers_t*)SYSCFG_BASE_ADDR)

/* Clock enable/disable macros for GPIOx peipherals */
#define GPIOA_CLK_EN()    ( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_CLK_EN()    ( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_CLK_EN()    ( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_CLK_EN()    ( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_CLK_EN()    ( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_CLK_EN()    ( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_CLK_EN()    ( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_CLK_EN()    ( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_CLK_EN()    ( RCC->AHB1ENR |= (1 << 8) )

#define GPIOB_CLK_DIS()    ( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOA_CLK_DIS()    ( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOC_CLK_DIS()    ( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_CLK_DIS()    ( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_CLK_DIS()    ( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_CLK_DIS()    ( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_CLK_DIS()    ( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_CLK_DIS()    ( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_CLK_DIS()    ( RCC->AHB1ENR &= ~(1 << 8) )

/* Clock enable/disable macros for I2Cx peippherals */
#define I2C1_CLK_EN()    ( RCC->APB1ENR |= (1 << 21) )
#define I2C2_CLK_EN()    ( RCC->APB1ENR |= (1 << 22) )
#define I2C3_CLK_EN()    ( RCC->APB1ENR |= (1 << 23) )

#define I2C1_CLK_DIS()    ( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_CLK_DIS()    ( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_CLK_DIS()    ( RCC->APB1ENR &= ~(1 << 23) )

/* Clock enable/disable macros for SPIx peippherals */
#define SPI1_CLK_EN()    ( RCC->APB2ENR |= (1 << 12) )
#define SPI2_CLK_EN()    ( RCC->APB1ENR |= (1 << 14) )
#define SPI3_CLK_EN()    ( RCC->APB1ENR |= (1 << 15) )

#define SPI1_CLK_DIS()    ( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_CLK_DIS()    ( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_CLK_DIS()    ( RCC->APB1ENR &= ~(1 << 15) )


/* Clock enable/disable macros for USARTx peippherals */


/* Clock enable/disable macros for SYSCFGx peippherals */
#define SYSCFG_CLK_EN()     ( RCC->APB2ENR |= (1 << 14) )


/* This macro return a code between 0 and 8 according to input GPIO port.
 * This port code is needed to configure the SYSCFG EXTICR registers
 * Ref 9.2 SYSCFG registers for STM32F405xx/07xx and STM32F415xx/17xx  
 */ 
#define GPIOADDR_TO_SYSCFG_PCODE(x)   ( (x == GPIOA) ? 0 :\
                                        (x == GPIOB) ? 1 :\
                                        (x == GPIOC) ? 2 :\
                                        (x == GPIOD) ? 3 :\
                                        (x == GPIOE) ? 4 :\
                                        (x == GPIOF) ? 5 :\
                                        (x == GPIOG) ? 6 :\
                                        (x == GPIOH) ? 7 :\
                                        (x == GPIOI) ? 8 : 0 )
                                            
/* Reset macros GPIOx peipherals */
#define GPIOA_RST()    do{ (RCC->AHB1RSTR |= (1 << 0)); ( RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_RST()    do{ (RCC->AHB1RSTR |= (1 << 1)); ( RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_RST()    do{ (RCC->AHB1RSTR |= (1 << 2)); ( RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define GPIOD_RST()    do{ (RCC->AHB1RSTR |= (1 << 3)); ( RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define GPIOE_RST()    do{ (RCC->AHB1RSTR |= (1 << 4)); ( RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define GPIOF_RST()    do{ (RCC->AHB1RSTR |= (1 << 5)); ( RCC->AHB1RSTR &= ~(1 << 5));} while(0)
#define GPIOG_RST()    do{ (RCC->AHB1RSTR |= (1 << 6)); ( RCC->AHB1RSTR &= ~(1 << 6));} while(0)
#define GPIOH_RST()    do{ (RCC->AHB1RSTR |= (1 << 7)); ( RCC->AHB1RSTR &= ~(1 << 7));} while(0)
#define GPIOI_RST()    do{ (RCC->AHB1RSTR |= (1 << 8)); ( RCC->AHB1RSTR &= ~(1 << 8));} while(0)


/* 
 * IRQ numbers for STM32F407x MCU
 * #TODO complete with IRQ numbers of other peripheral 
 */
#define IRQ_NB_EXTI0        (6)
#define IRQ_NB_EXTI1        (7)
#define IRQ_NB_EXTI2        (8)
#define IRQ_NB_EXTI3        (9)
#define IRQ_NB_EXTI4        (10)
#define IRQ_NB_EXTI9_5      (23)
#define IRQ_NB_EXTI15_10    (40)


/* Additional macros */
#define ENABLED 1
#define DISABLED 0
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define IRQ_ENABLE 1
#define IRQ_DISABLE 0


#endif /* INC_STM32F407VGT6_H_ */
