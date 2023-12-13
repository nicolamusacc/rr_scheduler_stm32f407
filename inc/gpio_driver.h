#ifndef INC_GPIO_DRIVER_H_
#define INC_GPIO_DRIVER_H_

#include "STM32F407VGT6.h"


/* Enumberate for GPIO pin number */
typedef enum{
    GPIO_PIN_NB_0 = 0,
    GPIO_PIN_NB_1 = 1,
    GPIO_PIN_NB_2 = 2,
    GPIO_PIN_NB_3 = 3,
    GPIO_PIN_NB_4 = 4,
    GPIO_PIN_NB_5 = 5,
    GPIO_PIN_NB_6 = 6,
    GPIO_PIN_NB_7 = 7,
    GPIO_PIN_NB_8 = 8,
    GPIO_PIN_NB_9 = 9,
    GPIO_PIN_NB_10 = 10,
    GPIO_PIN_NB_11 = 11,
    GPIO_PIN_NB_12 = 12,
    GPIO_PIN_NB_13 = 13,
    GPIO_PIN_NB_14 = 14,
    GPIO_PIN_NB_15 = 15
} GPIO_PinNumber;

/* Enumerate for GPIO pin mode */
typedef enum{
    GPIO_IN_MODE = 0,
    GPIO_OUT_MODE = 1,
    GPIO_ALTFN_MODE = 2,
    GPIO_ANALOG_MODE = 3,
    GPIO_FT_IT_MODE = 4,       // falling edge detection
    GPIO_RT_IT_MODE = 5,       // raising edge detection
    GPIO_RFT_IT_MODE = 6
} GPIO_PinMode;

/* Enumerate for GPIO output pin speed */
typedef enum{
    GPIO_SPEED_LOW = 0,
    GPIO_SPEED_MEDIUM = 1,
    GPIO_SPEED_HIGH = 2,
    GPIO_SPEED_VHIGH = 3
} GPIO_PinOutSpeed;

/* Enumerate for GPIO output pin state (pull-up pull-down) */
typedef enum{
    GPIO_PIN_NOPUPD = 0,
    GPIO_PIN_PU = 1,
    GPIO_PIN_PD = 2,
    GPIO_PIN_RESERVED = 3
} GPIO_PinPuPdControl;

/* Enumerate for GPIO output pin type */
typedef enum{
    GPIO_OUT_PUSHPULL = 0,
    GPIO_OUT_OPENDRAIN = 1
} GPIO_PinOutType;


/* Enumerate for GPIO pin alternate function selector */
typedef enum{
    GPIO_AF0 = 0,
    GPIO_AF1 = 1,
    GPIO_AF2 = 2,
    GPIO_AF3 = 3,
    GPIO_AF4 = 4,
    GPIO_AF5 = 5,
    GPIO_AF6 = 6,
    GPIO_AF7 = 7,
    GPIO_AF8 = 8,
    GPIO_AF9 = 9,
    GPIO_AF10 = 10,
    GPIO_AF11 = 11,
    GPIO_AF12 = 12,
    GPIO_AF13 = 13,
    GPIO_AF14 = 14,
    GPIO_AF15 = 15
} GPIO_PinAltFunSelect;

/* Configuration structure for a GPIO pin */
typedef struct 
{
    GPIO_PinNumber PinNumber;
    GPIO_PinMode PinMode;
    GPIO_PinOutSpeed PinSpeed;
    GPIO_PinPuPdControl PinPuPdControl;
    GPIO_PinOutType PinOutType;
    GPIO_PinAltFunSelect PinAltFunSelect;
} GPIO_PinConf_t;


/* GPIO handler structure contains the base address of a GPIO port and the pin informations */
typedef struct
{
    GPIO_registers_t *pGPIO;        // base address of the GPIO port
    GPIO_PinConf_t GPIO_PinConf;    // pin configuration settings
} GPIO_handler_t;


#define AFR_LOW 0
#define AFR_HIGH 1


/***********************************
*         GPIO DRIVER API
***********************************/


/** @brief This function enables or disables peripheral clock for the given GPIO port
 *
 *  @param[in] pGPIOx: base address of the gpio peripheral
 *  @param[in] is_clk_enabled: CLK_ENABLE or CLK_DISABLE macros
 *  @return void 
 */
void GPIO_PeripheralClkControl(GPIO_registers_t *pGPIOx, uint8_t enable_clk);


/** @brief This function initialize a GPIO pin according to the handler configuration
 *
 *  @param[in] pGPIOHandler: pointer to the GPIO handler
 *  @return void
*/
void GPIO_Init(GPIO_handler_t *pGPIOHandler);


/** @brief This function resets the given GPIO port
 *
 *  @param[in] pGPIOx: base address of the gpio peripheral
 *  @return void
 */
void GPIO_Reset(GPIO_registers_t *pGPIOx);


/** @brief This function allows to read from an input pin of a gpio port
 *
 *  @param[in] pGPIOx: base address of the gpio peripheral
 *  @param[in] PinNumber: pin number
 *  @return pin value
 */
uint8_t GPIO_ReadFromInputPin(GPIO_registers_t *pGPIOx, uint8_t PinNumber);


/** @brief This function allows to read from an input gpio port
 *
 * @param[in] pGPIOx: base address of the gpio peripheral
 * @param[in] PinNumber: pin number
 * @return pin value
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_registers_t *pGPIOx);


/** @brief This function writes a value to the selected output pin
 *                      
 *
 * @param[in] pGPIOx: base address of the gpio peripheral
 * @param[in] value: data to be written
 * @return pin value
 */
void GPIO_WriteToOutputPin(GPIO_registers_t *pGPIOx, uint8_t PinNumber, uint8_t value);


/** @brief This function writes data to the selected output port
 *
 *  @param[in] pGPIOx: base address of the gpio peripheral
 *  @param[in] value: data to be written
 *  @return pin value
 *
 */
void GPIO_WriteToOutputPort(GPIO_registers_t *pGPIOx, uint16_t value);


/** @brief This function toggles the selected output pin
 *
 *  @param[in] pGPIOx: base address of the gpio peripheral
 *  @param[in] value: data to be written
 *  @return pin value
 *
 */
void GPIO_ToggleOutputPin(GPIO_registers_t *pGPIOx, uint8_t PinNumber);


/*
* IRQ config and ISR handling
*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQEnable);
void GPIO_IRQ_PriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandler(uint8_t PinNumber);





#endif
