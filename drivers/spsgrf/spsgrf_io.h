/**
******************************************************************************
  * @file    spsgrf_io.h
  * @author  MCD Application Team
  * @brief   This file provides code for the configuration of all  GPIO pins 
             and the SPI instance used for Radio inetrface.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright © 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPSGRF_IO_H
#define __SPSGRF_IO_H

#ifdef __cplusplus
extern "C" {
#endif
  
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <board.h>


typedef enum
{
  RESET = 0,
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum
{
  DISABLE = 0,
  ENABLE = !DISABLE
} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum
{
  ERROR = 0,
  SUCCESS = !ERROR
} ErrorStatus;

typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
}GPIO_PinState;

/* Exported types ------------------------------------------------------------*/
  /* MCU GPIO pin working mode for GPIO */
/**
 * @defgroup SPSGR_IO_GPIO_Exported_Macros             SPSGR_GPIO Exported Macros
 * @{
 */  
 
/* Exported macro ------------------------------------------------------------*/
 /* MCU GPIO pin working mode for GPIO */
#define IS_RADIO_GPIO_MODE(MODE) (((MODE) == RADIO_MODE_GPIO_IN) || \
                               ((MODE) == RADIO_MODE_EXTI_IN) || \
                               ((MODE) == RADIO_MODE_GPIO_OUT))

/* Number of Arduino pins used for RADIO GPIO interface */
#define RADIO_GPIO_NUMBER    ((uint8_t)5)

/* MCU GPIO pin enumeration for GPIO */
#define IS_RADIO_GPIO_PIN(PIN)   (((PIN) == RADIO_GPIO_0) || \
                               ((PIN) == RADIO_GPIO_1) || \
                               ((PIN) == RADIO_GPIO_2) || \
                               ((PIN) == RADIO_GPIO_3) || \
                               ((PIN) == RADIO_GPIO_SDN))


/* @defgroup Radio_Gpio_config_Define */
/*NOTE: GPIO0, GPIO1, GPIO2 of SPIRIT1 are not used in the expansion board */
/******************************************************************************/
/*
#define RADIO_GPIO_3_PORT                        0
#define RADIO_GPIO_3_PIN                         0
#define RADIO_GPIO_3_CLOCK_ENABLE()                __HAL_RCC_GPIOA_CLK_ENABLE()
#define RADIO_GPIO_3_CLOCK_DISABLE()               __HAL_RCC_GPIOA_CLK_DISABLE() 
#define RADIO_GPIO_3_SPEED                       0
#define RADIO_GPIO_3_PUPD                        0
#define RADIO_GPIO_3_EXTI_LINE                   0
#define RADIO_GPIO_3_EXTI_MODE                   0
#define RADIO_GPIO_3_EXTI_PREEMPTION_PRIORITY    2
#define RADIO_GPIO_3_EXTI_SUB_PRIORITY           2

#define RADIO_GPIO_3_EXTI_IRQN                     EXTI0_IRQn 
#define RADIO_GPIO_3_EXTI_IRQ_HANDLER              EXTI0_IRQHandler
*/
/******************************************************************************/

/*
#define RADIO_GPIO_SDN_PORT                        GPIOB
#define RADIO_GPIO_SDN_PIN                         SPSGRF_SDN
#define RADIO_GPIO_SDN_CLOCK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define RADIO_GPIO_SDN_CLOCK_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()
#define RADIO_GPIO_SDN_SPEED                       GPIO_SPEED_FREQ_VERY_HIGH
#define RADIO_GPIO_SDN_PUPD                        GPIO_PULLUP

#define RADIO_GPIO_IRQ          RADIO_GPIO_3
#define SPIRIT_GPIO_IRQ         SPIRIT_GPIO_3
*/
/**
 * @}
 */

 
/**
 * @defgroup SPSGRF_IO_SPI_Exported_FunctionPrototypes             SPSGRF_IO_SPI Exported FunctionPrototypes
 * @{
 */
 
/* Radio SPI Exported functions ------------------------------------------------------- */
void RadioSpiInit(void);
uint8_t * RadioSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
uint8_t * RadioSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
uint8_t * RadioSpiCommandStrobes(uint8_t cCommandCode);
uint8_t * RadioSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer);
uint8_t * RadioSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer);

/**
 * @}
 */

/**
 * @defgroup SPSGR_IO_GPIO_Exported_Functions             SPSGR_GPIO Exported Functions
 * @{
 */
 
/* GPIO Exported functions ------------------------------------------------------- */
FlagStatus RadioGpioGetLevel(gpio_t xGpio);
void RadioGpioSetLevel(gpio_t xGpio, GPIO_PinState xState);
void RadioEnterShutdown(void);
void RadioExitShutdown(void);
FlagStatus RadioCheckShutdown(void);
void RadioGpioInit(void);
void RadioGpioInterruptCmd(gpio_t xGpio, uint8_t nPreemption, uint8_t nSubpriority, FunctionalState xNewState);

/**
 * @}
 */

/**
* @}
*/

#ifdef __cplusplus
}
#endif

#endif /*__SPSGRF_IO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
