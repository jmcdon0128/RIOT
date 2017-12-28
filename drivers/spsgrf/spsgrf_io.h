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
//#include "stm32l4xx.h"
#include "board.h"
#include "periph/gpio.h"

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

  

 
/**
 * @defgroup SPSGRF_IO_SPI_Exported_FunctionPrototypes             SPSGRF_IO_SPI Exported FunctionPrototypes
 * @{
 */
 
/* Radio SPI Exported functions ------------------------------------------------------- */
void SpiritSpiInit(void);
uint8_t * SpiritSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
uint8_t * SpiritSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
uint8_t * SpiritSpiCommandStrobes(uint8_t cCommandCode);
uint8_t * SpiritSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer);
uint8_t * SpiritSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer);

/**
 * @}
 */

/**
 * @defgroup SPSGR_IO_GPIO_Exported_Functions             SPSGR_GPIO Exported Functions
 * @{
 */
 
/* GPIO Exported functions ------------------------------------------------------- */
/*FlagStatus SpiritGpioGetLevel(gpio_t xGpio);
void SpiritGpioSetLevel(gpio_t xGpio, gpio_mode_t xState);
void SpiritEnterShutdown(void);
void SpiritExitShutdown(void);
FlagStatus SpiritCheckShutdown(void);
void SpiritGpioInit(gpio_t xGpio, gpio_mode_t xGpioMode);
void SpiritGpioInterruptCmd(gpio_t xGpio, uint8_t nPreemption, uint8_t nSubpriority, FunctionalState xNewState);
*/
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
