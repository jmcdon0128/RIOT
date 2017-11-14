/**
 ******************************************************************************
 * @file    spsgrf_io.h
 * @author  MCD Application Team
 * @brief   This file contains the transport layer functions (SPI instance). 
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

/* Includes ------------------------------------------------------------------*/
#include "spsgrf_io.h"
#include "board.h"
#include "periph/gpio.h"
#include "periph/spi.h"



/**
 * @defgroup SPSGR_IO_GPIO_Private_Defines                SPSGR_GPIO Private Defines
 * @{
 */
#define POR_TIME ((uint16_t)0x1E00)

/**
 * @}
 */


/**
 * @defgroup SPSGRF_IO_SPI_Private_Constants             SPSGRF_IO_SPI Private Constants
 * @{
 */ 
/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...) */    
#define RADIO_SPI_TIMEOUT_MAX                   ((uint32_t)10000)

/* SPIRIT1_Spi_config_Private_Defines */  
#define CS_TO_SCLK_DELAY     0x0100
#define CLK_TO_CS_DELAY      0x0100


/* SPIRIT1_Spi_config_Headers */
#define HEADER_WRITE_MASK     0x00                                /*!< Write mask for header byte*/
#define HEADER_READ_MASK      0x01                                /*!< Read mask for header byte*/
#define HEADER_ADDRESS_MASK   0x00                                /*!< Address mask for header byte*/
#define HEADER_COMMAND_MASK   0x80                                /*!< Command mask for header byte*/

#define LINEAR_FIFO_ADDRESS   0xFF                                  /*!< Linear FIFO address*/

#define ALL_IRQ_ENABLE()  __enable_irq()
#define ALL_IRQ_DISABLE() __disable_irq()

/* SPIRIT1_Spi_config_Private_FunctionPrototypes */
#define SPI_ENTER_CRITICAL()  SPIRIT1_IRQ_DISABLE()   
#define SPI_EXIT_CRITICAL()   SPIRIT1_IRQ_ENABLE()    

/* SPIRIT1_Spi_config_Private_Functions */
#define SpiritSpiCSLow()        HAL_GPIO_WritePin(RADIO_SPI_CS_PORT, RADIO_SPI_CS_PIN, GPIO_PIN_RESET)
#define SpiritSpiCSHigh()       HAL_GPIO_WritePin(RADIO_SPI_CS_PORT, RADIO_SPI_CS_PIN, GPIO_PIN_SET)

/* SPIRIT1_Spi_config_Private_Macros */
#define BUILT_HEADER(add_comm, w_r) (add_comm | w_r)                             /*!< macro to build the header byte*/
#define WRITE_HEADER        BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_WRITE_MASK) /*!< macro to build the write 
                                                                                   header byte*/
#define READ_HEADER         BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_READ_MASK)  /*!< macro to build the read 
                                                                                   header byte*/
#define COMMAND_HEADER      BUILT_HEADER(HEADER_COMMAND_MASK, HEADER_WRITE_MASK) /*!< macro to build the command 
                                                                                   header byte*/
/**
 * @}
 */ 


/**
 * @defgroup SPSGRF_IO_SPI_Private_Variables              SPSGRF_IO_SPI Private Variables
 * @{
 */
uint32_t SpiTimeout = RADIO_SPI_TIMEOUT_MAX;                         /*<! Value of Timeout when SPI communication fails */
uint16_t tmpstatus;
static struct {
    spi_mode_t oper_mode;
    spi_clk_t clk_speed;
    spi_t board_id;
    spi_cs_t cs;
} spi_conf;

/**
 * @}
 */


/**
 * @defgroup SPSGRF_IO_SPI_Exported_Functions              SPSGRF_IO_SPI Exported Functions
 * @{
 */

/**
 * @brief  Initializes SPI HAL.
 * @param  None
 * @retval None
 */
void SpiritSpiInit(void)
{
    int tmp;
    spi_conf.board_id = SPI_DEV(0);
    spi_conf.oper_mode = SPI_MODE_0;
    spi_conf.clk_speed = SPI_CLK_10MHZ;
    spi_conf.cs = (spi_cs_t)SPSGRF_CSN;            
    /* test setup */
    tmp = spi_init_cs(spi_conf.board_id, spiconf.cs);
    if (tmp != SPI_OK) {
        DEBUG("error: unable to initialize the given chip select line");
    }
    tmp = spi_acquire(spi_conf.board_id, spi_conf.cs,
            spi_conf.oper_mode, spi_conf.clk_speed);
    if (tmp == SPI_NOMODE) {
        DEBUG("error: given SPI mode is not supported");
    }
    else if (tmp == SPI_NOCLK) {
        DEBUG("error: targeted clock speed is not supported");
    }
    else if (tmp != SPI_OK) {
        DEBUG("error: unable to acquire bus with given parameters");
    }
    spi_release(spi_conf.board_id);

}


/**
 * @brief  Write single or multiple RF Transceivers register
 * @param  cRegAddress: base register's address to be write
 * @param  cNbBytes: number of registers and bytes to be write
 * @param  pcBuffer: pointer to the buffer of values have to be written into registers
 * @retval StatusBytes
 */
uint8_t * SpiritSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
    uint8_t aHeader[2] = {0};
    uint8_t *pStatus=(uint8_t *)&tmpstatus;

    tmpstatus = 0x0000;
    /* Built the aHeader bytes */
    aHeader[0] = WRITE_HEADER;
    aHeader[1] = cRegAddress;

    spi_acquire(spi_conf.board_id, spi_conf.cs, 
            spi_conf.oper_mode, spi_conf.clk_speed);

    for (volatile uint16_t Index = 0; Index < CS_TO_SCLK_DELAY; Index++);

    /* Write the aHeader bytes and read the SPIRIT1 status bytes */
    tmpstatus = spi_transfer_reg(spi_conf.board_id, spi_conf.cs, 
            aHeader[0],NULL);
    /* Write the aHeader bytes and read the SPIRIT1 status bytes */
    tmpstatus = spi_transfer_reg(spi_conf.board_id, spi_conf.cs, 
            aHeader[1],NULL);

    /* Writes the registers according to the number of bytes */
    spi_transfer_bytes(spi_conf.board_id, spi_conf.cs,
            false, pcBuffer, NULL, cNbBytes);  

    spi_release(spi_conf.board_id);

    return pStatus;

}


/**
 * @brief  Read single or multiple SPIRIT1 register
 * @param  cRegAddress: base register's address to be read
 * @param  cNbBytes: number of registers and bytes to be read
 * @param  pcBuffer: pointer to the buffer of registers' values read
 * @retval StatusBytes
 */
uint8_t * SpiritSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
    uint8_t *pStatus = (uint8_t *)&tmpstatus;
    uint8_t aHeader[2] = {0};
    uint8_t dummy = 0xFF;

    tmpstatus = 0x0000;

    /* Built the aHeader bytes */
    aHeader[0] = READ_HEADER;
    aHeader[1] = cRegAddress;

    for (volatile uint16_t Index = 0; Index < CS_TO_SCLK_DELAY; Index++);

    /* Write the aHeader bytes and read the SPIRIT1 status bytes */
    HAL_SPI_TransmitReceive(&pSpiHandle, (uint8_t *)&aHeader[0], (uint8_t *)&(tmpstatus), 1, SpiTimeout);
    tmpstatus = tmpstatus << 8;  

    /* Write the aHeader bytes and read the SPIRIT1 status bytes */
    HAL_SPI_TransmitReceive(&pSpiHandle, (uint8_t *)&aHeader[1], (uint8_t *)&tmpstatus, 1, SpiTimeout);

    for (int index = 0; index < cNbBytes; index++)
    { 
        HAL_SPI_TransmitReceive(&pSpiHandle, (uint8_t *)&dummy, (uint8_t *)&(pcBuffer)[index], 1, SpiTimeout);
    } 

    /* To be sure to don't rise the Chip Select before the end of last sending */
    while (__HAL_SPI_GET_FLAG(&pSpiHandle, SPI_FLAG_TXE) == RESET);

    /* Put the SPI chip select high to end the transaction */
    SpiritSpiCSHigh();

    SPI_EXIT_CRITICAL();

    return pStatus;

}


/**
 * @brief  Send a command
 * @param  cCommandCode: command code to be sent
 * @retval StatusBytes
 */
uint8_t * SpiritSpiCommandStrobes(uint8_t cCommandCode)
{
    uint8_t aHeader[2] = {0};
    uint8_t *pStatus = (uint8_t *)&tmpstatus;

    tmpstatus = 0x0000;  
    /* Built the aHeader bytes */
    aHeader[0] = COMMAND_HEADER;
    aHeader[1] = cCommandCode;

    SPI_ENTER_CRITICAL();

    /* Puts the SPI chip select low to start the transaction */
    SpiritSpiCSLow();

    for (volatile uint16_t Index = 0; Index < CS_TO_SCLK_DELAY; Index++);
    /* Write the aHeader bytes and read the SPIRIT1 status bytes */
    HAL_SPI_TransmitReceive(&pSpiHandle, (uint8_t *)&aHeader[0], (uint8_t *)&tmpstatus, 1, SpiTimeout);
    tmpstatus = tmpstatus<<8;  

    /* Write the aHeader bytes and read the SPIRIT1 status bytes */
    HAL_SPI_TransmitReceive(&pSpiHandle, (uint8_t *)&aHeader[1], (uint8_t *)&tmpstatus, 1, SpiTimeout);

    /* To be sure to don't rise the Chip Select before the end of last sending */
    while (__HAL_SPI_GET_FLAG(&pSpiHandle, SPI_FLAG_TXE) == RESET);

    /* Puts the SPI chip select high to end the transaction */
    SpiritSpiCSHigh();

    SPI_EXIT_CRITICAL();

    return pStatus;

}


/**
 * @brief  Write data into TX FIFO
 * @param  cNbBytes: number of bytes to be written into TX FIFO
 * @param  pcBuffer: pointer to data to write
 * @retval StatusBytes
 */
uint8_t * SpiritSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
    uint8_t *pStatus = (uint8_t *)&tmpstatus;
    uint8_t aHeader[2] = {0};

    tmpstatus = 0x0000;
    /* Built the aHeader bytes */
    aHeader[0] = WRITE_HEADER;
    aHeader[1] = LINEAR_FIFO_ADDRESS;

    SPI_ENTER_CRITICAL();

    /* Put the SPI chip select low to start the transaction */
    SpiritSpiCSLow();

    for (volatile uint16_t Index = 0; Index < CS_TO_SCLK_DELAY; Index++);

    /* Write the aHeader bytes and read the SPIRIT1 status bytes */
    HAL_SPI_TransmitReceive(&pSpiHandle, (uint8_t *)&aHeader[0], (uint8_t *)&tmpstatus, 1, SpiTimeout);
    tmpstatus = tmpstatus<<8;  

    /* Write the aHeader bytes and read the SPIRIT1 status bytes */
    HAL_SPI_TransmitReceive(&pSpiHandle, (uint8_t *)&aHeader[1], (uint8_t *)&tmpstatus, 1, SpiTimeout);

    /* Writes the registers according to the number of bytes */
    for (int index = 0; index < cNbBytes; index++)
    {
        SPI_Write(pcBuffer[index]);
    }

    /* To be sure to don't rise the Chip Select before the end of last sending */
    while (__HAL_SPI_GET_FLAG(&pSpiHandle, SPI_FLAG_TXE) == RESET); 

    /* Put the SPI chip select high to end the transaction */
    SpiritSpiCSHigh();

    SPI_EXIT_CRITICAL();

    return pStatus; 
}

/**
 * @brief  Read data from RX FIFO
 * @param  cNbBytes: number of bytes to read from RX FIFO
 * @param  pcBuffer: pointer to data read from RX FIFO
 * @retval StatusBytes
 */
uint8_t * SpiritSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
    uint8_t *pStatus = (uint8_t *)&tmpstatus;
    uint8_t aHeader[2];
    uint8_t dummy=0xFF;

    tmpstatus = 0x0000;
    /* Built the aHeader bytes */
    aHeader[0]=READ_HEADER;
    aHeader[1]=LINEAR_FIFO_ADDRESS;

    SPI_ENTER_CRITICAL();

    /* Put the SPI chip select low to start the transaction */
    SpiritSpiCSLow();

    for (volatile uint16_t Index = 0; Index < CS_TO_SCLK_DELAY; Index++);

    /* Write the aHeader bytes and read the SPIRIT1 status bytes */
    HAL_SPI_TransmitReceive(&pSpiHandle, (uint8_t *)&aHeader[0], (uint8_t *)&tmpstatus, 1, SpiTimeout);
    tmpstatus = tmpstatus<<8;  

    /* Write the aHeader bytes and read the SPIRIT1 status bytes */
    HAL_SPI_TransmitReceive(&pSpiHandle, (uint8_t *)&aHeader[1], (uint8_t *)&tmpstatus, 1, SpiTimeout);

    for (int index = 0; index < cNbBytes; index++)
    { 
        HAL_SPI_TransmitReceive(&pSpiHandle, (uint8_t *)&dummy, (uint8_t *)&pcBuffer[index], 1, SpiTimeout);
    } 

    /* To be sure to don't rise the Chip Select before the end of last sending */
    while(__HAL_SPI_GET_FLAG(&pSpiHandle, SPI_FLAG_TXE) == RESET);

    /* Put the SPI chip select high to end the transaction */
    SpiritSpiCSHigh();

    SPI_EXIT_CRITICAL();

    return pStatus;  
}

/**
 * @}
 */


/**
 * @defgroup SPSGR_IO_GPIO_Exported_Functions              SPSGR_GPIO Exported Functions
 * @{
 */


/**
 * @brief  Configures MCU GPIO and EXTI Line for GPIOs.
 * @retval None.
 */
void SpiritGpioInit()
{
    /* Initialize the SDN pin micro side */
    gpio_init((gpio_t)SPSGRF_SDN ,GPIO_OUT);
    /* Micro EXTI config */      
    gpio_init_int(SPSGRF_GPIO3_EXTI5, GPIO_IN, GPIO_FALLING, NULL, NULL);

    gpio_irq_disable(SPSGRF_GPIO3_EXTI5);
    gpio_irq_enable(SPSGRF_GPIO3_EXTI5);
    //TODO: Set interupt priority pins.
} 
}



/**
 * @brief  Returns the level of a specified GPIO.
 * @param  xGpio Specifies the GPIO to be read.
 *         This parameter can be one of following parameters:
 *         @arg GPIO_0
 *         @arg GPIO_1
 *         @arg GPIO_2
 *         @arg GPIO_3
 * @retval FlagStatus Level of the GPIO. This parameter can be:
 *         SET or RESET.
 */
FlagStatus SpiritGpioGetLevel(gpio_t xGpio)
{
    /* Gets the GPIO level */
    return gpio_read(xGpio);
}


/**
 * @brief  Sets the level of a specified GPIO.
 * @param  xGpio Specifies the GPIO to be set.
 *         This parameter can be one of following parameters:
 *         @arg GPIO_0
 *         @arg GPIO_1
 *         @arg GPIO_2
 *         @arg GPIO_3
 * @param  GPIO_PinState Level of the GPIO. This parameter can be:
 *         GPIO_PIN_SET or GPIO_PIN_RESET.
 * @retval None.
 */
void SpiritGpioSetLevel(gpio_t xGpio, int xState)
{
    /* Sets the GPIO level */
    gpio_write(xGpio, xState); 
}


/**
 * @brief  Puts at logic 1 the SDN pin.
 * @param  None.
 * @retval None.
 */
void SpiritEnterShutdown(void)
{
    /* Puts high the GPIO connected to shutdown pin */
    /* Check the parameters */ 
    gpio_set(SPSGRF_SDN); 
}


/**
 * @brief  Put at logic 0 the SDN pin.
 * @param  None.
 * @retval None.
 */
void SpiritExitShutdown(void)
{
    /* Puts low the GPIO connected to shutdown pin */
    gpio_clear(SPSGRF_SDN); 

    /* Delay to allow the circuit POR, about 700 us */
    for (volatile uint32_t Index = 0; Index < POR_TIME; Index++);
}


/**
 * @brief  check the logic(0 or 1) at the SDN pin.
 * @param  None.
 * @retval FlagStatus.
 */
FlagStatus SpiritCheckShutdown(void)
{
    return SpiritGpioGetLevel(RADIO_GPIO_SDN);
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
