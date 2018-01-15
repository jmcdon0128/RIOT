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
/**
 * Modified by Jason McDonald <jason.mcdonald@cumberlandgroupit.com> to work with stm32l475 running the RIOT OS
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "spsgrf_io.h"
#include "board.h"
#include "periph/gpio.h"
#include "periph/spi.h"
#define ENABLE_DEBUG (1)
#include "debug.h"


/**
 * @defgroup SPSGR_GPIO Private Defines
 */
#define POR_TIME ((uint16_t)0xFE00)

/* Maximum Timeout values for flags waiting loops. These timeouts are 
 * not based on accurate values, they just guarantee that the application
 * will not remain stuck if the SPI communication is corrupted.
 * You may modify these timeout values depending on CPU frequency and 
 * application conditions (interrupts routines ...) */    
#define RADIO_SPI_TIMEOUT_MAX  ((uint32_t)10000)

/* SPIRIT1_Spi_config_Private_Defines */  
#define CS_TO_SCLK_DELAY     0x0500
#define CLK_TO_CS_DELAY      0x0500


/* SPIRIT1_Spi_config_Headers */
#define HEADER_WRITE_MASK     0x00 /*!< Write mask for header byte*/
#define HEADER_READ_MASK      0x01 /*!< Read mask for header byte*/
#define HEADER_ADDRESS_MASK   0x00 /*!< Address mask for header byte*/
#define HEADER_COMMAND_MASK   0x80 /*!< Command mask for header byte*/

#define LINEAR_FIFO_ADDRESS   0xFF /*!< Linear FIFO address*/

/* SPIRIT1_Spi_config_Private_Macros */
/* macros for building spi headers to sesnd to spirit module */
#define BUILT_HEADER(add_comm, w_r) (add_comm | w_r) 

#define WRITE_HEADER        BUILT_HEADER(\
        HEADER_ADDRESS_MASK,\
        HEADER_WRITE_MASK)

#define READ_HEADER         BUILT_HEADER(\
        HEADER_ADDRESS_MASK,\
        HEADER_READ_MASK)  

#define COMMAND_HEADER      BUILT_HEADER(\
        HEADER_COMMAND_MASK,\
        HEADER_WRITE_MASK) 
/**
 * @defgroup SPSGRF_IO_SPI Private Variables
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
 * @defgroup SPSGRF_IO_SPI Exported Functions
 */
void SPI_TEST(void)
{
    uint8_t bench_wbuf[10];
    uint8_t bench_rbuf[10];

    memset(bench_wbuf, 0xAA, 10);

    spi_acquire(spi_conf.board_id, spi_conf.cs,
            spi_conf.oper_mode, spi_conf.clk_speed);
    DEBUG("---Running SPI Test---\n");
    for (int i=0; i < 1000; i++) 
    {
        spi_transfer_bytes(spi_conf.board_id, 
                spi_conf.cs, false,bench_wbuf, bench_rbuf, 10);
        for (int j = 0; j< 10; j++)
        {
            if (bench_rbuf[j] != 0xFF)
                DEBUG("READ IN 0x%x\n", bench_rbuf[j]);
        }
    }
    spi_release(spi_conf.board_id);
    DEBUG("---DONE Running SPI Test---\n");
}

/**
 * @brief  Initializes SPI HAL.
 * @param  None
 * @retval None
 */
void RadioSpiInit(void)
{
    int tmp;
    DEBUG("RadioSpiInit\n");
    spi_conf.board_id = SPI_DEV(0);
    spi_conf.oper_mode = SPI_MODE_0;
    spi_conf.clk_speed = SPI_CLK_400KHZ;
    spi_conf.cs = (spi_cs_t)SPSGRF_CSN;            
    /* test setup */
    tmp = spi_init_cs(spi_conf.board_id, spi_conf.cs);
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
 * @brief  write the Spirit header and register address to the 
 *          spirit module AND aquire the spi line
 * @param  Header: the type of spi message being sent/recieved 
 *          (WRITE, READ, ..)
 * @param  RegisterAddress: the address ofhte regster that needs read 
 *          or writen to
 * @retval statusbytes
 */
void _SPIWriteHead(uint8_t Header, uint8_t RegisterAddress )
{
    uint8_t Head[2] = {Header, RegisterAddress};
    tmpstatus = 0x0000;

    spi_acquire(spi_conf.board_id, spi_conf.cs, 
            spi_conf.oper_mode, spi_conf.clk_speed);

    for (volatile uint16_t Index = 0; Index < CS_TO_SCLK_DELAY; Index++);

    spi_transfer_bytes(spi_conf.board_id, spi_conf.cs,
            true, Head, &tmpstatus, 2);  

    /* Write the aHeader bytes and read the SPIRIT1 status bytes */
    /*tmpstatus = spi_transfer_byte(spi_conf.board_id, spi_conf.cs,
            false, Header);
    tmpstatus  <<= 8;
     Write the aHeader bytes and read the SPIRIT1 status bytes 
    tmpstatus |= spi_transfer_byte(spi_conf.board_id, spi_conf.cs,
            false, RegisterAddress);
    */
}

/**
 * @brief  write single or multiple rf transceivers register
 * @param  cregaddress: base register's address to be write
 * @param  cnbbytes: number of registers and bytes to be write
 * @param  pcbuffer: pointer to the buffer of values have to be written into registers
 * @retval statusbytes
 */
uint8_t * RadioSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{

    uint8_t *pStatus=(uint8_t *)&tmpstatus;

    _SPIWriteHead(WRITE_HEADER, cRegAddress);
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
uint8_t * RadioSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
    uint8_t *pStatus=(uint8_t *)&tmpstatus;
    uint8_t dummy[cNbBytes];
    memset(dummy, 0xFF, sizeof(dummy));

    _SPIWriteHead(READ_HEADER, cRegAddress);

    spi_transfer_bytes(spi_conf.board_id, spi_conf.cs,
            false, dummy, pcBuffer, cNbBytes);  

    for (volatile uint16_t Index = 0; Index < CS_TO_SCLK_DELAY; Index++);

    spi_release(spi_conf.board_id);
    return pStatus;

}


/**
 * @brief  Send a command
 * @param  cCommandCode: command code to be sent
 * @retval StatusBytes
 */
uint8_t * RadioSpiCommandStrobes(uint8_t cCommandCode)
{
    //uint8_t aHeader[2] = {0};
    uint8_t *pStatus=(uint8_t *)&tmpstatus;

    _SPIWriteHead(COMMAND_HEADER, cCommandCode);
    spi_release(spi_conf.board_id);
    return pStatus;

}


/**
 * @brief  Write data into TX FIFO
 * @param  cNbBytes: number of bytes to be written into TX FIFO
 * @param  pcBuffer: pointer to data to write
 * @retval StatusBytes
 */
uint8_t * RadioSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
    uint8_t *pStatus=(uint8_t *)&tmpstatus;

    _SPIWriteHead(WRITE_HEADER, LINEAR_FIFO_ADDRESS);
    spi_transfer_bytes(spi_conf.board_id, spi_conf.cs,
            false, pcBuffer, NULL, cNbBytes);  
    spi_release(spi_conf.board_id);
    return pStatus; 
}

/**
 * @brief  Read data from RX FIFO
 * @param  cNbBytes: number of bytes to read from RX FIFO
 * @param  pcBuffer: pointer to data read from RX FIFO
 * @retval StatusBytes
 */
uint8_t * RadioSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
    uint8_t *pStatus=(uint8_t *)&tmpstatus;
    uint8_t dummy = 0xFF;

    _SPIWriteHead(WRITE_HEADER, LINEAR_FIFO_ADDRESS);
    spi_transfer_bytes(spi_conf.board_id, spi_conf.cs,
            false, &dummy, pcBuffer, cNbBytes);  
    spi_release(spi_conf.board_id);
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
 * @brief  Configures MCU SPIRIT GPIO and EXTI Line for GPIOs.
 * @retval None.
 */
void RadioGpioInit(void)
{
    /* Initialize the SDN pin micro side */
    DEBUG("Initalizing Gpio of radio\n");
    gpio_init((gpio_t)SPSGRF_SDN ,GPIO_OUT);
    /* Micro EXTI config */      
   // gpio_init_int(SPSGRF_GPIO3_EXTI5, GPIO_IN, GPIO_FALLING, NULL, NULL);

    //gpio_irq_disable(SPSGRF_GPIO3_EXTI5);
    //gpio_irq_enable(SPSGRF_GPIO3_EXTI5);
    //TODO: Set interupt priority pins.
}


/**
 * @brief  Returns the level of a specified GPIO.
 * @param  xGpio Specifies the GPIO to be read.
 * @retval FlagStatus Level of the GPIO. This parameter can be:
 *         SET or RESET.
 */
FlagStatus RadioGpioGetLevel(gpio_t xGpio)
{
    return gpio_read(xGpio);
}


/**
 * @brief  Sets the level of a specified GPIO.
 * @param  xGpio Specifies the GPIO to be set.
 * @param  GPIO_PinState Level of the GPIO. This parameter can be:
 *         GPIO_PIN_SET or GPIO_PIN_RESET.
 * @retval None.
 */
void RadioGpioSetLevel(gpio_t xGpio, GPIO_PinState xState)
{
    /* Sets the GPIO level */
    gpio_write(xGpio, xState); 
}


/**
 * @brief  Puts at logic 1 the SDN pin.
 * @param  None.
 * @retval None.
 */
void RadioEnterShutdown(void)
{
    /* Puts high the GPIO connected to shutdown pin */
    gpio_set(SPSGRF_SDN); 
}


/**
 * @brief  Put at logic 0 the SDN pin.
 * @param  None.
 * @retval None.
 */
void RadioExitShutdown(void)
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
FlagStatus RadioCheckShutdown(void)
{
    return RadioGpioGetLevel(SPSGRF_SDN);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
