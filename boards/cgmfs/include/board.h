/**
 * @defgroup    boards_cgmfs Board 
 * @ingroup     boards
 * @brief       Board specific files for the  cgmfs board
 * @{
 *
 * @file
 * @brief       Board specific definitions for the  cgmfs board
 *
 * @author     Jason McDonald <jason.mcdonald@cumberlandgroupit.com >
 */


#ifndef BOARD_H
#define BOARD_H

#include "cpu.h"
#include "periph_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SPSGRF Pin definitions 
 * @{
 */
#define SPSGRF_GPIO3_EXTI5    GPIO_PIN(PORT_E, 5 )
#define SPSGRF_GPIO3_EXTI2    GPIO_PIN(PORT_A, 2 )
#define SPSGRF_SDN            GPIO_PIN(PORT_B, 15)
#define SPSGRF_CSN            GPIO_PIN(PORT_B, 5 )

/**
 * @brief SPBTLE Pin definitions 
 */
#define SPBTLE_RF_IRQ_EXTI6       GPIO_PIN(PORT_E, 6 )
#define SPBTLE_RF_SPI3_CSN        GPIO_PIN(PORT_D, 13)
#define SPBTLE_RF_RST             GPIO_PIN(PORT_A, 8 )

/**
 *  @brief usb pins
 */
#define USB_N                     GPIO_PIN(PORT_A, 11)
#define USB_P                     GPIO_PIN(PORT_A, 12)

/**
 * @brief wifi module pin definitions
 */
#define ISM43362_RST              GPIO_PIN(PORT_E, 8 )
#define ISM43362_BOOT0            GPIO_PIN(PORT_B, 12)
#define ISM43362_WAKEUP           GPIO_PIN(PORT_B, 13)
#define ISM43362_SPI3_CSN         GPIO_PIN(PORT_E, 0 )
#define ISM43362_DRDY_EXTI1       GPIO_PIN(PORT_E, 1 )

/**
 * @brief quad spi pin definitions
 */
#define QUADSPI_CLK               GPIO_PIN(PORT_E, 10)
#define QUADSPI_NCS               GPIO_PIN(PORT_E, 11)
#define QUADSPI_BK1_IO0           GPIO_PIN(PORT_E, 12)
#define QUADSPI_BK1_IO1           GPIO_PIN(PORT_E, 13)
#define QUADSPI_BK1_IO2           GPIO_PIN(PORT_E, 14)
#define QUADSPI_BK1_IO3           GPIO_PIN(PORT_E, 15)

/**
 * @brief expansion port pins
 */
#define EXP_ARDD5_PWM                GPIO_PIN(PORT_B, 4 )
#define EXP_ARDA5_ADC                GPIO_PIN(PORT_C, 0 )
#define EXP_ARDD15_I2C1_SCL          GPIO_PIN(PORT_B, 8 )
#define EXP_ARDD14_I2C1_SDA          GPIO_PIN(PORT_B, 9 )
#define EXP_RESET                    GPIO_PIN(PORT_D, 0 )
#define EXP_SPI2_SCK                 GPIO_PIN(PORT_D, 1 )
#define EXP_IRQ_EXTI2                GPIO_PIN(PORT_D, 2 )
#define EXP_SPI2_MISO                GPIO_PIN(PORT_D, 3 )
#define EXP_SPI2_MOSI                GPIO_PIN(PORT_D, 4 )
#define EXP_SPI2_CSN                 GPIO_PIN(PORT_D, 5 )
#define EXP_USART1_TX                GPIO_PIN(PORT_B, 6 )
#define EXP_USART1_RX                GPIO_PIN(PORT_B, 7 )

/*
 * @brief internal bus pins 
 */
#define INTERNAL_I2C2_SCL         GPIO_PIN(PORT_B, 10)
#define INTERNAL_I2C2_SDA         GPIO_PIN(PORT_B, 11)
#define INTERNAL_UART3_TX         GPIO_PIN(PORT_D, 8 )
#define INTERNAL_UART3_RX         GPIO_PIN(PORT_D, 9 )
#define INTERNAL_SPI3_SCK         GPIO_PIN(PORT_C, 10)
#define INTERNAL_SPI3_MISO        GPIO_PIN(PORT_C, 11)
#define INTERNAL_SPI3_MOSI        GPIO_PIN(PORT_C, 12)

/**
 * @brief JTAG interface pins
 */
#define SYS_JTMS_SWDIO            GPIO_PIN(PORT_A, 13)
#define SYS_JTCK_SWCLK            GPIO_PIN(PORT_A, 14)
#define SYS_JTCK_T1               GPIO_PIN(PORT_A, 15)
#define SYS_JTDO_SWO              GPIO_PIN(PORT_B, 3 )

/**
 * @brief LMT1 temperature sensor pins
 */
#define LMT_VP                   GPIO_PIN(PORT_C, 4 )
#define LMT_VN                   GPIO_PIN(PORT_C, 5 )
#define LMT_VNN                  GPIO_PIN(PORT_B, 2 )
#define LMT_VNN_WAKE             GPIO_PIN(PORT_A, 0 )

/** 
 * @brief vcc enable pins
 */
#define WIFI_VCC_EN                   GPIO_PIN(PORT_C, 6 )
#define FLASH_VCC_EN                  GPIO_PIN(PORT_C, 7 )

/** 
 * @brief led pin definitions 
 */
#define LED_BT                        GPIO_PIN(PORT_C, 8)
#define LED_WIFI                      GPIO_PIN(PORT_C, 9)
//TODO rgb led


/** 
 * @brief button pin definitions
 */
#define BUTTON_EXTI13             GPIO_PIN(PORT_C, 13)


/** 
 * @brief external Osc pins
 */
#define RCC_OSC32_IN              GPIO_PIN(PORT_C, 14)
#define RCC_OSC32_OUT             GPIO_PIN(PORT_C, 15)


/**
 * @brief Initialize board specific hardware, including clock, LEDs and std_IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
