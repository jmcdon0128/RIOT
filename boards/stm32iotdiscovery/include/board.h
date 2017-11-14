/**
 * @defgroup    boards_stm32iotdiscovery stm32 IoT Discovery Board 
 * @ingroup     boards
 * @brief       Board specific files for the  stm32 iot discovery board
 * @{
 *
 * @file
 * @brief       Board specific definitions for the  stm32 iot discovery board
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
#define SPSGRF_SDN            GPIO_PIN(PORT_B, 15)
#define SPSGRF_CSN            GPIO_PIN(PORT_B, 5 )

/**
 * @brief SPBTLE Pin definitions 
 */
#define SPBTLE_RF_IRQ_EXTI6       GPIO_PIN(PORT_E, 6 )
#define SPBTLE_RF_SPI3_CSN        GPIO_PIN(PORT_D, 13)
#define SPBTLE_RF_RST             GPIO_PIN(PORT_A, 8 )

#define M24SR64_Y_RF_DISABLE      GPIO_PIN(PORT_E, 2 )
#define M24SR64_Y_GPO             GPIO_PIN(PORT_E, 4 )


#define USB_OTG_OVRCR_EXTI3       GPIO_PIN(PORT_E, 3 )
#define USB_OTG_FS_PWR_EN         GPIO_PIN(PORT_D, 12)
#define USB_OTG_FS_VBUS           GPIO_PIN(PORT_A, 9 )
#define USB_OTG_FS_ID             GPIO_PIN(PORT_A, 10)
#define USB_OTG_FS_DM             GPIO_PIN(PORT_A, 11)
#define USB_OTG_FS_DP             GPIO_PIN(PORT_A, 12)


#define ISM43362_RST              GPIO_PIN(PORT_E, 8 )
#define ISM43362_BOOT0            GPIO_PIN(PORT_B, 12)
#define ISM43362_WAKEUP           GPIO_PIN(PORT_B, 13)
#define ISM43362_SPI3_CSN         GPIO_PIN(PORT_E, 0 )
#define ISM43362_DRDY_EXTI1       GPIO_PIN(PORT_E, 1 )


#define QUADSPI_CLK               GPIO_PIN(PORT_E, 10)
#define QUADSPI_NCS               GPIO_PIN(PORT_E, 11)
#define QUADSPI_BK1_IO0           GPIO_PIN(PORT_E, 12)
#define QUADSPI_BK1_IO1           GPIO_PIN(PORT_E, 13)
#define QUADSPI_BK1_IO2           GPIO_PIN(PORT_E, 14)
#define QUADSPI_BK1_IO3           GPIO_PIN(PORT_E, 15)


#define PMOD_RESET                GPIO_PIN(PORT_D, 0 )
#define PMOD_SPI2_SCK             GPIO_PIN(PORT_D, 1 )
#define PMOD_IRQ_EXTI2            GPIO_PIN(PORT_D, 2 )
#define PMOD_UART2_CTS_SPI2_MISO  GPIO_PIN(PORT_D, 3 )
#define PMOD_UART2_RTS_SPI2_MOSI  GPIO_PIN(PORT_D, 4 )
#define PMOD_UART2_TX_SPI2_CSN    GPIO_PIN(PORT_D, 5 )
#define PMOD_UART2_RX             GPIO_PIN(PORT_D, 6 )


#define INTERNAL_I2C2_SCL         GPIO_PIN(PORT_B, 10)
#define INTERNAL_I2C2_SDA         GPIO_PIN(PORT_B, 11)
#define INTERNAL_UART3_TX         GPIO_PIN(PORT_D, 8 )
#define INTERNAL_UART3_RX         GPIO_PIN(PORT_D, 9 )
#define INTERNAL_SPI3_SCK         GPIO_PIN(PORT_C, 10)
#define INTERNAL_SPI3_MISO        GPIO_PIN(PORT_C, 11)
#define INTERNAL_SPI3_MOSI        GPIO_PIN(PORT_C, 12)


#define SYS_JTMS_SWDIO            GPIO_PIN(PORT_A, 13)
#define SYS_JTCK_SWCLK            GPIO_PIN(PORT_A, 14)
#define SYS_JTDO_SWO              GPIO_PIN(PORT_B, 3 )


#define RCC_OSC32_IN              GPIO_PIN(PORT_C, 14)
#define RCC_OSC32_OUT             GPIO_PIN(PORT_C, 15)
#define RCC_OSC_IN                GPIO_PIN(PORT_H, 0 )
#define RCC_OSC_OUT               GPIO_PIN(PORT_H, 1 )


#define DFSDM1_DATIN2             GPIO_PIN(PORT_E, 7 )
#define DFSDM1_CKOUT              GPIO_PIN(PORT_E, 9 )


#define VL53L0X_XSHUT             GPIO_PIN(PORT_C, 6 )
#define VL53L0X_GPIO1_EXTI7       GPIO_PIN(PORT_C, 7 )


#define ST_LINK_UART1_TX          GPIO_PIN(PORT_B, 6 )
#define ST_LINK_UART1_RX          GPIO_PIN(PORT_B, 7 )


#define LPS22HB_INT_DRDY_EXTI10   GPIO_PIN(PORT_D, 10)
#define LSM6DSL_INT1_EXTI11       GPIO_PIN(PORT_D, 11)
#define HTS221_DRDY_EXTI15        GPIO_PIN(PORT_D, 15)
#define LIS3MDL_DRDY_EXTI8        GPIO_PIN(PORT_C, 8 )


#define BUTTON_EXTI13             GPIO_PIN(PORT_C, 13)
#define LED2                      GPIO_PIN(PORT_B, 14)
#define LED4_LED3                 GPIO_PIN(PORT_C, 9 )
#define STSAFE_A100_RESET         GPIO_PIN(PORT_D, 7 )


#define ARDA5_ADC                GPIO_PIN(PORT_C, 0 )
#define ARDA4_ADC                GPIO_PIN(PORT_C, 1 )
#define ARDA3_ADC                GPIO_PIN(PORT_C, 2 )
#define ARDA2_ADC                GPIO_PIN(PORT_C, 3 )
#define ARDD1_UART4_TX           GPIO_PIN(PORT_A, 0 )
#define ARDD0_UART4_RX           GPIO_PIN(PORT_A, 1 )
#define ARDD10_SPI_SSN_PWM       GPIO_PIN(PORT_A, 2 )
#define ARDD4                    GPIO_PIN(PORT_A, 3 )
#define ARDD7                    GPIO_PIN(PORT_A, 4 )
#define ARDD13_SPI1_SCK_LED1     GPIO_PIN(PORT_A, 5 )
#define ARDD12_SPI1_MISO         GPIO_PIN(PORT_A, 6 )
#define ARDD11_SPI1_MOSI_PWM     GPIO_PIN(PORT_A, 7 )
#define ARDA1_ADC                GPIO_PIN(PORT_C, 4 )
#define ARDA0_ADC                GPIO_PIN(PORT_C, 5 )
#define ARDD3_PWM_INT1_EXTI0     GPIO_PIN(PORT_B, 0 )
#define ARDD6_PWM                GPIO_PIN(PORT_B, 1 )
#define ARDD8                    GPIO_PIN(PORT_B, 2 )
#define ARDD2_INT0_EXTI14        GPIO_PIN(PORT_D, 14)
#define ARDD9_PWM                GPIO_PIN(PORT_A, 15)
#define ARDD5_PWM                GPIO_PIN(PORT_B, 4 )
#define ARDD15_I2C1_SCL          GPIO_PIN(PORT_B, 8 )
#define ARDD14_I2C1_SDA          GPIO_PIN(PORT_B, 9 )

/**
 * @brief Initialize board specific hardware, including clock, LEDs and std_IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
