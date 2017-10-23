/*
 * Copyright (C) 2017 Freie Universität Berlin
 *               2017 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_ 
 * @ingroup     boards_
 * @brief       Board specific files for the  board
 * @{
 *
 * @file
 * @brief       Board specific definitions for the  board
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 */

#ifndef BOARD_H
#define BOARD_H

//#include "board_common.h"
#include "cpu.h"
#include "periph_conf.h"
//#include "arduino_pinmap.h"

#ifdef __cplusplus
extern "C" {
#endif
#define LED0_PORT           GPIOA
#define LED0_PIN            GPIO_PIN(PORT_A, 5)
#define LED0_MASK           (1 << 5)
#define LED0_ON             (LED0_PORT->BSRR = LED0_MASK)
#define LED0_OFF            (LED0_PORT->BSRR = (LED0_MASK << 16))
#define LED0_TOGGLE         (LED0_PORT->ODR  ^= LED0_MASK)

#define LED1_PORT           GPIOB
#define LED1_PIN            GPIO_PIN(PORT_B, 14)
#define LED1_MASK           (1 << 14)
#define LED1_ON             (LED1_PORT->BSRR = LED1_MASK)
#define LED1_OFF            (LED1_PORT->BSRR = (LED1_MASK << 16))
#define LED1_TOGGLE         (LED1_PORT->ODR  ^= LED1_MASK)

/** @} */

/**
 * @brief   User button
 * @{
 */
#define BTN0_PIN            GPIO_PIN(PORT_C, 13)
#define BTN0_MODE           GPIO_IN_PU

/**
 * @brief Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
/** @} */
