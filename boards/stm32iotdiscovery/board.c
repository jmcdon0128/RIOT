/*
 * Copyright (C) 2017 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_iotdiscovery-l475
 * @{
 *
 * @file
 * @brief       Board specific implementations for the iotdiscovery-l475
 *
 * @author      
 *
 * @}
 */

#include "board.h"
#include "periph/gpio.h"
#include "periph/spi.h"

void board_init(void)
{
    /* initialize the CPU */
    cpu_init();
    /* Uncomment to initialize leds */
    //gpio_init(LED0_PIN, GPIO_OUT);
    //gpio_init(LED1_PIN, GPIO_OUT);

}
