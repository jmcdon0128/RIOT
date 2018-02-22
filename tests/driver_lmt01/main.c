/*
 * Copyright (C) 2014 Cumberlandgroup It
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup tests
 * @{
 *
 * @file
 * @brief       Test application for the LMT01 driver
 *
 * @author      Jason McDonald <jason.mcdonald@cumberlandgroupit.com>
 *
 * @}
 */

#include <stdio.h>

#include "lmt01.h"
#include "lmt01_params.h"
#include "xtimer.h"

#define READ_DELAY (1) /**> How long to wait in between reads in seconds */

lmt01_t temp_sensor;

int main(void) {
    puts("lmt01 test application\n");

    /* setup temperature device */
    lmt01_setup(&temp_sensor, &(lmt01_params[0]));

    while (1) {
        if (lmt01_read_temperature(&temp_sensor)) {
            printf("Temperature in Celsius    :%d\n",
                   temp_sensor.last_temperature);
        } else {
            printf("ERROR: Invalid temperature reading\n");
        }
        xtimer_sleep(READ_DELAY);
    }
    return 0;
}
