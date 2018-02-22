/**
 * @ingroup     drivers_lmt01
 * @{
 *
  @file
 * @brief       Driver for the LMT01 ±0.5°C Accurate Temperature Sensor,
 *
 * @author      Jason McDonald <jason.mcdonald@cumberlandgroupit.com>
 *
 * @}
 */
#include "lmt01.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "byteorder.h"
#include "lmt01_params.h"
#include "xtimer.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#define PULSE_MIN (15) /**> Smallest valid pulse count */

static void count_pulse(lmt01_t *dev) { dev->pulse_count++; }

static uint16_t get_raw_pulse(lmt01_t *dev) {
    uint16_t pulses = 0;
    uint32_t time = xtimer_now_usec();

    dev->pulse_count = 0;
    gpio_set(dev->params.vcc_pin);
    while (pulses == 0 && (xtimer_now_usec() - time < 200000)) {
        if (dev->pulse_count) {
            do {
                dev->hold = dev->pulse_count;
                xtimer_usleep(1000);
            } while (dev->pulse_count != dev->hold);
            pulses = dev->pulse_count;
            dev->pulse_count = 0;
            dev->hold = 0;
            gpio_set(dev->params.vcc_pin);
        }
    }
    return pulses;
}

/**
 * @brief   setup LMT01 for operation
 *
 * @param[out] dev      device descriptor
 * @param[in]  params   device configuration
 *
 * @pre     @p dev != NULL
 */
void lmt01_setup(lmt01_t *dev, const lmt01_params_t *params) {
    if (!params) { /* if no params given use default */
        memcpy(&dev->params, lmt01_params, sizeof(lmt01_params_t));
    } else {
        memcpy(&dev->params, params, sizeof(lmt01_params_t));
    }

    /* init gpio for power and interrupts */
    gpio_init_int(dev->params.read_pin, GPIO_IN, GPIO_RISING,
                  (gpio_cb_t)count_pulse, dev);
    gpio_init(dev->params.vcc_pin, GPIO_OUT);

    /* init counters */
    dev->pulse_count = 0;
    dev->hold = 0;
}

/**
 *
 * @brief   Read temperature of given LMT01 device in d°C
 *
 * @param[in] dev    Device descriptor of LMT01 device to read from
*
* @return            Temperature in d°C
*/
lmt01_valid_t lmt01_read_temperature(lmt01_t *dev) {
    uint16_t pulses;

    if ((pulses = (double)get_raw_pulse(dev)) < PULSE_MIN)
        return LMT01_INVALID_TEMPERATURE;

    dev->last_temperature = (pulses - 800);

    return LMT01_VALID_TEMPERATURE;
}
