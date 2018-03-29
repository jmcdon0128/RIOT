/**
 * @defgroup    drivers_LMT01 Digital Temperature Sensor
 * @ingroup     drivers_sensors
 * @brief       Device driver interface for the LMT01 sensor.
 * @{
 *
 * @file
 * @brief       Device driver interface for the LMT01 sensor.
 *
 * @author      Jason McDonald <jason.mcdonald@cumberlandgroupit.com>
 */

#ifndef LMT01_H
#define LMT01_H
#include "board.h"
#include "periph/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Valid temperature status
 */
typedef enum {
    LMT01_INVALID_TEMPERATURE = 0,
    LMT01_VALID_TEMPERATURE
} lmt01_valid_t;

/**
 * @brief   Device initialization parameters
 */
typedef struct {
    gpio_t read_pin; /**<GPIO pin connected to the LMT01 read pin*/
    gpio_t vcc_pin;  /**<GPIO pin connected to the LMT01 positive */
    gpio_t gnd_pin;  /**<GPIO pin connected to the LMT01 ground*/
} lmt01_params_t;

/**
 * @brief   Device descriptor for the LMT01 sensor
 */
typedef struct {
    lmt01_params_t params;    /**< Device initialization parameters */
    uint16_t pulse_count;     /**< number of pulses counted since last reset */
    uint16_t hold;            /**< number of pulses counted since last reset */
    int16_t last_temperature; /**< last valid temp ( needs divided by 16) */
} lmt01_t;

/**
 * @brief   Initialize LMT01
 *
 * @param[out] dev      device descriptor
 * @param[in]  params   device configuration
 *
 * @pre     @p dev != NULL
 */
int lmt01_init(lmt01_t *dev, const lmt01_params_t *params);

/**
 * @brief   Read temperature of given LMT01 device in d°C
 *
 * @param[in] dev    Device descriptor of LMT01 device to read from
 *
 * @return           Success or failure flag
 */
lmt01_valid_t lmt01_read_temperature(lmt01_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* LMT01_H */
       /** @} */
