/**
 * @ingroup     drivers_lmt01
 * @{
 *
 * @file
 * @brief       SAUL adoption for LMT1 sensor.
 *
 * @author      Jason McDonald <jason.mcdonald@cumberlandgroupit.com>
 *
 * @}
 */

#include "saul.h"

#include "lmt01.h"

static int read_temperature(const void *dev, phydat_t *res) {
    if (lmt01_read_temperature((lmt01_t *)dev) ==
        LMT01_VALID_TEMPERATURE) {
        res->val[0] = (uint16_t)(((lmt01_t *)dev)->last_temperature);
        res->unit = UNIT_TEMP_C;
        res->scale = 0;
        return LMT01_VALID_TEMPERATURE;
    }

    return LMT01_INVALID_TEMPERATURE;
}

const saul_driver_t lmt01_temperature_saul_driver = {
    .read = read_temperature, .write = saul_notsup, .type = SAUL_SENSE_TEMP};
