/**
 * @ingroup     drivers_lmt01
 *
 * @{
 * @file
 * @brief       Default configuration for LMT01
 *
 * @author      Jason McDonald <jason.mcdonald@cumberlandgroupit.com>
 */

#ifndef LMT01_PARAMS_H
#define LMT01_PARAMS_H

#include "board.h"
#include "lmt01.h"
#include "saul_reg.h"

#ifdef __cplusplus
extern "C" {
#endif
//TODO: more sensable defaults can surely be found...
#define LMT01_PARAM_READ GPIO_PIN(0, 10)
#define LMT01_PARAM_VCC GPIO_PIN(0, 11)
#define LMT01_PARAM_GND GPIO_PIN(0, 12)
#define LMT01_PARAM_UNIT CELSIOUS

/**
 * @name    Set default configuration parameters for the LMT01
 * @{
 */
#ifndef LMT01_PARAMS_DEFAULT
#define LMT01_PARAMS_DEFAULT                                      \
    {                                                             \
        .read_pin = LMT01_PARAM_READ, .vcc_pin = LMT01_PARAM_VCC, \
        .gnd_pin = LMT01_PARAM_GND,                               \
    }
#endif

#ifndef LMT01_SAUL_INFO
#define LMT01_SAUL_INFO \
    { .name = "lmt01" }
#endif

/**@}*/

enum { LMT01_ERROR, LMT01_OK };

/**
 * @brief   Configure LMT01
 */
static const lmt01_params_t lmt01_params[] = {
#ifdef LMT01_PARAMS_BOARD
    LMT01_PARAMS_BOARD,
#else
    LMT01_PARAMS_DEFAULT,
#endif
};

/**
 *  * @brief   Configure SAUL registry entries
 *   */
static const saul_reg_info_t lmt01_saul_reg_info[] = {LMT01_SAUL_INFO};

#ifdef __cplusplus
}
#endif

#endif /* LMT01_PARAMS_H */
       /** @} */
