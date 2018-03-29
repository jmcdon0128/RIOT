/**
 ** @ingroup     auto_init_saul
 ** @{
 **
 ** @file
 ** @brief       Auto initialization of LMT01 driver.
 **
 ** @author      Jason McDonald <jason.mcdonald@cumberlandgroupit.com>
 **
 ** @}
 **/

#ifdef MODULE_LMT01

#include "lmt01.h"
#include "lmt01_params.h"
#include "log.h"
#include "saul_reg.h"

/**
 *  * @brief   Define the number of configured sensors
 *   */
#define LMT01_NUMOF (sizeof(lmt01_params) / sizeof(lmt01_params[0]))

/**
 *  * @brief   Allocation of memory for device descriptors
 *   */
static lmt01_t lmt01_devs[LMT01_NUMOF];

/**
 *  * @brief   Memory for the SAUL registry entries
 *   */
static saul_reg_t saul_entries[LMT01_NUMOF];

/**
 *  * @brief   Define the number of saul info
 *   */
#define LMT01_INFO_NUMOF \
    (sizeof(lmt01_saul_reg_info) / sizeof(lmt01_saul_reg_info[0]))

/**
 *  * @brief   Reference the driver structs.
 *   * @{
 *    */
extern const saul_driver_t lmt01_temperature_saul_driver;
/** @} */

void auto_init_lmt01(void) {
    assert(LMT01_INFO_NUMOF == LMT01_NUMOF);

    for (unsigned i = 0; i < LMT01_NUMOF; i++) {
        LOG_DEBUG("[auto_init_saul] initializing LMT01 #%u\n", i);

        if (lmt01_init(&lmt01_devs[i], &lmt01_params[i]) == LMT01_OK) {
            saul_entries[i].dev = &lmt01_devs[i];
            saul_entries[i].name = lmt01_saul_reg_info[i].name;
            saul_entries[i].driver = &lmt01_temperature_saul_driver;
            saul_reg_add(&saul_entries[i]);
        } else {
            LOG_ERROR("[auto_init_saul] error initializing LMT01 #%i\n", i);
        }
    }
}

#else
typedef int dont_be_pedantic;
#endif /* MODULE_LMT01 */
