
/**
 * @ingroup     drivers_spsgrf
 * @{
 * @file
 * @brief       Netdev driver definitions for spsgrf driver
 *
 * @author      Jason McDonald <jason.mcdonald@cubmerlandgroupit.com>
 */

#ifndef SPSGRF_NETDEV_H
#define SPSGRF_NETDEV_H

#include "net/netdev.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Reference to the netdev device driver struct
 */
extern const netdev_driver_t spsgrf_driver;

/**
 * @brief   Received 6lowpan packet status information
 */
typedef struct netdev_radio_lora_packet_info {
    uint8_t rssi;           /**< RSSI of a received packet */
    uint8_t lqi;            /**< LQI of a received packet */
    int8_t snr;             /**< S/N ratio */
    uint32_t time_on_air;   /**< Time on air of a received packet (ms) */
} netdev_spsgrf_6lowpan_packet_info_t;

#ifdef __cplusplus
}
#endif

#endif /* SPSGRF_NETDEV_H */
/** @} */
