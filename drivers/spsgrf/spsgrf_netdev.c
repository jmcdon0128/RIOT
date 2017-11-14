
/**
 * @ingroup     drivers_spsgrf
 * @{
 * @file
 * @brief       Netdev adaptation for the spsgrf driver
 *
 * @author      Jason McDonald <jason.mcdonald@cumberlandgroupit.com>
 * @}
 */

#include <stddef.h>
#include <string.h>
#include <errno.h>

#include "net/netopt.h"
#include "net/netdev.h"
#include "spsgrf_netdev.h"
#include "spsgrf.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

/* Netdev driver api functions */
static int _send(netdev_t *netdev, const struct iovec *vector, unsigned count);
static int _recv(netdev_t *netdev, void *buf, size_t len, void *info);
static int _init(netdev_t *netdev);
static void _isr(netdev_t *netdev);
static int _get(netdev_t *netdev, netopt_t opt, void *val, size_t max_len);
static int _set(netdev_t *netdev, netopt_t opt, const void *val, size_t len);

const netdev_driver_t spsgrf_driver = {
    .send = _send,
    .recv = _recv,
    .init = _init,
    .isr = _isr,
    .get = _get,
    .set = _set,
};

static int _send(netdev_t *netdev, const struct iovec *vector, unsigned count)
{
    //sx127x_t *dev = (sx127x_t*) netdev;

    return 0;
}

static int _recv(netdev_t *netdev, void *buf, size_t len, void *info)
{
    //sx127x_t *dev = (sx127x_t*) netdev;
    return 0;
}

static int _init(netdev_t *netdev)
{
 //   sx127x_t *sx127x = (sx127x_t*) netdev;

    /* Launch initialization of driver and device */
    DEBUG("init_radio: initializing driver...\n");
    /* Put chip into sleep */

    DEBUG("init_radio: sx127x initialization done\n");

    return 0;
}

static void _isr(netdev_t *netdev)
{
//    sx127x_t *dev = (sx127x_t *) netdev;

}

static int _get(netdev_t *netdev, netopt_t opt, void *val, size_t max_len)
{
//    sx127x_t *dev = (sx127x_t*) netdev;

/*    if (dev == NULL) {
        return -ENODEV;
    }

    switch(opt) {
        case NETOPT_STATE:
            assert(max_len >= sizeof(netopt_state_t));
            return _get_state(dev, val);

        case NETOPT_DEVICE_MODE:
            assert(max_len >= sizeof(uint8_t));
            *((uint8_t*) val) = dev->settings.modem;
            return sizeof(uint8_t);

        case NETOPT_CHANNEL:
            assert(max_len >= sizeof(uint32_t));
            *((uint32_t*) val) = sx127x_get_channel(dev);
            return sizeof(uint32_t);

        case NETOPT_BANDWIDTH:
            assert(max_len >= sizeof(uint8_t));
            *((uint8_t*) val) = sx127x_get_bandwidth(dev);
            return sizeof(uint8_t);

        case NETOPT_SPREADING_FACTOR:
            assert(max_len >= sizeof(uint8_t));
            *((uint8_t*) val) = sx127x_get_spreading_factor(dev);
            return sizeof(uint8_t);

        case NETOPT_CODING_RATE:
            assert(max_len >= sizeof(uint8_t));
            *((uint8_t*) val) = sx127x_get_coding_rate(dev);
            return sizeof(uint8_t);

        case NETOPT_MAX_PACKET_SIZE:
            assert(max_len >= sizeof(uint8_t));
            *((uint8_t*) val) = sx127x_get_max_payload_len(dev);
            return sizeof(uint8_t);

        case NETOPT_INTEGRITY_CHECK:
            assert(max_len >= sizeof(netopt_enable_t));
            *((netopt_enable_t*) val) = sx127x_get_crc(dev) ? NETOPT_ENABLE : NETOPT_DISABLE;
            break;

        case NETOPT_CHANNEL_HOP:
            assert(max_len >= sizeof(netopt_enable_t));
            *((netopt_enable_t*) val) = (dev->settings.lora.flags & SX127X_CHANNEL_HOPPING_FLAG) ? NETOPT_ENABLE : NETOPT_DISABLE;
            break;

        case NETOPT_CHANNEL_HOP_PERIOD:
            assert(max_len >= sizeof(uint8_t));
            *((uint8_t*) val) = sx127x_get_hop_period(dev);
            return sizeof(uint8_t);

        case NETOPT_SINGLE_RECEIVE:
            assert(max_len >= sizeof(uint8_t));
            *((netopt_enable_t*) val) = sx127x_get_rx_single(dev) ? NETOPT_ENABLE : NETOPT_DISABLE;
            break;

        default:
            break;
    }
*/
    return 0;
}

static int _set(netdev_t *netdev, netopt_t opt, const void *val, size_t len)
{
 //   sx127x_t *dev = (sx127x_t*) netdev;
  //  int res = -ENOTSUP;
/*
    if (dev == NULL) {
        return -ENODEV;
    }

    switch(opt) {
        case NETOPT_STATE:
            assert(len <= sizeof(netopt_state_t));
            return _set_state(dev, *((const netopt_state_t*) val));

        case NETOPT_DEVICE_MODE:
            assert(len <= sizeof(uint8_t));
            sx127x_set_modem(dev, *((const uint8_t*) val));
            return sizeof(netopt_enable_t);

        case NETOPT_CHANNEL:
            assert(len <= sizeof(uint32_t));
            sx127x_set_channel(dev, *((const uint32_t*) val));
            return sizeof(uint32_t);

        case NETOPT_BANDWIDTH:
            assert(len <= sizeof(uint8_t));
            uint8_t bw = *((const uint8_t *)val);
            if (bw < SX127X_BW_125_KHZ ||
                bw > SX127X_BW_500_KHZ) {
                res = -EINVAL;
                break;
            }
            sx127x_set_bandwidth(dev, bw);
            return sizeof(uint8_t);

        case NETOPT_SPREADING_FACTOR:
            assert(len <= sizeof(uint8_t));
            uint8_t sf = *((const uint8_t *)val);
            if (sf < SX127X_SF6 ||
                sf > SX127X_SF12) {
                res = -EINVAL;
                break;
            }
            sx127x_set_spreading_factor(dev, sf);
            return sizeof(uint8_t);

        case NETOPT_CODING_RATE:
            assert(len <= sizeof(uint8_t));
            uint8_t cr = *((const uint8_t *)val);
            if (cr < SX127X_CR_4_5 ||
                cr > SX127X_CR_4_8) {
                res = -EINVAL;
                break;
            }
            sx127x_set_coding_rate(dev, cr);
            return sizeof(uint8_t);

        case NETOPT_MAX_PACKET_SIZE:
            assert(len <= sizeof(uint8_t));
            sx127x_set_max_payload_len(dev, *((const uint8_t*) val));
            return sizeof(uint8_t);

        case NETOPT_INTEGRITY_CHECK:
            assert(len <= sizeof(netopt_enable_t));
            sx127x_set_crc(dev, *((const netopt_enable_t*) val) ? true : false);
            return sizeof(netopt_enable_t);

        case NETOPT_CHANNEL_HOP:
            assert(len <= sizeof(netopt_enable_t));
            sx127x_set_freq_hop(dev, *((const netopt_enable_t*) val) ? true : false);
            return sizeof(netopt_enable_t);

        case NETOPT_CHANNEL_HOP_PERIOD:
            assert(len <= sizeof(uint8_t));
            sx127x_set_hop_period(dev, *((const uint8_t*) val));
            return sizeof(uint8_t);

        case NETOPT_SINGLE_RECEIVE:
            assert(len <= sizeof(uint8_t));
            sx127x_set_rx_single(dev, *((const netopt_enable_t*) val) ? true : false);
            return sizeof(netopt_enable_t);

        case NETOPT_RX_TIMEOUT:
            assert(len <= sizeof(uint32_t));
            sx127x_set_rx_timeout(dev, *((const uint32_t*) val));
            return sizeof(uint32_t);

        case NETOPT_TX_TIMEOUT:
            assert(len <= sizeof(uint32_t));
            sx127x_set_tx_timeout(dev, *((const uint32_t*) val));
            return sizeof(uint32_t);

        case NETOPT_TX_POWER:
            assert(len <= sizeof(uint8_t));
            sx127x_set_tx_power(dev, *((const uint8_t*) val));
            return sizeof(uint16_t);

        case NETOPT_FIXED_HEADER:
            assert(len <= sizeof(netopt_enable_t));
            sx127x_set_fixed_header_len_mode(dev, *((const netopt_enable_t*) val) ? true : false);
            return sizeof(netopt_enable_t);

        case NETOPT_PREAMBLE_LENGTH:
            assert(len <= sizeof(uint16_t));
            sx127x_set_preamble_length(dev, *((const uint16_t*) val));
            return sizeof(uint16_t);

        case NETOPT_IQ_INVERT:
            assert(len <= sizeof(netopt_enable_t));
            sx127x_set_iq_invert(dev, *((const netopt_enable_t*) val) ? true : false);
            return sizeof(bool);

        default:
            break;
    }
*/
    return 0;
}

