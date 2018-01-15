/**
 * @ingroup tests
 * @{
 *
 * @file
 * @brief       Test application for the SPSGRF  driver.
 *
 * @author       McDonald <jason.mcdonald@cumberblandgroupit.com>
 *
 * @}
 */

#include <stdio.h>
#include <stdlib.h>
#include "spsgrf.h"
#include "spsgrf_app.h"
#include "periph/spi.h"

#define XTAL_OFFSET_PPM             0
#define INFINITE_TIMEOUT            0.0

#define BASE_FREQUENCY              915.0e6

#define CHANNEL_SPACE               100e3
#define CHANNEL_NUMBER              0
#define MODULATION_SELECT           FSK
#define DATARATE                    38400
#define FREQ_DEVIATION              20e3
#define BANDWIDTH                   100E3

#define POWER_DBM                   11.6
#define POWER_INDEX                 7
#define RECEIVE_TIMEOUT             2000.0 /*change the value for required timeout period*/   

#define RSSI_THRESHOLD              -120  /* Default RSSI at reception, more 
                                                             than noise floor */
#define CSMA_RSSI_THRESHOLD         -90   /* Higher RSSI to Transmit. 
                              If it's lower, the Channel will be seen as busy */
typedef struct sRadioDriver
{
    void ( *Init )( void );
    void ( *GpioIrq )( SGpioInit *pGpioIRQ );
    void ( *RadioInit )( SRadioInit *pRadioInit );
    void ( *SetRadioPower )( uint8_t cIndex, float fPowerdBm );
    void ( *PacketConfig )( void );
    void ( *SetPayloadLen )( uint8_t length);
    void ( *SetDestinationAddress )( uint8_t address);
    void ( *EnableTxIrq )( void );
    void ( *EnableRxIrq )( void );
    void ( *DisableIrq )(void);
    void ( *SetRxTimeout )( float cRxTimeout );
    void ( *EnableSQI )(void);
    void ( *SetRssiThreshold)(int dbmValue);
    void ( *ClearIrqStatus )(void);
    void ( *StartRx )( void );
    void ( *StartTx )( uint8_t *buffer, uint8_t size ); 
    void ( *GetRxPacket )( uint8_t *buffer, uint8_t *size );
}RadioDriver_t;   
/*
RadioDriver_t spirit_cb =
{
  .Init = Spirit1InterfaceInit, 
  .GpioIrq = Spirit1GpioIrqInit,
  .RadioInit = Spirit1RadioInit,
  .SetRadioPower = Spirit1SetPower,
  .PacketConfig = Spirit1PacketConfig,
  .SetPayloadLen = Spirit1SetPayloadlength,
  .SetDestinationAddress = Spirit1SetDestinationAddress,
  .EnableTxIrq = Spirit1EnableTxIrq,
  .EnableRxIrq = Spirit1EnableRxIrq,
  .DisableIrq = Spirit1DisableIrq,
  .SetRxTimeout = Spirit1SetRxTimeout,
  .EnableSQI = Spirit1EnableSQI,
  .SetRssiThreshold = Spirit1SetRssiTH,
  .ClearIrqStatus = Spirit1ClearIRQ,
  .StartRx = Spirit1StartRx,
  .StartTx = Spirit1StartTx,
  .GetRxPacket = Spirit1GetRxPacket
};
*/

SGpioInit xGpioIRQ={
  SPIRIT_GPIO_3,
  SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,
  SPIRIT_GPIO_DIG_OUT_IRQ
};

SRadioInit xRadioInit = {
  XTAL_OFFSET_PPM,
  BASE_FREQUENCY,
  CHANNEL_SPACE,
  CHANNEL_NUMBER,
  MODULATION_SELECT,
  DATARATE,
  FREQ_DEVIATION,
  BANDWIDTH
};

int main(void)
{
    printf("-----Manual SPSGRF Driver test---------\n");
    //RadioSpiInit();
    Spirit1InterfaceInit();
    Spirit1GpioIrqInit(&xGpioIRQ);
    Spirit1RadioInit(&xRadioInit);
    Spirit1SetPower(POWER_INDEX, POWER_DBM);
    Spirit1PacketConfig();
    Spirit1EnableSQI();
    Spirit1SetRssiTH(RSSI_THRESHOLD);
    printf("-----All Calls Completed---------\n");
    //RadioSpiInit();
  

    //spirit_cb.Init();
    //spirit_cb.GpioIrq(&xGpioIRQ);
    //spirit_cb.RadioInit(&xRadioInit);
    //spirit_cb.SetRadioPower(POWER_INDEX, POWER_DBM);
    //spirit_cb.PacketConfig();
    //spirit_cb.EnableSQI();
    //spirit_cb.SetRssiThreshold(RSSI_THRESHOLD);
    
    /*
     * GPIOIRQ
     * RADIOINT
     * SETRADIOPOWER
     * PACKETCONF
     * ENABLESQI
     * SETRSSI
     */
    
    return 0;
}
