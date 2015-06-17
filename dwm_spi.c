/* 
 * File:   dwm_spi.c
 * Author: jonathan
 *
 * Created on June 9, 2015, 11:17 AM
 */

#include "dwm_spi.h"
#include "decadriver/instance.h"
#include <stdlib.h>
#include <spi.h>
#include <xc.h>

dwm_1000_status dwm_status;

chConfig_t chConfig[8] ={
                    //mode 1 - S1: 7 off, 6 off, 5 off
                    {
                        2,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_110K,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 2
                    {
                        2,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_6M8,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 3
                    {
                        2,              // channel
                        DWT_PRF_64M,    // prf
                        DWT_BR_110K,    // datarate
                        9,             // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 4
                    {
                        2,              // channel
                        DWT_PRF_64M,    // prf
                        DWT_BR_6M8,    // datarate
                        9,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 5
                    {
                        5,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_110K,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 6
                    {
                        5,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_6M8,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 7
                    {
                        5,              // channel
                        DWT_PRF_64M,    // prf
                        DWT_BR_110K,    // datarate
                        9,             // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 8
                    {
                        5,              // channel
                        DWT_PRF_64M,    // prf
                        DWT_BR_6M8,    // datarate
                        9,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    }
};

#if (DR_DISCOVERY == 0)
int instance_anchaddr = 0; //0 = 0xDECA020000000001; 1 = 0xDECA020000000002; 2 = 0xDECA020000000003
//Tag address list
uint64_t tagAddressList[3] =
{
     0xDECA010000001001,         // First tag
     0xDECA010000000002,         // Second tag
     0xDECA010000000003          // Third tag
} ;

//Anchor address list
uint64_t anchorAddressList[ANCHOR_LIST_SIZE] =
{
     0xDECA020000000001 ,       // First anchor
     0xDECA020000000002 ,       // Second anchor
     0xDECA020000000003 ,       // Third anchor
     0xDECA020000000004         // Fourth anchor
} ;

//ToF Report Forwarding Address
uint64_t forwardingAddress[1] =
{
     0xDECA030000000001
} ;


// ======================================================
//
//  Configure instance tag/anchor/etc... addresses
//
void addressconfigure(void)
{
    instanceAddressConfig_t ipc ;

    ipc.forwardToFRAddress = forwardingAddress[0];
    ipc.anchorAddress = anchorAddressList[instance_anchaddr];
    ipc.anchorAddressList = anchorAddressList;
    ipc.anchorListSize = ANCHOR_LIST_SIZE ;
    ipc.anchorPollMask = 0x1; //0x7;              // anchor poll mask

    ipc.sendReport = 1 ;  //1 => anchor sends TOF report to tag
    //ipc.sendReport = 2 ;  //2 => anchor sends TOF report to listener

    instancesetaddresses(&ipc);
}
#endif

uint8_t dwm_init(int mode)
{
    uint8_t result = 1;
    uint32_t devID ;
    instanceConfig_t instConfig;

    devID = dwt_readdevid();

    if (DWT_DEVICE_ID != devID) {
        return -1;
    }

    result = instance_init();
    if (0 > result) return(-1);

    instancesetrole(mode);

    instance_init_s(mode);
    int dr_mode = 2; //TODO: check to see what this does?

    instConfig.channelNumber = chConfig[dr_mode].channel ;
    instConfig.preambleCode = chConfig[dr_mode].preambleCode ;
    instConfig.pulseRepFreq = chConfig[dr_mode].prf ;
    instConfig.pacSize = chConfig[dr_mode].pacSize ;
    instConfig.nsSFD = chConfig[dr_mode].nsSFD ;
    instConfig.sfdTO = chConfig[dr_mode].sfdTO ;
    instConfig.dataRate = chConfig[dr_mode].datarate ;
    instConfig.preambleLen = chConfig[dr_mode].preambleLength ;

    instance_config(&instConfig) ;                  // Set operating channel etc
#if (DR_DISCOVERY == 0)
    addressconfigure() ;                            // set up initial payload configuration
#endif
    instancesettagsleepdelay(POLL_SLEEP_DELAY, BLINK_SLEEP_DELAY); //set the Tag sleep time

    // NOTE: this is the delay between receiving the blink and sending the ranging init message
    // The anchor ranging init response delay has to match the delay the tag expects
    // the tag will then use the ranging response delay as specified in the ranging init message
    // use this to set the long blink response delay (e.g. when ranging with a PC anchor that wants to use the long response times != 150ms)
    instancesetblinkreplydelay(FIXED_REPLY_DELAY);
//    instancesetblinkreplydelay(FIXED_LONG_BLINK_RESPONSE_DELAY);
    
    //set the default response delays
    instancesetreplydelay(FIXED_REPLY_DELAY, 0);

    dwt_setleds(1);

    return result;
}

/**
 * This function is the low level function used by the decaWave DWM1000 API
 */
int writetospi( uint16_t headerLength,
                const uint8_t *headerBuffer,
                uint32_t bodylength,
                const uint8_t *bodyBuffer)
{
    nSELECT = 0;
    write_data_SPI(headerBuffer, headerLength);
    write_data_SPI(bodyBuffer, bodylength);
    nSELECT = 1;
    return DWT_SUCCESS;
}

/**
 * This function is the low level function used by the decaWave DWM1000 API
 */
 int readfromspi( uint16_t	headerLength,
                const uint8_t *headerBuffer,
                uint32_t readlength,
                uint8_t *readBuffer)
 {
    nSELECT = 0;
    write_data_SPI(headerBuffer, headerLength);
    read_data_SPI(readlength, readBuffer);
    nSELECT = 1;
    return DWT_SUCCESS;
 }

/**
 * Reads num_bytes of data from the device
 */
void read_data_SPI(uint16_t num_bytes, uint8_t* rec)
{
    uint16_t cur_byte;
    for(cur_byte=0;cur_byte<num_bytes;cur_byte++){
        SPI2BUF = 0x00;
        while(!SPI2STATbits.SPIRBF);
        rec[cur_byte] = SPI2BUF;
    }
}

/**
 * Writes num_bytes of data to the device
 */
void write_data_SPI(const uint8_t* data, uint16_t num_bytes)
{
    uint16_t cur_byte;
    uint8_t blahRead;
    for(cur_byte=0;cur_byte<num_bytes;cur_byte++){
        SPI2BUF = data[cur_byte];
        while(!SPI2STATbits.SPIRBF);
        blahRead = SPI2BUF;
    }
}

/**
 * This interrupt function is tied to the IRQ pin from the DWM1000 module
 */
void __attribute__((__interrupt__, __auto_psv__)) _INT2Interrupt(void)
{
    // This is the interrupt handler used by the DWM1000 API library.
    dwm_status.irq_enable = 1;
    IFS1bits.INT2IF = 0;
}
