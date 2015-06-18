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

/**
     * Code Copied directly or modified
     * from polypoint github code
     * https://github.com/lab11/polypoint
     */
    dwt_config_t global_ranging_config;
    dwt_txconfig_t global_tx_config;

    const uint8_t xtaltrim[11] = {
        8,
        8,
        8,
        8,
        8,
        8,
        8,
        8,
        8,
        8,
        8
    };

    const uint8_t pgDelay[8] = {
        0x0,
        0xc9,
        0xc2,
        0xc5,
        0x95,
        0xc0,
        0x0,
        0x93
    };

    const uint8_t txPower[8] = {
        0x0,
        0x67,
        0x67,
        0x8b,
        0x9a,
        0x85,
        0x0,
        0xd1
    };

    uint32_t global_seq_count = 0;
    uint16_t global_tx_antenna_delay = 0;

    uint32_t global_pkt_delay_upper32 = 0;

    uint64_t global_tag_poll_tx_time = 0;
    uint64_t global_tag_anchor_resp_rx_time = 0;

    uint64_t global_tRP = 0;
    uint32_t global_tSR = 0;
    uint64_t global_tRF = 0;
    uint8_t global_recv_pkt[512];

    uint32_t global_subseq_num = 0xFFFFFFFF;
    uint8_t global_chan = 1;
    float global_distances[NUM_ANCHORS*NUM_ANTENNAS*NUM_ANTENNAS*NUM_CHANNELS];

    struct ieee154_msg  {
        uint8_t frameCtrl[2];                             //  frame control bytes 00-01
        uint8_t seqNum;                                   //  sequence_number 02
        uint8_t panID[2];                                 //  PAN ID 03-04
        uint8_t destAddr[8];
        uint8_t sourceAddr[8];
        uint8_t messageType; //   (application data and any user payload)
        uint8_t anchorID;
        uint8_t fcs[2] ;                                  //  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
    } __attribute__ ((__packed__));

    struct ieee154_final_msg  {
        uint8_t frameCtrl[2];                             //  frame control bytes 00-01
        uint8_t seqNum;                                   //  sequence_number 02
        uint8_t panID[2];                                 //  PAN ID 03-04
        uint8_t destAddr[8];
        uint8_t sourceAddr[8];
        uint8_t messageType; //   (application data and any user payload)
        uint8_t anchorID;
        float distanceHist[NUM_ANTENNAS*NUM_ANTENNAS*NUM_CHANNELS];
        uint8_t fcs[2] ;                                  //  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
    } __attribute__ ((__packed__));

    struct ieee154_bcast_msg  {
        uint8_t frameCtrl[2];                             //  frame control bytes 00-01
        uint8_t seqNum;                                   //  sequence_number 02
        uint8_t panID[2];                                 //  PAN ID 03-04
        uint8_t destAddr[2];
        uint8_t sourceAddr[8];
        uint8_t messageType; //   (application data and any user payload)
        uint8_t subSeqNum;
        uint32_t tSP;
        uint32_t tSF;
        uint64_t tRR[NUM_ANCHORS]; // time differences
        uint8_t fcs[2] ;                                  //  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
    } __attribute__ ((__packed__));

    struct ieee154_msg msg;
    struct ieee154_final_msg fin_msg;
    struct ieee154_bcast_msg bcast_msg;
    /*****https://github.com/lab11/polypoint******/

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
    
    devID = dwt_readdevid();

    if (DWT_DEVICE_ID != devID) {
        return -1;
    }

    result = instance_init();
    if (0 > result) return(-1);

    instancesetrole(mode);

    instance_init_s(mode);
//    int dr_mode = 2; //TODO: check to see what this does?

    /**
     * Code Copied directly or modified
     * from polypoint github code
     * https://github.com/lab11/polypoint
     */
    // Set the parameters of ranging and channel and whatnot
    global_ranging_config.chan           = 2;
    global_ranging_config.prf            = DWT_PRF_64M;
    global_ranging_config.txPreambLength = DWT_PLEN_4096;//DWT_PLEN_4096
    // global_ranging_config.txPreambLength = DWT_PLEN_256;
    global_ranging_config.rxPAC          = DWT_PAC64;
    global_ranging_config.txCode         = 9;  // preamble code
    global_ranging_config.rxCode         = 9;  // preamble code
    global_ranging_config.nsSFD          = 1;
    global_ranging_config.dataRate       = DWT_BR_110K;
    global_ranging_config.phrMode        = DWT_PHRMODE_EXT; //Enable extended PHR mode (up to 1024-byte packets)
    global_ranging_config.smartPowerEn   = 0;
    global_ranging_config.sfdTO          = 4096+64+1;//(1025 + 64 - 32);
    dwt_configure(&global_ranging_config, 0);
    
    // Configure TX power
    {
        global_tx_config.PGdly = pgDelay[global_ranging_config.chan];
        global_tx_config.power = txPower[global_ranging_config.chan];
        dwt_configuretxrf(&global_tx_config);
    }

    if(DW1000_ROLE_TYPE == TAG){
        dwt_xtaltrim(xtaltrim[0]);
    }
    else{
        dwt_xtaltrim(xtaltrim[ANCHOR_EUI]);
    }

    // Configure the antenna delay settings
    {
        uint16_t antenna_delay;

        //Antenna delay not really necessary if we're doing an end-to-end calibration
        antenna_delay = 0;
        dwt_setrxantennadelay(antenna_delay);
        dwt_settxantennadelay(antenna_delay);
        global_tx_antenna_delay = antenna_delay;
    }

    // Setup the constants in the outgoing packet
    msg.frameCtrl[0] = 0x41; // data frame, ack req, panid comp
    msg.frameCtrl[1] = 0xCC; // ext addr
    msg.panID[0] = DW1000_PANID & 0xff;
    msg.panID[1] = DW1000_PANID >> 8;
    msg.seqNum = 0;
    msg.anchorID = ANCHOR_EUI;

    // Setup the constants in the outgoing packet
    fin_msg.frameCtrl[0] = 0x41; // data frame, ack req, panid comp
    fin_msg.frameCtrl[1] = 0xCC; // ext addr
    fin_msg.panID[0] = DW1000_PANID & 0xff;
    fin_msg.panID[1] = DW1000_PANID >> 8;
    fin_msg.seqNum = 0;
    fin_msg.anchorID = ANCHOR_EUI;

    // Setup the constants in the outgoing packet
    bcast_msg.frameCtrl[0] = 0x41; // data frame, ack req, panid comp
    bcast_msg.frameCtrl[1] = 0xC8; // ext addr
    bcast_msg.panID[0] = DW1000_PANID & 0xff;
    bcast_msg.panID[1] = DW1000_PANID >> 8;
    bcast_msg.seqNum = 0;
    bcast_msg.subSeqNum = 0;

    // Calculate the delay between packet reception and transmission.
    // This applies to the time between POLL and RESPONSE (on the anchor side)
    // and RESPONSE and POLL (on the tag side).
    global_pkt_delay_upper32 = (convertmicrosectodevicetimeu32(NODE_DELAY_US) & DELAY_MASK) >> 8;

    if (DW1000_ROLE_TYPE == ANCHOR) {
	uint8_t eui_array[8];

        // Enable frame filtering
        dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN);

        dw1000_populate_eui(eui_array, ANCHOR_EUI);
  	dwt_seteui(eui_array);
        dwt_setpanid(DW1000_PANID);

        // Set more packet constants
        dw1000_populate_eui(msg.sourceAddr, ANCHOR_EUI);
        dw1000_populate_eui(fin_msg.sourceAddr, ANCHOR_EUI);

        // hard code destination for now....
        dw1000_populate_eui(msg.destAddr, TAG_EUI);
        dw1000_populate_eui(fin_msg.destAddr, TAG_EUI);

        // We do want to enable auto RX
        dwt_setautorxreenable(1);
        // Let's do double buffering
        dwt_setdblrxbuffmode(0);
        // Disable RX timeout by setting to 0
        dwt_setrxtimeout(0);

        // Try pre-populating this
        msg.seqNum++;
        msg.messageType = MSG_TYPE_ANC_RESP;
        fin_msg.messageType = MSG_TYPE_ANC_FINAL;

        // Go for receiving
        dwt_rxenable(0);

    } else if (DW1000_ROLE_TYPE == TAG) {
	uint8_t eui_array[8];

        // First thing we do as a TAG is send the POLL message when the
        // timer fires
        // tag_state = TAG_SEND_POLL;

        // Allow data and ack frames
        dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN);

        dw1000_populate_eui(eui_array, TAG_EUI);
  	dwt_seteui(eui_array);
        dwt_setpanid(DW1000_PANID);

        // Set more packet constants
        dw1000_populate_eui(bcast_msg.sourceAddr, TAG_EUI);
	memset(bcast_msg.destAddr, 0xFF, 2);

        // Do this for the tag too
        dwt_setautorxreenable(1);
        dwt_setdblrxbuffmode(1);
        dwt_enableautoack(5 /*ACK_RESPONSE_TIME*/);

        // Configure sleep
        {
            int mode = DWT_LOADUCODE    |
                       DWT_PRESRV_SLEEP |
                       DWT_CONFIG       |
                       DWT_TANDV;
            if (dwt_getldotune() != 0) {
                // If we need to use LDO tune value from OTP kick it after sleep
                mode |= DWT_LOADLDO;
            }
            // NOTE: on the EVK1000 the DEEPSLEEP is not actually putting the
            // DW1000 into full DEEPSLEEP mode as XTAL is kept on
            dwt_configuresleep(mode, DWT_WAKE_CS | DWT_SLP_EN);
        }

    }
    /*****https://github.com/lab11/polypoint******/

//#if (DR_DISCOVERY == 0)
//    addressconfigure() ;                            // set up initial payload configuration
//#endif
//    instancesettagsleepdelay(POLL_SLEEP_DELAY, BLINK_SLEEP_DELAY); //set the Tag sleep time
//
//    // NOTE: this is the delay between receiving the blink and sending the ranging init message
//    // The anchor ranging init response delay has to match the delay the tag expects
//    // the tag will then use the ranging response delay as specified in the ranging init message
//    // use this to set the long blink response delay (e.g. when ranging with a PC anchor that wants to use the long response times != 150ms)
//    instancesetblinkreplydelay(FIXED_REPLY_DELAY);
////    instancesetblinkreplydelay(FIXED_LONG_BLINK_RESPONSE_DELAY);
//
//    //set the default response delays
//    instancesetreplydelay(FIXED_REPLY_DELAY, 0);

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
 * Code Copied directly or modified
 * from polypoint github code
 * https://github.com/lab11/polypoint
 */
void dw1000_populate_eui (uint8_t *eui_buf, uint8_t id) {
  eui_buf[0] = id;
  eui_buf[1] = 0x55;
  eui_buf[2] = 0x44;
  eui_buf[3] = 'N';
  eui_buf[4] = 'P';
  eui_buf[5] = 0xe5;
  eui_buf[6] = 0x98;
  eui_buf[7] = 0xc0;
}
/*****https://github.com/lab11/polypoint******/

/**
 * This interrupt function is tied to the IRQ pin from the DWM1000 module
 */
void __attribute__((__interrupt__, __auto_psv__)) _INT2Interrupt(void)
{
    // This is the interrupt handler used by the DWM1000 API library.
    dwm_status.irq_enable = 1;
    IFS1bits.INT2IF = 0;
}
