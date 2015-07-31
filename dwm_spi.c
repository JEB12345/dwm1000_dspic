/* 
 * File:   dwm_spi.c
 * Author: jonathan
 *
 * Created on June 9, 2015, 11:17 AM
 */

#include "dwm_spi.h"
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"
#include <stdlib.h>
#include <spi.h>
#include <string.h>
#include <xc.h>
#include <float.h>

dwm_1000_status dwm_status;
uint16_t wait_count = 0;
uint16_t stall_count = 0;
anchor_states anchor_state = anchor_init;
tag_states tag_state = tag_init;


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

    const uint8_t txPower16[8] = {
        0x0,
        0x75757575,
        0x75757575,
        0x6F6F6F6F,
        0x5F5F5F5F,
        0x48484848,
        0x0,
        0x92929292
    };
    
    const uint8_t txPower64[8] = {
        0x0,
        0x67676767,
        0x67676767,
        0x8B8B8B8B,
        0x9A9A9A9A,
        0x85858585,
        0x0,
        0xD1D1D1D1
    };

    const double txDelayCal[11*3] = {
    //    -0.13, 0.36, -0.05,//T2
    //    -0.14, 0.37, -0.03,//A1
    //    -0.20, 0.39, -0.06,//A2
    //    -0.07, 0.43, 0.04,//A3
    //    -0.15, 0.38, -0.01,//A4
    //    -0.03, 0.49, 0.07,//A5
    //    -0.18, 0.45, -0.07,//A6
    //    -0.08, 0.42, -0.00,//A7
    //    0.04, 0.51, 0.07,//A8
    //    -0.15, 0.38, -0.01,//A9
    //    -0.12, 0.44, 0.01,//A10
    154.82214201598609, 154.82214201598609, 154.82214201598609, //Need to fine tune this parameter
       155.5433333333333,   155.0400000000000,   155.4433333333333,
       155.5766666666667,   154.9922222222222,   155.4500000000000,
       155.4855555555555,   154.9122222222222,   155.3466666666667,
       155.5000000000000,   154.9377777777778,   155.3444444444445,
       155.3955555555556,   154.8555555555555,   155.2922222222222,
       155.4966666666667,   154.9100000000000,   155.3755555555556,
       155.4133333333333,   154.9233333333333,   155.3422222222222,
       155.3266666666667,   154.8377777777778,   155.2800000000000,
       155.5188888888889,   154.9277777777778,   155.3722222222222,
       155.4877777777778,   154.9033333333333,   155.3533333333333
    };

    static uint32_t global_seq_count = 0;
    static uint16_t global_tx_antenna_delay = 0;

    static uint32_t global_pkt_delay_upper32 = 0;

   // uint64_t global_tag_poll_tx_time = 0;
//    uint64_t global_tag_anchor_resp_rx_time = 0;
    
    static uint64_t global_tag_anchor_resp_rx_time[NUM_TOTAL_NODES];
    static uint64_t global_tRR[NUM_TOTAL_NODES];
    static uint64_t global_tRP[NUM_TOTAL_NODES];
    static uint64_t global_tSR = 0;
    static uint64_t global_tRF[NUM_TOTAL_NODES];
    static uint64_t global_tSP[NUM_TOTAL_NODES];
    static uint64_t global_tSF[NUM_TOTAL_NODES];
    static uint8_t global_received_poll[NUM_TOTAL_NODES]; //flags to indicate from which nodes we received a valid message
    static uint8_t global_received_response[NUM_TOTAL_NODES];
    static uint8_t global_received_final[NUM_TOTAL_NODES];
    static uint8_t global_recv_pkt[512];

    static uint32_t global_subseq_num = 0x0;
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
        uint8_t destAddr[2];
        uint8_t sourceAddr[2];
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
        uint8_t sourceAddr;
        uint8_t messageType; //   (application data and any user payload)
//        uint32_t data[2];
        uint8_t tSP[5];
        uint8_t tSF[5];
        uint8_t tRR[NUM_TOTAL_NODES][5];
        //uint64_t tRR[NUM_ANCHORS]; // time differences
        uint8_t fcs[2] ;                                  //  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
    } __attribute__ ((__packed__));

    struct ieee154_msg msg;
    struct ieee154_final_msg fin_msg;
    struct ieee154_bcast_msg bcast_msg;
    
//    void to_lower_40bits(uint8_t* out, uint64_t in){//too slow!
//        unsigned i;
//        for(i=0;i<5;++i){
//            out[i] = (in>>(i<<3))&0xFF; 
//        }
//    }
#define to_lower_40bits(_OUT,_IN) _OUT[0]=(_IN)&0xFF;_OUT[1]=((_IN)>>8)&0xFF;_OUT[2]=((_IN)>>16)&0xFF;_OUT[3]=((_IN)>>24)&0xFF;_OUT[4]=((_IN)>>32)&0xFF
    
//    uint64_t from_lower_40_bits(uint8_t* in){ //too slow!
//        unsigned i;
//        uint64_t result = 0;
//        for(i=0;i<5;++i){
//            result <<=8;
//            result |= in[4-i]&0xFF;
//        }
//        return result;
//    }
#define from_lower_40_bits(_IN) (((_IN)[0])|(((uint64_t)(_IN)[1])<<8)|(((uint64_t)(_IN)[2])<<16)|(((uint64_t)(_IN)[3])<<24)|(((uint64_t)(_IN)[4])<<32))
    
    /*****https://github.com/lab11/polypoint******/

#define BLINK_SLEEP_DELAY					1000 //ms
#define POLL_SLEEP_DELAY					500 //ms
#define FIXED_REPLY_DELAY       			150
#define FIXED_LONG_BLINK_RESPONSE_DELAY       (5*FIXED_REPLY_DELAY) //NOTE: this should be a multiple of FIXED_LONG_REPLY_DELAY see DELAY_MULTIPLE below

void dwm_reset()
{
    DWM_RESET_ON;
    Delay_us(20);
    DWM_RESET_OFF;
}

uint8_t dwm_init(uint8_t node_id, void (*timer_func)(uint16_t microseconds,void (*cb)()))
{
    uint8_t result = 1;
    uint32_t devID ;

    dwm_status.timer_func = timer_func;
    dwm_status.timer_interrupt = 0; 
    dwm_status.tx_state = DWM_SEND_POLL;
    dwm_status.node_id = node_id;
    
    dwm_reset();

    while(DWM_RESET_IN != 1);

    Delay_us(10); // The PLL clock is not settled after the chip goes into INIT state, a minimum of 5us is needed.

    devID = dwt_readdevid();

    if (DWT_DEVICE_ID != devID) {
        return -1;
    }
    
    Delay_us(1000);

    result = dwt_initialise(DWT_LOADUCODE    |
                         DWT_LOADLDO      |
                         DWT_LOADTXCONFIG |
                         DWT_LOADLDOTUNE);
    if (0 > result) return(-1);
    
    // Try to force PLLLDT bit set so PLL error bits work
    dwt_write32bitoffsetreg(EXT_SYNC_ID,EC_CTRL_OFFSET, 0x00000004);

    // Setup interrupts
    // Note: using auto rx re-enable so don't need to trigger on error frames
    dwt_setinterrupt(DWT_INT_TFRS |
                     DWT_INT_RFCG |
                     DWT_INT_SFDT |
                     DWT_INT_RFTO |
                     DWT_INT_RPHE |
                     DWT_INT_RFCE |
                     DWT_INT_RFSL |
                     DWT_INT_RXPTO | DWT_INT_ARFE |  DWT_INT_LDED | DWT_INT_RXOVRR |
                     DWT_INT_SFDT, 1);

    // Configure the callbacks from the dwt library
    dwt_setcallbacks(app_dw1000_txcallback, app_dw1000_rxcallback);
//    int dr_mode = 2; //TODO: check to see what this does?

    /**
     * Code Copied directly or modified
     * from polypoint github code
     * https://github.com/lab11/polypoint
     */
    // Set the parameters of ranging and channel and whatnot
    global_ranging_config.chan           = 2;
	global_ranging_config.prf            = DWT_PRF_64M;
	global_ranging_config.txPreambLength = DWT_PLEN_64;//DWT_PLEN_4096
	// global_ranging_config.txPreambLength = DWT_PLEN_256;
	global_ranging_config.rxPAC          = DWT_PAC8;
	global_ranging_config.txCode         = 9;  // preamble code
	global_ranging_config.rxCode         = 9;  // preamble code
	global_ranging_config.nsSFD          = 0;
	global_ranging_config.dataRate       = DWT_BR_6M8;
	global_ranging_config.phrMode        = DWT_PHRMODE_EXT; //Enable extended PHR mode (up to 1024-byte packets)
	global_ranging_config.smartPowerEn   = 0;
	global_ranging_config.sfdTO          = 64+8+1;//(1025 + 64 - 32);
	dwt_configure(&global_ranging_config, 0);//(DWT_LOADANTDLY | DWT_LOADXTALTRIM));
	dwt_setsmarttxpower(global_ranging_config.smartPowerEn);

    dwt_configeventcounters(1);
    // Configure TX power
    {
        global_tx_config.PGdly = pgDelay[global_ranging_config.chan];
        global_tx_config.power = txPower64[global_ranging_config.chan];
        dwt_configuretxrf(&global_tx_config);
    }

//    if(DW1000_ROLE_TYPE == TAG){
//        dwt_xtaltrim(xtaltrim[0]);
//    }
//    else{
//        dwt_xtaltrim(xtaltrim[ANCHOR_EUI]);
//    }

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
//    bcast_msg.subSeqNum = 0;

    // Calculate the delay between packet reception and transmission.
    // This applies to the time between POLL and RESPONSE (on the anchor side)
    // and RESPONSE and POLL (on the tag side).
//    global_pkt_delay_upper32 = (convertmicrosectodevicetimeu32(NODE_DELAY_US) & DELAY_MASK) >> 8;

    
    
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

        dw1000_populate_eui(eui_array, TAG_EUI+dwm_status.node_id);
        
  	dwt_seteui(eui_array);
        dwt_setpanid(DW1000_PANID);

        // Set more packet constants
        //dw1000_populate_eui(bcast_msg.sourceAddr, TAG_EUI);
	memset(bcast_msg.destAddr, 0xFF, 2);

        // Do this for the tag too
        dwt_setautorxreenable(1);
        dwt_setdblrxbuffmode(0);
        dwt_enableautoack(5 /*ACK_RESPONSE_TIME*/);
        dwt_rxenable(0);
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
    dwt_setleds(1);

    return result;
}
unsigned delayed_tx = 0;
// Triggered after a TX
void app_dw1000_txcallback (const dwt_callback_data_t *txd) {
    //NOTE: No need for tx timestamping after-the-fact (everything's done beforehand)
}

void dwt_timer_cb(){
    dwm_status.timer_interrupt = 1;
}

// Triggered when we receive a packet
void app_dw1000_rxcallback (const dwt_callback_data_t *rxd) {
    uint8_t rxtimestamp[5];
    unsigned i; //loop counter
    if (rxd->event == DWT_SIG_RX_OKAY) {
        dwt_readrxdata(global_recv_pkt, rxd->datalength, 0);
        //Assume everything is a bcast msg for now
        struct ieee154_bcast_msg* msg_ptr;
        msg_ptr = (struct ieee154_bcast_msg*) global_recv_pkt;
        
        dwt_readrxtimestamp(rxtimestamp);
        if(msg_ptr->messageType == DWM_SEND_POLL){
            if(msg_ptr->sourceAddr==0){ //received poll from first node: start of new sequence
                //reset state
                memset(global_tag_anchor_resp_rx_time,0x0,sizeof(global_tag_anchor_resp_rx_time));
                memset(global_tRR,0x0,sizeof(global_tRR));
                memset(global_tRP,0x0,sizeof(global_tRP));
                memset(global_tRF,0x0,sizeof(global_tRF));
                memset(global_tSP,0x0,sizeof(global_tSP));
                memset(global_tSF,0x0,sizeof(global_tSF));
                //start transmit sequence
                dwm_status.tx_state = DWM_SEND_POLL;
                dwm_status.timer_func(dwm_status.node_id*1000,dwt_timer_cb);
            }
            //store information contained in and related to poll message  
            global_tRP[msg_ptr->sourceAddr] = from_lower_40_bits(rxtimestamp);
            global_tSP[msg_ptr->sourceAddr] = from_lower_40_bits(msg_ptr->tSP);
            global_received_poll[msg_ptr->sourceAddr] = 1;
        } else if (msg_ptr->messageType == DWM_SEND_RESPONSE){
            //store information contained in and related to response message
            dwt_readrxtimestamp(rxtimestamp);
            global_tag_anchor_resp_rx_time[msg_ptr->sourceAddr] = from_lower_40_bits(rxtimestamp);
            global_received_response[msg_ptr->sourceAddr] = 1;
        } else if (msg_ptr->messageType == DWM_SEND_FINAL) {
            //store information contained in and related to final message
            global_tRF[msg_ptr->sourceAddr] = from_lower_40_bits(rxtimestamp);
            global_tRR[msg_ptr->sourceAddr] = from_lower_40_bits(msg_ptr->tRR[dwm_status.node_id]); //note: own id as we're reading "our" reception time on a remote node
            global_tSF[msg_ptr->sourceAddr] = from_lower_40_bits(msg_ptr->tSF);
            global_received_final[msg_ptr->sourceAddr] = 1;
        } else if (msg_ptr->messageType == DWM_SEND_DISTANCES){
            //received distance measurements from a fixed node, just store it for now
            if(msg_ptr->sourceAddr>=NUM_FLOATING_NODES){ //just a safety precaution
                for(i=0;i<NUM_TOTAL_NODES;++i){
                    memcpy(&(dwm_status.distance_mm_fixed[msg_ptr->sourceAddr-NUM_FLOATING_NODES][i]),msg_ptr->tRR[i],sizeof(dwm_status.distance_mm_fixed[msg_ptr->sourceAddr-NUM_FLOATING_NODES][i]));
                }
            }
        } else {
            //unknown message, ignore
        }
        
        dwt_forcetrxoff();        
        dwt_setrxtimeout(0); //TODO: not sure if needed
        dwt_rxenable(0);
    }

}

void send_poll(){
    //Reset all the tRRs at the beginning of each poll event
	memset(bcast_msg.tRR, 0, sizeof(bcast_msg.tRR));//TODO: hacked
    //bcast_msg.tRR = 0;//_H = bcast_msg.tRR_L = 0;
    
	// Through tSP (tSF is field after) then +2 for FCS
    uint16_t tx_frame_length = sizeof(bcast_msg);
	memset(bcast_msg.destAddr, 0xFF, 2);

	bcast_msg.seqNum++;
//	bcast_msg.subSeqNum = global_subseq_num;

	// First byte identifies this as a POLL
	bcast_msg.messageType = DWM_SEND_POLL;//MSG_TYPE_TAG_POLL;
    
    bcast_msg.sourceAddr = dwm_status.node_id;

	// Tell the DW1000 about the packet
	dwt_writetxfctrl(tx_frame_length, 0);

	// We'll get multiple responses, so let them all come in
	dwt_setrxtimeout(0);

	// Delay RX?
//	dwt_setrxaftertxdelay(1); // us
    
    uint8_t TimeStamp[5] = {0, 0, 0, 0, 0};
    dwt_readsystime(TimeStamp);
    uint64_t systimestamp = from_lower_40_bits(TimeStamp);

//	uint32_t temp = dwt_readsystimestamphi32();
	//uint32_t delay_time = temp + GLOBAL_PKT_DELAY_UPPER32;
	//(APP_US_TO_DEVICETIMEU32(NODE_DELAY_US) & DELAY_MASK) >> 8
//	uint32_t delay_time = temp + ((convertmicrosectodevicetimeu32(TAG_SEND_POLL_DELAY_US)>>8));
    uint32_t delay_time = (uint32_t)((systimestamp)>>8) + ((convertmicrosectodevicetimeu32(TAG_SEND_POLL_DELAY_US)>>8));
	//uint32_t delay_time = dwt_readsystimestamphi32() + GLOBAL_PKT_DELAY_UPPER32;
	delay_time &= 0xFFFFFFFE; //Make sure last bit is zero
	dwt_setdelayedtrxtime(delay_time);
    uint64_t tSP_tmp = systimestamp + (convertmicrosectodevicetimeu(TAG_SEND_POLL_DELAY_US));
    to_lower_40bits(bcast_msg.tSP,tSP_tmp);
	
	// Write the data
	dwt_writetxdata(tx_frame_length, (uint8_t*) &bcast_msg, 0);

	// Start the transmission
	dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

	// MP bug - TX antenna delay needs reprogramming as it is
	// not preserved
	dwt_settxantennadelay(TX_ANTENNA_DELAY);
}

void send_response(){
    //Reset all the tRRs at the beginning of each poll event
	memset(bcast_msg.tRR, 0, sizeof(bcast_msg.tRR));//TODO: hacked
    //bcast_msg.tRR = 0;//_H = bcast_msg.tRR_L = 0;
    
	// Through tSP (tSF is field after) then +2 for FCS
    uint16_t tx_frame_length = sizeof(bcast_msg);
	memset(bcast_msg.destAddr, 0xFF, 2);

	bcast_msg.seqNum++;
//	bcast_msg.subSeqNum = global_subseq_num;

	// First byte identifies this as a POLL
	bcast_msg.messageType = DWM_SEND_RESPONSE;//MSG_TYPE_TAG_POLL;
    
    bcast_msg.sourceAddr = dwm_status.node_id;

	// Tell the DW1000 about the packet
	dwt_writetxfctrl(tx_frame_length, 0);

	// We'll get multiple responses, so let them all come in
	dwt_setrxtimeout(0);

	// Delay RX?
//	dwt_setrxaftertxdelay(1); // us
    
    uint8_t TimeStamp[5] = {0, 0, 0, 0, 0};
    dwt_readsystime(TimeStamp);
    uint64_t systimestamp = from_lower_40_bits(TimeStamp);

//	uint32_t temp = dwt_readsystimestamphi32();
	//uint32_t delay_time = temp + GLOBAL_PKT_DELAY_UPPER32;
	//(APP_US_TO_DEVICETIMEU32(NODE_DELAY_US) & DELAY_MASK) >> 8
//	uint32_t delay_time = temp + ((convertmicrosectodevicetimeu32(TAG_SEND_POLL_DELAY_US)>>8));
    uint32_t delay_time = (uint32_t)((systimestamp)>>8) + ((convertmicrosectodevicetimeu32(ANC_RESP_DELAY)>>8));
	//uint32_t delay_time = dwt_readsystimestamphi32() + GLOBAL_PKT_DELAY_UPPER32;
	delay_time &= 0xFFFFFFFE; //Make sure last bit is zero
	dwt_setdelayedtrxtime(delay_time);
    uint64_t tSR_tmp = systimestamp + (convertmicrosectodevicetimeu(ANC_RESP_DELAY));
    global_tSR = tSR_tmp;
	
	// Write the data
	dwt_writetxdata(tx_frame_length, (uint8_t*) &bcast_msg, 0);

	// Start the transmission
	dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

	// MP bug - TX antenna delay needs reprogramming as it is
	// not preserved
	dwt_settxantennadelay(TX_ANTENNA_DELAY);
}

void send_final(){
    //Reset all the tRRs at the beginning of each poll event
	memset(bcast_msg.tRR, 0, sizeof(bcast_msg.tRR));//TODO: hacked
    //bcast_msg.tRR = 0;//_H = bcast_msg.tRR_L = 0;
    
	// Through tSP (tSF is field after) then +2 for FCS
    uint16_t tx_frame_length = sizeof(bcast_msg);
	memset(bcast_msg.destAddr, 0xFF, 2);

	bcast_msg.seqNum++;
//	bcast_msg.subSeqNum = global_subseq_num;

	// First byte identifies this as a POLL
	bcast_msg.messageType = DWM_SEND_FINAL;//MSG_TYPE_TAG_POLL;
    
    bcast_msg.sourceAddr = dwm_status.node_id;

	// Tell the DW1000 about the packet
	dwt_writetxfctrl(tx_frame_length, 0);

	// We'll get multiple responses, so let them all come in
	dwt_setrxtimeout(0);

	// Delay RX?
//	dwt_setrxaftertxdelay(1); // us
    
    uint8_t TimeStamp[5] = {0, 0, 0, 0, 0};
    dwt_readsystime(TimeStamp);
    uint64_t systimestamp = from_lower_40_bits(TimeStamp);

    unsigned i;
    for(i=0;i<NUM_TOTAL_NODES;++i){
        to_lower_40bits(bcast_msg.tRR[i], global_tag_anchor_resp_rx_time[i]);
    }
    
    uint32_t delay_time = (uint32_t)((systimestamp)>>8) + ((convertmicrosectodevicetimeu32(TAG_SEND_FINAL_DELAY_US)>>8));
    
	delay_time &= 0xFFFFFFFE; //Make sure last bit is zero
	dwt_setdelayedtrxtime(delay_time);
    to_lower_40bits(bcast_msg.tSF,systimestamp + (convertmicrosectodevicetimeu(TAG_SEND_FINAL_DELAY_US)));
	
	// Write the data
	dwt_writetxdata(tx_frame_length, (uint8_t*) &bcast_msg, 0);

	// Start the transmission
	dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

	// MP bug - TX antenna delay needs reprogramming as it is
	// not preserved
	dwt_settxantennadelay(TX_ANTENNA_DELAY);
}

void send_distances()
{
     //Reset all the tRRs at the beginning of each event
	memset(bcast_msg.tRR, 0, sizeof(bcast_msg.tRR));//TODO: hacked
    
	// Through tSP (tSF is field after) then +2 for FCS
    uint16_t tx_frame_length = sizeof(bcast_msg);
	memset(bcast_msg.destAddr, 0xFF, 2);

	bcast_msg.seqNum++;
	// First byte identifies this as a POLL
	bcast_msg.messageType = DWM_SEND_FINAL;//MSG_TYPE_TAG_POLL;
    
    bcast_msg.sourceAddr = dwm_status.node_id;

	// Tell the DW1000 about the packet
	dwt_writetxfctrl(tx_frame_length, 0);

	// We'll get multiple responses, so let them all come in
	dwt_setrxtimeout(0);
    
    uint8_t TimeStamp[5] = {0, 0, 0, 0, 0};
    dwt_readsystime(TimeStamp);
    uint64_t systimestamp = from_lower_40_bits(TimeStamp);

    unsigned i;
    for(i=0;i<NUM_TOTAL_NODES;++i){
        memcpy(bcast_msg.tRR[i],dwm_status.distance_mm[i],sizeof(uint16_t));
    }
    
    uint32_t delay_time = (uint32_t)((systimestamp)>>8) + ((convertmicrosectodevicetimeu32(FIXED_SEND_DISTANCES_DELAY_US)>>8));
    
	delay_time &= 0xFFFFFFFE; //Make sure last bit is zero
	dwt_setdelayedtrxtime(delay_time);
    to_lower_40bits(bcast_msg.tSF,systimestamp + (convertmicrosectodevicetimeu(FIXED_SEND_DISTANCES_DELAY_US)));
	
	// Write the data
	dwt_writetxdata(tx_frame_length, (uint8_t*) &bcast_msg, 0);

	// Start the transmission
	dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

	// MP bug - TX antenna delay needs reprogramming as it is
	// not preserved
	dwt_settxantennadelay(TX_ANTENNA_DELAY);
}

void instance_process(){
    static uint16_t ctr = 0;
    if(dwm_status.node_id==0){
        if(ctr%100==0){
            //start a new sequence
            //reset state
            memset(global_tag_anchor_resp_rx_time,0x0,sizeof(global_tag_anchor_resp_rx_time));
            memset(global_tRR,0x0,sizeof(global_tRR));
            memset(global_tRP,0x0,sizeof(global_tRP));
            memset(global_tRF,0x0,sizeof(global_tRF));
            memset(global_tSP,0x0,sizeof(global_tSP));
            memset(global_tSF,0x0,sizeof(global_tSF));
            
            dwt_forcetrxoff();
            send_poll();
            
            //kick start state machine
            dwm_status.tx_state = DWM_SEND_RESPONSE;
            dwm_status.timer_func(20000,dwt_timer_cb);
        }
    }
    
    ctr++;
    if(ctr>=100){
        ctr = 0;
    }
}

void incr_subsequence_counter(){
    global_subseq_num++;
    if(global_subseq_num > NUM_ANTENNAS*NUM_ANTENNAS*NUM_CHANNELS+1){
        global_subseq_num = 0;
    }
}

// -------------------------------------------------------------------------------------------------------------------
// convert microseconds to device time
uint32_t convertmicrosectodevicetimeu32 (double microsecu)
{
    uint32_t dt;
    long double dtime;

    dtime = (microsecu / (double) DWT_TIME_UNITS) / 1e6 ;

    dt =  (uint32_t) (dtime) ;

    return dt;
}

// convert microseconds to device time
uint64_t convertmicrosectodevicetimeu (double microsecu)
{
    uint64_t dt;
    long double dtime;

    dtime = (microsecu / (double) DWT_TIME_UNITS) / 1e6 ;

    dt =  (uint64_t) (dtime) ;

    return dt;
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
        rec[cur_byte] = (uint8_t)(SPI2BUF&0xFF);
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

 
void dwm_compute_distances()
{
    long double tmp, tmp2;
    uint64_t tRF_tRP;
    uint64_t tSF_tSP;
    uint64_t tRF_tSR;
    uint64_t tSF_tRR;
    uint64_t tRR_tmp;
    uint64_t tSF_tmp;
    uint64_t local_tSR;
    unsigned i;

    for (i = 0; i < NUM_TOTAL_NODES; ++i) {
        if(global_received_poll[i] && global_received_response[i] && global_received_final[i]){
        // Correct TAG Times if rollover
        tRR_tmp = global_tRR[i];
        tSF_tmp = global_tSF[i];

        if (global_tSP[i] > tRR_tmp) {
            tRR_tmp += TWOPOWER40;
            tSF_tmp += TWOPOWER40;
        }
        if (tRR_tmp > tSF_tmp) {
            tSF_tmp += TWOPOWER40;
        }
        global_tRR[i] = tRR_tmp;
        global_tSF[i] = tSF_tmp;
        
        local_tSR = global_tSR;
        // Correct ANCHOR times if rollover
        if (global_tRP[i] > local_tSR) {
            local_tSR += TWOPOWER40;
            global_tRF[i] += TWOPOWER40;
        }
        if (local_tSR > global_tRF[i]) {
            global_tRF[i] += TWOPOWER40;
        }

        tRF_tRP = global_tRF[i] - global_tRP[i];
        tSF_tSP = global_tSF[i] - global_tSP[i];
        tRF_tSR = global_tRF[i] - local_tSR;
        tSF_tRR = global_tSF[i] - global_tRR[i];

        tmp = (long double) tRF_tRP;
        tmp2 = (long double) tSF_tSP;
        if (i == 0) {
            dwm_status.distance[2] = 1;
        }
        if (tSF_tSP == 0) {
            tmp /= tmp2 + 1; //aot (divide by zero crashes the PIC)
        } else {
            tmp /= tmp2; //aot
        }
        tmp2 = (long double) tSF_tRR;
        tmp *= tmp2; //(tSF - tRR) * aot
        tmp *= -1.; //-(tSF - tRR) * aot
        tmp2 = (long double) tRF_tSR;
        tmp += tmp2; //tTOF
        tmp /= 426.40678517020814; //dist ==  DWT_TIME_UNITS) / 2 * SPEED_OF_LIGHT ;
        //aot = (tRF - tRP) / (tSF - tSP);
        //tTOF = (tRF - tSR)-(tSF - tRR) * aot;
        //dist = (tTOF * DWT_TIME_UNITS) / 2;
        //dist *= SPEED_OF_LIGHT;

        dwm_status.distance[i] = (double) tmp;
#ifdef CONF71
        dwm_status.distance_mm[i] = (uint16_t)((dwm_status.distance[i]-154.274)*1000.); //constant offset
#else
        dwm_status.distance_mm[i] = (uint16_t)((dwm_status.distance[i]-150)*1000.); //constant offset
#endif
        } else {
            //didn't receive all necessary data
            dwm_status.distance[i] = 0;
            dwm_status.distance_mm[i] = 0; 
        }
        //reset flags
        global_received_poll[i] = 0;
        global_received_response[i] = 0;
        global_received_final[i] = 0;
    }
}

void dwt_timer_interrupt()
{
    dwm_status.timer_interrupt = 0;
    
    switch(dwm_status.tx_state){
        case DWM_SEND_POLL:
            dwm_status.tx_state = DWM_SEND_RESPONSE;
            dwt_forcetrxoff();
            send_poll();
            dwm_status.timer_func(20000+1000*dwm_status.node_id,dwt_timer_cb);
            break;
        case DWM_SEND_RESPONSE:
            dwm_status.tx_state = DWM_SEND_FINAL;
            dwt_forcetrxoff();
            send_response();
            dwm_status.timer_func(20000,dwt_timer_cb);
            break;
        case DWM_SEND_FINAL:
            dwm_status.tx_state = DWM_COMPUTE_TOF;
            dwt_forcetrxoff();
            send_final();
            dwm_status.timer_func(20000-1000*dwm_status.node_id,dwt_timer_cb);
            break;
        case DWM_COMPUTE_TOF:
            if(dwm_status.node_id>=NUM_FLOATING_NODES){
                dwm_status.tx_state = DWM_SEND_DISTANCES; //fixed nodes need to broadcast measurement results
                dwm_status.timer_func(20000+1000*(dwm_status.node_id-NUM_FLOATING_NODES),dwt_timer_cb);
            } else {
                dwm_status.tx_state = DWM_SEND_POLL;
            }
            //TODO: compute results
            dwm_compute_distances();
            break;
        case DWM_SEND_DISTANCES:
            dwm_status.tx_state = DWM_SEND_POLL;
            dwt_forcetrxoff();
            send_distances();
            break;
        default:
            //error!!!
            dwm_status.tx_state = DWM_SEND_POLL;
            break;
    };
}
/**
 * This interrupt function is tied to the IRQ pin from the DWM1000 module
 */
void __attribute__((__interrupt__, __no_auto_psv__)) _INT2Interrupt(void)
{
    // This is the interrupt handler used by the DWM1000 API library.
    dwm_status.irq_enable = 1;
    IFS1bits.INT2IF = 0;
}
