/*! ----------------------------------------------------------------------------
 *  @file    instance.h
 *  @brief   DecaWave header for application level instance
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#ifndef _INSTANCE_H_
#define _INSTANCE_H_

#ifdef __cplusplus
extern "C" {
#endif

//#include "instance_sws.h"
#include "deca_types.h"
#include "deca_device_api.h"
#include "../dwm_spi.h"
#include "../dwm_interrupts.h"
#include "../dwm_mutex.h"
#include "../../sensor_state.h"
#include <math.h>

extern timer_data timer_state;

#define Delay(x) Delay_us(x*1000)

#define portGetTickCount() timer_state.systime

/******************************************************************************************************************
********************* NOTES on DW (MP) features/options ***********************************************************
*******************************************************************************************************************/
#define DEEP_SLEEP (0) //To enable deep-sleep set this to 1
//DEEP_SLEEP mode can be used, for example, by a Tag instance to put the DW1000 into low-power deep-sleep mode, there are two cases:
// 1. when the Anchor is sending the range report back to the Tag, the Tag will enter sleep after a ranging exchange is finished
// once it receives a report or times out, before the next poll message is sent (before next ranging exchange is started).
// 2. when the Anchor is not sending the report the Tag will go automatically to sleep after the final message is sent

#define DEEP_SLEEP_XTAL_ON (0)
//NOTE: on the EVK1000 the DEEPSLEEP is not actually putting the DW1000 into full DEEPSLEEP mode as XTAL is kept on


#define ENABLE_AUTO_ACK		(0)		//To enable auto-ack feature set this to 1, frame filtering also needs to be set (to allow ACK frames)
#define ACK_RESPONSE_TIME	(5)     //ACK response time is 5 us (5 symb.), the instance receiving an ACK request will send the ACK after this delay.
#define WAIT_FOR_RESPONSE_DLY	(0) //Tx to Rx delay time, the instance waiting for response will delay turning the receiver on by this amount
// The receiver will be enabled automatically after frame transmission if DWT_RESPONSE_EXPECTED is set in dwt_starttx.
// The instance requesting an ACK, can also program a delay in turning on its receiver, if it knows that the ACK will be sent after particular delay.
// Here it is set to 0 us, which will enable the receiver a.s.a.p so it is ready for the incoming ACK message. 
// The minimum ACK response time about 5us, and the IEEE standard, specifies that the ACK has to be able to be sent within 12 us of reception of an ACK request frame.


//Note:	Double buffer mode can only be used with auto rx re-enable, auto rx re-enable is on by default in Listener and Anchor instances
#define DOUBLE_RX_BUFFER (0) //To enable double RX buffer set this to 1 - this only works for the Listener instance
//NOTE: this feature is really meant for a RX only instance, as TX will not be possible while double-buffer and auto rx-enable is on.

#define DR_DISCOVERY	(1) //to use discovery ranging mode (tag will blink until it receives ranging request from an anchor)
							//after which it will pair with that anchor and start ranging exchange

#define CORRECT_RANGE_BIAS  (1)     // Compensate for small bias due to uneven accumulator growth at close up high power

#define DEEP_SLEEP_AUTOWAKEUP (0) //to test SLEEP mode

#define PUT_TEMP_VOLTAGE_INTO_POLL  (0)     // to insert wakeup sampled TEMP/VOLTAGE into POLL message


#define TWSYMRANGE	1   //when set to 0, the tag calculates range based on Poll Tx and 2 Response Rx times (Poll, Response, Response) method
						//when set to 1, symmetric two way ranging method Poll, Response, Final

/******************************************************************************************************************
*******************************************************************************************************************
*******************************************************************************************************************/

#define NUM_INST            1
#define SPEED_OF_LIGHT      (299702547.0)     // in m/s in air
#define MASK_40BIT          (0x00FFFFFFFFFF)  // MP counter is 40 bits
#define MASK_TXDTS          (0x00FFFFFFFE00)  //The TX timestamp will snap to 8 ns resolution - mask lower 9 bits.


#define USING_64BIT_ADDR (1) //when set to 0 - the DecaRanging application will use 16-bit addresses

#if (DR_DISCOVERY == 0) && (USING_64BIT_ADDR == 0)
	#pragma message (__FILE__" When DISCOVERY mode is not used the application requires that 64-bit addresses are used.")
#endif

#define EXTRA_LENGTH	 (2)

#define SIG_RX_ACK				5		// Frame Received is an ACK (length 5 bytes)
#define SIG_RX_BLINK			7		// Received ISO EUI 64 blink message
#define SIG_RX_BLINKDW			8		// Received ISO EUI 64 DW blink message
#define SIG_RX_UNKNOWN			99		// Received an unknown frame

//Fast 2WR function codes
#define RTLS_DEMO_MSG_RNG_INIT              (0x20)          // Ranging initiation message
#define RTLS_DEMO_MSG_TAG_POLL              (0x21)          // Tag poll message
#define RTLS_DEMO_MSG_ANCH_RESP             (0x10)          // Anchor response to poll
#define RTLS_DEMO_MSG_TAG_FINAL             (0x29)          // Tag final massage back to Anchor (0x29 because of 5 byte timestamps needed for PC app)
#define RTLS_DEMO_MSG_ANCH_TOFR             (0x2A)          // Anchor TOF Report message

//#define RTLS_DEMO_MSG_RNG_INITF              (0x60)          // Ranging initiation message
#define RTLS_DEMO_MSG_TAG_POLLF              (0x61)          // Tag poll message
#define RTLS_DEMO_MSG_ANCH_RESPF             (0x50)          // Anchor response to poll
#define RTLS_DEMO_MSG_TAG_FINALF             (0x69)          // Tag final massage back to Anchor
#define RTLS_DEMO_MSG_ANCH_TOFRF             (0x6A)          // Anchor TOF Report message


//lengths including the Decaranging Message Function Code byte
#define TAG_POLL_F_MSG_LEN                    1				// FunctionCode(1),
#define ANCH_RESPONSE_F_MSG_LEN               1               // FunctionCode(1),
#define TAG_FINAL_F_MSG_LEN                   9              // FunctionCode(1), (Final_TxTime - Resp_RxTime) (4), (Resp_RxTime - Poll_TxTime) (4),
#define TOF_REPORT_F_MSG_LEN                  5               // FunctionCode(1), Measured_TOF_Time(4)

//lengths including the Decaranging Message Function Code byte
#define TAG_POLL_MSG_LEN                    3				// FunctionCode(1), Temp (1), Volt (1)
#define ANCH_RESPONSE_MSG_LEN               4               // FunctionCode(1), RespOption (1), OptionParam(2)
#define TAG_FINAL_MSG_LEN                   16              // FunctionCode(1), Poll_TxTime(5), Resp_RxTime(5), Final_TxTime(5)
#define TOF_REPORT_MSG_LEN                  6               // FunctionCode(1), Measured_TOF_Time(5)
#define RANGINGINIT_MSG_LEN					5				// FunctionCode(1), Tag Address (2), Response Time (2)

#define MAX_MAC_MSG_DATA_LEN                (TAG_FINAL_MSG_LEN) //max message len of the above

#define STANDARD_FRAME_SIZE         127

#define ADDR_BYTE_SIZE_L            (8)
#define ADDR_BYTE_SIZE_S            (2)

#define FRAME_CONTROL_BYTES         2
#define FRAME_SEQ_NUM_BYTES         1
#define FRAME_PANID                 2
#define FRAME_CRC					2
#define FRAME_SOURCE_ADDRESS_S        (ADDR_BYTE_SIZE_S)
#define FRAME_DEST_ADDRESS_S          (ADDR_BYTE_SIZE_S)
#define FRAME_SOURCE_ADDRESS_L        (ADDR_BYTE_SIZE_L)
#define FRAME_DEST_ADDRESS_L          (ADDR_BYTE_SIZE_L)
#define FRAME_CTRLP					(FRAME_CONTROL_BYTES + FRAME_SEQ_NUM_BYTES + FRAME_PANID) //5
#define FRAME_CRTL_AND_ADDRESS_L    (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_L + FRAME_CTRLP) //21 bytes for 64-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_S    (FRAME_DEST_ADDRESS_S + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP) //9 bytes for 16-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_LS	(FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP) //15 bytes for 1 16-bit address and 1 64-bit address)
#define MAX_USER_PAYLOAD_STRING_LL     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_L-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 21 - 16 - 2 = 88
#define MAX_USER_PAYLOAD_STRING_SS     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_S-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 9 - 16 - 2 = 100
#define MAX_USER_PAYLOAD_STRING_LS     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_LS-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 15 - 16 - 2 = 94

//NOTE: the user payload assumes that there are only 88 "free" bytes to be used for the user message (it does not scale according to the addressing modes)
#define MAX_USER_PAYLOAD_STRING	MAX_USER_PAYLOAD_STRING_LL

#define BLINK_FRAME_CONTROL_BYTES       1
#define BLINK_FRAME_SEQ_NUM_BYTES       1
#define BLINK_FRAME_CRC					2
#define BLINK_FRAME_SOURCE_ADDRESS      8
#define BLINK_FRAME_CTRLP				(BLINK_FRAME_CONTROL_BYTES + BLINK_FRAME_SEQ_NUM_BYTES) //2
#define BLINK_FRAME_CRTL_AND_ADDRESS    (BLINK_FRAME_SOURCE_ADDRESS + BLINK_FRAME_CTRLP) //10 bytes

#define ANCHOR_LIST_SIZE			(4)
#define TAG_LIST_SIZE				(1)	//anchor will range with 1st Tag it gets blink from

#define SEND_TOF_REPORT				(1)	//use this to set sendTOFR2Tag parameter if the anchor sends the report back to the tag
#define NO_TOF_REPORT				(0)

#define BLINK_SLEEP_DELAY					1000 //ms
#define POLL_SLEEP_DELAY					500 //ms

//NOTE: only one transmission is allowed in 1ms when using smart tx power setting (applies for 6.81Mb data rate)
//blink tx time ~ 180us with 128 preamble length
//so TX time is 180us, then idle 600us, then rx for 220us, then idle until it sends a poll.
//anchor is in rx, then sends a response 180us, then idle for 300us, then receives a final 220us after which
//it goes back to idle/rx. Once the tag processes the response it prepares the final message and after transmitting
//it goes to sleep. the minimum sleep/idle time before next poll transmission is 880us.
//so the tag sends a poll (176us), then idle for (312us) then rx for (192us (TO is set to 198us)), then idle again for 320us,
//thus time from poll tx to final tx start is 176 + 312 + 192 + 320 = 1000 us.
//this can be seen on the scope when GPIO5 is configured as EXTTXE output and GPIO6 as EXTRXE output
//NOTE2: the ranging init message in this mode does not request an ACK as it would violate the 1ms regulation spec.

#define IDLE_TIME						(225) //this time is added to the FIXED_RP_REPLY_DELAY_US to meet the above requirement

#define FIXED_RP_REPLY_DELAY_US         (275+IDLE_TIME)  //us //response delay time during ranging (Poll RX to Resp TX, Resp RX to Final TX)
#define FIXED_DP_REPLY_DELAY_US         600 //us  //response delay time before sending the Ranging Init

#if(FIXED_RP_REPLY_DELAY_US < 480)
	#pragma message (__FILE__" See note above when using 6.81Mb data rate only one transmission per 1ms is allowed")
#endif



#define MAX_NUMBER_OF_REPORT_RETRYS (3)         // max number of times to send/re-send the report message


#define FINAL_MSG_OFFSET					(30)  //offset in the TX buffer where the Final message starts
#define RESPONSE_MSG_OFFSET					(30)  //offset in the TX buffer where the Response message starts
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// NOTE: the maximum RX timeout is ~ 65ms
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#define INST_DONE_WAIT_FOR_NEXT_EVENT   1   //this signifies that the current event has been processed and instance is ready for next one
#define INST_DONE_WAIT_FOR_NEXT_EVENT_TO    2   //this signifies that the current event has been processed and that instance is waiting for next one with a timeout
                                        //which will trigger if no event coming in specified time
#define INST_NOT_DONE_YET               0   //this signifies that the instance is still processing the current event

//application data message byte offsets
#define FCODE                               0               // Function code is 1st byte of messageData
#define TOFR                                1
#define RES_R1                              1               // Response option octet 0x02 (1),
#define RES_R2                              2               // Response option paramter 0x00 (1) - used to notify Tag that the report is coming
#define RES_T1                              3               // Ranging request response delay low byte
#define RES_T2                              4               // Ranging request response delay high byte


#define ACK_REQUESTED                       (1)             // Request an ACK frame


//response delay time (Tag or Anchor when sending Final/Response messages respectively)
#define FIXED_REPLY_DELAY       			150
#define FIXED_LONG_BLINK_RESPONSE_DELAY       (5*FIXED_REPLY_DELAY) //NOTE: this should be a multiple of FIXED_LONG_REPLY_DELAY see DELAY_MULTIPLE below
#define DELAY_MULTIPLE				(FIXED_LONG_BLINK_RESPONSE_DELAY/FIXED_LONG_REPLY_DELAY - 1)

typedef enum instanceModes{LISTENER, TAG, ANCHOR, TAG_TDOA, NUM_MODES} INST_MODE;

//Listener = in this mode, the instance only receives frames, does not respond
//Tag = Exchanges DecaRanging messages (Poll-Response-Final) with Anchor and enabling Anchor to calculate the range between the two instances
//Anchor = see above

typedef enum inst_states
{
    TA_INIT, //0

    TA_TXE_WAIT,                //1
    TA_TXPOLL_WAIT_SEND,        //2
    TA_TXFINAL_WAIT_SEND,       //3
    TA_TXRESPONSE_WAIT_SEND,    //4
    TA_TXREPORT_WAIT_SEND,      //5
    TA_TX_WAIT_CONF,            //6

    TA_RXE_WAIT,                //7
    TA_RX_WAIT_DATA,            //8

    TA_SLEEP_DONE,              //9
    TA_TXBLINK_WAIT_SEND,       //10
    TA_TXRANGINGINIT_WAIT_SEND  //11
} INST_STATES;


// This file defines data and functions for access to Parameters in the Device
//message structure for Poll, Response and Final message

typedef struct
{
    uint8_t frameCtrl[2];                         	//  frame control bytes 00-01
    uint8_t seqNum;                               	//  sequence_number 02
    uint8_t panID[2];                             	//  PAN ID 03-04
    uint8_t destAddr[ADDR_BYTE_SIZE_L];             	//  05-12 using 64 bit addresses
    uint8_t sourceAddr[ADDR_BYTE_SIZE_L];           	//  13-20 using 64 bit addresses
    uint8_t messageData[MAX_USER_PAYLOAD_STRING_LL] ; //  22-124 (application data and any user payload)
    uint8_t fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dlsl ;

typedef struct
{
    uint8_t frameCtrl[2];                         	//  frame control bytes 00-01
    uint8_t seqNum;                               	//  sequence_number 02
    uint8_t panID[2];                             	//  PAN ID 03-04
    uint8_t destAddr[ADDR_BYTE_SIZE_S];             	//  05-06
    uint8_t sourceAddr[ADDR_BYTE_SIZE_S];           	//  07-08
    uint8_t messageData[MAX_USER_PAYLOAD_STRING_SS] ; //  09-124 (application data and any user payload)
    uint8_t fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dsss ;

typedef struct
{
    uint8_t frameCtrl[2];                         	//  frame control bytes 00-01
    uint8_t seqNum;                               	//  sequence_number 02
    uint8_t panID[2];                             	//  PAN ID 03-04
    uint8_t destAddr[ADDR_BYTE_SIZE_L];             	//  05-12 using 64 bit addresses
    uint8_t sourceAddr[ADDR_BYTE_SIZE_S];           	//  13-14
    uint8_t messageData[MAX_USER_PAYLOAD_STRING_LS] ; //  15-124 (application data and any user payload)
    uint8_t fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dlss ;

typedef struct
{
    uint8_t frameCtrl[2];                         	//  frame control bytes 00-01
    uint8_t seqNum;                               	//  sequence_number 02
    uint8_t panID[2];                             	//  PAN ID 03-04
    uint8_t destAddr[ADDR_BYTE_SIZE_S];             	//  05-06
    uint8_t sourceAddr[ADDR_BYTE_SIZE_L];           	//  07-14 using 64 bit addresses
    uint8_t messageData[MAX_USER_PAYLOAD_STRING_LS] ; //  15-124 (application data and any user payload)
    uint8_t fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dssl ;

//12 octets for Minimum IEEE ID blink
typedef struct
{
    uint8_t frameCtrl;                         		//  frame control bytes 00
    uint8_t seqNum;                               	//  sequence_number 01
    uint8_t tagID[BLINK_FRAME_SOURCE_ADDRESS];        //  02-09 64 bit address
    uint8_t fcs[2] ;                              	//  10-11  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} iso_IEEE_EUI64_blink_msg ;

//18 octets for IEEE ID blink with Temp and Vbat values
typedef struct
{
    uint8_t frameCtrl;                         		//  frame control bytes 00
    uint8_t seqNum;                               	//  sequence_number 01
    uint8_t tagID[BLINK_FRAME_SOURCE_ADDRESS];        //  02-09 64 bit addresses
	uint8_t enchead[2];								//  10-11 2 bytes (encoded header and header extension)
	uint8_t messageID;								//  12 message ID (0xD1) - DecaWave message
	uint8_t temp;										//  13 temperature value
	uint8_t vbat;										//  14 voltage value
	uint8_t gpio;										//  15 gpio status
    uint8_t fcs[2] ;                              	//  16-17  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} iso_IEEE_EUI64_blinkdw_msg ;

typedef struct
{
    uint8_t frameCtrl[2];                         	//  frame control bytes 00-01
    uint8_t seqNum;                               	//  sequence_number 02
    uint8_t fcs[2] ;                              	//  03-04  CRC
} ack_msg ;

typedef struct
{
    uint8_t channelNumber ;       // valid range is 1 to 11
    uint8_t preambleCode ;        // 00 = use NS code, 1 to 24 selects code
    uint8_t pulseRepFreq ;        // NOMINAL_4M, NOMINAL_16M, or NOMINAL_64M
    uint8_t dataRate ;            // DATA_RATE_1 (110K), DATA_RATE_2 (850K), DATA_RATE_3 (6M81)
    uint8_t preambleLen ;         // values expected are 64, (128), (256), (512), 1024, (2048), and 4096
    uint8_t pacSize ;
    uint8_t nsSFD ;
    uint16_t sfdTO;  //!< SFD timeout value (in symbols) e.g. preamble length (128) + SFD(8) - PAC + some margin ~ 135us... DWT_SFDTOC_DEF; //default value
} instanceConfig_t ;

#if (DR_DISCOVERY == 0)
typedef struct
{
	uint64_t forwardToFRAddress;
    uint64_t anchorAddress;
	uint64_t *anchorAddressList;
	int anchorListSize ;
	int anchorPollMask ;
	int sendReport ;
} instanceAddressConfig_t ;
#endif

typedef struct
{
	uint32_t icid;

	dwt_rxdiag_t diag;

#if DECA_LOG_ENABLE==1
    int         accumLogging ;                                // log data to a file, used to indicate that we are currenty logging (file is open)
	FILE        *accumLogFile ;                               // file
#endif

} devicelogdata_t ;

/******************************************************************************************************************
*******************************************************************************************************************
*******************************************************************************************************************/

#define MAX_EVENT_NUMBER (10)
//NOTE: Accumulators don't need to be stored as part of the event structure as when reading them only one RX event can happen...
//the receiver is singly buffered and will stop after a frame is received

typedef struct
{
	uint8_t  type;			// event type
	uint8_t  type2;			// holds the event type - does not clear (not used to show if event has been processed)
	//uint8_t  broadcastmsg;	// specifies if the rx message is broadcast message
	uint16_t rxLength ;

	uint64_t timeStamp ;		// last timestamp (Tx or Rx)

	uint32_t timeStamp32l ;		   // last tx/rx timestamp - low 32 bits
	uint32_t timeStamp32h ;		   // last tx/rx timestamp - high 32 bits

	union {
			//holds received frame (after a good RX frame event)
			uint8_t   frame[STANDARD_FRAME_SIZE];
    		srd_msg_dlsl rxmsg_ll ; //64 bit addresses
			srd_msg_dssl rxmsg_sl ;
			srd_msg_dlss rxmsg_ls ;
			srd_msg_dsss rxmsg_ss ; //16 bit addresses
			ack_msg rxackmsg ; //holds received ACK frame
			iso_IEEE_EUI64_blink_msg rxblinkmsg;
			iso_IEEE_EUI64_blinkdw_msg rxblinkmsgdw;
	}msgu;

	//uint32_t eventtime ;
	//uint32_t eventtimeclr ;
	//uint8_t gotit;
}event_data_t ;

#define RTD_MED_SZ          8      // buffer size for mean of 8

typedef struct{
	uint32_t diffRmP;
	uint32_t diffFmR;
} rtd_t;

typedef struct {
                uint8_t PGdelay;

                //TX POWER
                //31:24     BOOST_0.125ms_PWR
                //23:16     BOOST_0.25ms_PWR-TX_SHR_PWR
                //15:8      BOOST_0.5ms_PWR-TX_PHR_PWR
                //7:0       DEFAULT_PWR-TX_DATA_PWR
                uint32_t txPwr[2]; //
}tx_struct;

typedef struct
{
    INST_MODE mode;				//instance mode (tag or anchor)

    INST_STATES testAppState ;			//state machine - current state
    INST_STATES nextState ;				//state machine - next state
    INST_STATES previousState ;			//state machine - previous state
    int done ;					//done with the current event/wait for next event to arrive

	//configuration structures
	dwt_config_t    configData ;	//DW1000 channel configuration
	dwt_txconfig_t  configTX ;		//DW1000 TX power configuration
	uint16_t			txantennaDelay ; //DW1000 TX antenna delay
	// "MAC" features
	uint8_t rxautoreenable;			//auto RX re-enable (the receiver will re-enable on an errored frame)
	uint8_t doublebufferon;			//double buffer is enabled
    uint8_t frameFilteringEnabled ;	//frame filtering is enabled

#if (DR_DISCOVERY == 0)
	//user payload and address structure for non-discovery mode
    instanceAddressConfig_t payload ;
#endif

	//timeouts and delays
	int tagSleepTime_ms; //in milliseconds
	int tagBlinkSleepTime_ms;
	//this is the delay used for the delayed transmit (when sending the ranging init, response, and final messages)
	uint64_t fixedReplyDelay ;
	int fixedReplyDelay_ms ;

	// xx_sy the units are 1.0256 us
	int fixedReplyDelay_sy ;    // this is the delay used after sending a poll or response and turning on the receiver to receive response or final
	int rnginitW4Rdelay_sy ;	// this is the delay used after sending a blink and turning on the receiver to receive the ranging init message

	int fwtoTime_sy ;	//this is final message duration (longest out of ranging messages)
	int fwtoTimeB_sy ;	//this is the ranging init message duration

    uint32_t fixedRngInitReplyDelay32h ;	//this is the delay after receiving a blink before responding with ranging init message
    uint32_t fixedFastReplyDelay32h ; //this it the is the delay used before sending response or final message

	uint64_t delayedReplyTime;		// delayed reply time of ranging-init/response/final message
	uint32_t delayedReplyTime32;

    uint32_t rxTimeouts ;
	// - not used in the ARM code uint32_t responseTimeouts ;



	//message structures used for transmitted messages
#if (USING_64BIT_ADDR == 1)
	srd_msg_dlsl rng_initmsg ;	// ranging init message (destination long, source long)
    srd_msg_dlsl msg ;			// simple 802.15.4 frame structure (used for tx message) - using long addresses
#else
	srd_msg_dlss rng_initmsg ;  // ranging init message (destination long, source short)
    srd_msg_dsss msg ;			// simple 802.15.4 frame structure (used for tx message) - using short addresses
#endif
	iso_IEEE_EUI64_blink_msg blinkmsg ; // frame structure (used for tx blink message)

//messages used in "fast" ranging ...
	srd_msg_dlss rnmsg ; // ranging init message structure
	srd_msg_dsss msg_f ; // ranging message with 16-bit addresses - used for "fast" ranging

	//Tag function address/message configuration
	uint8_t   eui64[8];				// devices EUI 64-bit address
	uint16_t  tagShortAdd ;		    // Tag's short address (16-bit) used when USING_64BIT_ADDR == 0
	uint16_t  psduLength ;			// used for storing the frame length
    uint8_t   frame_sn;				// modulo 256 frame sequence number - it is incremented for each new frame transmittion
	uint16_t  panid ;					// panid used in the frames

    uint8_t relpyAddress[8] ;

	//64 bit timestamps
	//union of TX timestamps
	union {
		uint64_t txTimeStamp ;		   // last tx timestamp
		uint64_t tagPollTxTime ;		   // tag's poll tx timestamp
	    uint64_t anchorRespTxTime ;	   // anchor's reponse tx timestamp
	}txu;
	uint64_t anchorRespRxTime ;	    // receive time of response message
	uint64_t tagPollRxTime ;          // receive time of poll message

	//32 bit timestamps (when "fast" ranging is used)
	uint32_t tagPollTxTime32l ;      // poll tx time - low 32 bits
	uint32_t tagPollRxTime32l ;      // poll rx time - low 32 bits
	uint32_t anchorRespTxTime32l ;    // response tx time - low 32 bits

	uint32_t anchResp1RxTime32l ;		// response 1 rx time - low 32 bits

	//application control parameters
	uint8_t	ackreq;					// set if the last RX message had ACK request bit set - it is cleared when the ACK tx confirmation is processed
									// the received frame with the ACK request bit set will only be processed once the ACK has been sent
    uint8_t	wait4ack ;				// if this is set to DWT_RESPONSE_EXPECTED, then the receiver will turn on automatically after TX completion
    uint8_t	ackexpected ;			// application has sent ACK request and is waiting for an ACK
	uint8_t	ackTO ;					// using ACK timeout (this means that after an ACK request the RX will timeout in ~5ms)
	uint8_t	newReportSent;
    uint8_t	sentSN;					// sent sequence number
    uint8_t	sendTOFR2Tag ;			// sends report to Tag else forwards to Forwarding Address
    uint8_t	tag2rxReport ;			// tag should get ready to rx report after final message is sent

	uint8_t   instToSleep;			// if set the instance will go to sleep before sending the blink/poll message
	uint8_t	stoptimer;				// stop/disable an active timer
    uint8_t	instancetimer_en;		// enable/start a timer
    uint32_t	instancetimer;			// e.g. this timer is used to timeout Tag when in deep sleep so it can send the next poll message
	// - not used in the ARM code
    //uint8_t	deviceissleeping;		// this disabled reading/writing to DW1000 while it is in sleep mode
									// (DW1000 will wake on chip select so need to disable and chip select line activity)
	uint8_t	gotTO;					// got timeout event

	uint8_t   responseRxNum;			// response number


    //diagnostic counters/data, results and logging
    uint32_t tof32 ;
    uint64_t tof ;
    double clockOffset ;

    uint32_t blinkRXcount ;
	int txmsgcount;
	int	rxmsgcount;
	int lateTX;
	int lateRX;

    double adist[RTD_MED_SZ] ;
    double adist4[4] ;
    double longTermRangeSum ;
    int longTermRangeCount ;
    int tofindex ;
    int tofcount ;
    uint8_t lastReportSN ;
    int last_update ;           // detect changes to status report
	uint8_t    dispClkOffset ;								// Display Clock Offset
    double idistmax;
    double idistmin;
    double idistance ; // instantaneous distance
    int newrange;
    // - not used in the ARM code uint32_t	lastReportTime;
    int respPSC;

    uint8_t canprintinfo ;
	//devicelogdata_t devicelogdata;


	uint8_t tagToRangeWith;	//it is the index of the tagList array which contains the address of the Tag we are ranging with
    uint8_t tagListLen ;
    uint8_t anchorListIndex ;
	uint8_t tagList[TAG_LIST_SIZE][8];


	//event queue - used to store DW1000 events as they are processed by the dw_isr/callback functions
    event_data_t dwevent[MAX_EVENT_NUMBER]; //this holds any TX/RX events and associated message data
	event_data_t saved_dwevent; //holds an RX event while the ACK is being sent
    uint8_t dweventIdxOut;
    uint8_t dweventIdxIn;
	uint8_t dweventPeek;

	int dwIDLE;
} instance_data_t ;

typedef struct
{

	int (*testapprun_fn)(instance_data_t *inst, int message);

} instance_localdata_t ;
//-------------------------------------------------------------------------------------------------------------
//
//	Functions used in logging/displaying range and status data
//
//-------------------------------------------------------------------------------------------------------------

// function to calculate and report the Time of Flight to the GUI/display
void reportTOF(instance_data_t *inst);
void reportTOF_f(instance_data_t *inst);
// clear the status/ranging data 
void instanceclearcounts(void) ;
void instcleartaglist(void);
void instsettagtorangewith(int tagID);
int instaddtagtolist(instance_data_t *inst, uint8_t *tagAddr);

void instance_readaccumulatordata(void);
//-------------------------------------------------------------------------------------------------------------
//
//	Functions used in driving/controlling the ranging application
//
//-------------------------------------------------------------------------------------------------------------

// opent the SPI Cheetah interface - called from inittestapplication()
int instancespiopen(void) ;  // Open SPI and return handle
// close the SPI Cheetah interface  
void instance_close(void);
// Call init, then call config, then call run. call close when finished
// initialise the instance (application) structures and DW1000 device
int instance_init(void);
int instance_init_f(int mode);
int instance_init_s(int mode);

// configure the instance and DW1000 device
void instance_config(instanceConfig_t *config) ;  
void instance_config_f(void);

// configure the antenna delays
void instancesetantennadelays(double fdelay) ;                      // delay in nanoseconds

void instancerxon(instance_data_t *inst, int delayed, uint64_t delayedReceiveTime);
void inst_processackmsg(instance_data_t *inst, uint8_t seqNum);
void inst_processrxtimeout(instance_data_t *inst);

// called (periodically or from and interrupt) to process any outstanding TX/RX events and to drive the ranging application
int instance_run(void) ;       // returns indication of status report change
int testapprun_f(instance_data_t *inst, int message);
int testapprun(instance_data_t *inst, int message);

void instance_setapprun(int (*apprun_fn)(instance_data_t *inst, int message));

// calls the DW1000 interrupt handler
#define instance_process_irq(x) 	dwt_isr()  //call device interrupt handler
// configure TX/RX callback functions that are called from DW1000 ISR
void instance_rxcallback(const dwt_callback_data_t *rxd);
void instance_txcallback(const dwt_callback_data_t *txd);

// sets the Tag sleep delay time (the time Tag "sleeps" between each ranging attempt)
void instancesettagsleepdelay(int rangingsleep, int blinkingsleep);
void instancesetreplydelay(int delayms, int datalength);

//NOTE: this is the delay between receiving the blink and sending the ranging init message
// The anchor ranging init response delay has to match the delay the tag expects
void instancesetblinkreplydelay(int delayms); //

// set/get the instance roles e.g. Tag/Anchor/Listener
void instancesetrole(int mode) ;                // 
int instancegetrole(void) ;
// get the DW1000 device ID (e.g. 0xDECA0130 for MP)
uint32_t instancereaddeviceid(void) ;                                 // Return Device ID reg, enables validation of physical device presence


void instancerxon(instance_data_t *inst, int delayed, uint64_t delayedReceiveTime);
double instance_get_adist(void);

double instance_get_idist(void);

int instance_get_lcount(void);

uint64_t instance_get_addr(void); //get own address (8 bytes)
uint64_t instance_get_tagaddr(void); //get tag address (8 bytes)
uint64_t instance_get_anchaddr(void); //get anchor address (that sent the ToF)

int instancenewrange(void);
int instancesleeping(void);
int instanceanchorwaiting(void);

int instance_get_dly(void);

int instance_get_rxf(void);

int instance_get_txf(void); //get number of Txed frames

int instance_get_respPSC(void);

int instance_get_txl(void) ;
int instance_get_rxl(void) ;

uint32_t convertmicrosectodevicetimeu32 (double microsecu);
uint64_t convertmicrosectodevicetimeu (double microsecu);
double convertdevicetimetosec(int32_t dt);
double convertdevicetimetosec8(uint8_t* dt);

#define DWT_PRF_64M_RFDLY   (514.462f)
#define DWT_PRF_16M_RFDLY   (513.9067f)
extern const uint16_t rfDelays[2];
extern const tx_struct txSpectrumConfig[8];

extern instance_data_t instance_data[NUM_INST] ;

int testapprun_af(instance_data_t *inst, int message);
int testapprun_tf(instance_data_t *inst, int message);

int instance_peekevent(void);

void instance_saveevent(event_data_t newevent);

event_data_t instance_getsavedevent(void);

void instance_putevent(event_data_t newevent);

event_data_t* instance_getevent(int x);

void instance_clearevents(void);

void instance_notify_DW1000_inIDLE(int idle);
#ifdef __cplusplus
}
#endif

#endif
