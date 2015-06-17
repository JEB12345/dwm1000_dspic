/*! ----------------------------------------------------------------------------
 *  @file    instance.c
 *  @brief   DecaWave application level message exchange for ranging demo
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include "../dwm_interrupts.h"
#include "../dwm_spi.h"
#include "../dwm_mutex.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "instance.h"

// -------------------------------------------------------------------------------------------------------------------

//application data message byte offsets
#define FCODE                               0               // Function code is 1st byte of messageData
#define PTXT                                1
#define RRXT                                6
#define FTXT                                11
#define TOFR                                1
#define RES_R1                              1               // Response option octet 0x02 (1),
#define RES_R2                              2               // Response option paramter 0x00 (1) - used to notify Tag that the report is coming
#define RES_R3                              3               // Response option paramter 0x00 (1),
#define RES_T1                              3               // Ranging request response delay low byte
#define RES_T2                              4               // Ranging request response delay high byte
#define POLL_TEMP                           1               // Poll message TEMP octet
#define POLL_VOLT                           2               // Poll message Voltage octet


// -------------------------------------------------------------------------------------------------------------------
//      Data Definitions
// -------------------------------------------------------------------------------------------------------------------
const uint16_t rfDelays[2];
const tx_struct txSpectrumConfig[8];

instance_data_t instance_data[NUM_INST] ;
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// NOTE: the maximum RX timeout is ~ 65ms
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


// -------------------------------------------------------------------------------------------------------------------
// Functions
// -------------------------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------------------------
//
// function to construct the message/frame header bytes
//
// -------------------------------------------------------------------------------------------------------------------
//
void instanceconfigframeheader(instance_data_t *inst, int ackrequest)
{
    inst->msg.panID[0] = (inst->panid) & 0xff;
    inst->msg.panID[1] = inst->panid >> 8;

    //set frame type (0-2), SEC (3), Pending (4), ACK (5), PanIDcomp(6)
    inst->msg.frameCtrl[0] = 0x1 /*frame type 0x1 == data*/ | 0x40 /*PID comp*/;
    inst->msg.frameCtrl[0] |= (ackrequest ? 0x20 : 0x00);
#if (USING_64BIT_ADDR==1)
    //source/dest addressing modes and frame version
    inst->msg.frameCtrl[1] = 0xC /*dest extended address (64bits)*/ | 0xC0 /*src extended address (64bits)*/;
#else
    inst->msg.frameCtrl[1] = 0x8 /*dest short address (16bits)*/ | 0x80 /*src short address (16bits)*/;
#endif
    inst->msg.seqNum = inst->frame_sn++;

}



#if (DR_DISCOVERY == 0)
int destaddress(instance_data_t *inst)
{
    int getnext = 1;
    int tempanchorListIndex = 0;
    //set destination address (Tag will cycle though the list of anchor addresses)

    if(inst->payload.anchorPollMask == 0)
        return 1; //error - list is empty

    while(getnext)
    {
        if((0x1 << inst->anchorListIndex) & inst->payload.anchorPollMask)
        {
            memcpy(&inst->msg.destAddr[0], &inst->payload.anchorAddressList[inst->anchorListIndex], ADDR_BYTE_SIZE_L);
            getnext = 0;
        }

        inst->anchorListIndex++ ;
    }

    tempanchorListIndex = inst->anchorListIndex;

    while(tempanchorListIndex <= inst->payload.anchorListSize)
        {
        //check if there are any anchors left to poll
        if((0x1 << tempanchorListIndex) & inst->payload.anchorPollMask)
        {
            return 0;
        }
        tempanchorListIndex++;
    }

    //if we got this far means that we are just about to poll the last anchor in the list
    inst->instToSleep = 1; //we'll sleep after this poll
    inst->anchorListIndex = 0; //start from the first anchor in the list after sleep finishes

    return 0;
}
#endif


// -------------------------------------------------------------------------------------------------------------------
//
// function to configure the frame data, prior to writing the frame to the TX buffer
//
// -------------------------------------------------------------------------------------------------------------------
//
void setupmacframedata(instance_data_t *inst, int len, int framectrllen, int fcode, int ack)
{
    inst->msg.messageData[FCODE] = fcode; //message function code (specifies if message is a poll, response or other...)

    inst->psduLength = len + framectrllen;

	//inst->psduLength += adduserpayload(inst, len); //add any user data to the message payload

	instanceconfigframeheader(inst, ack);

    if(ack == ACK_REQUESTED)
        inst->wait4ack = DWT_RESPONSE_EXPECTED;

    inst->ackexpected = ack ; //used to ignore unexpected ACK frames
}

// -------------------------------------------------------------------------------------------------------------------
//
// Turn on the receiver with/without delay
//
void instancerxon(instance_data_t *inst, int delayed, uint64_t delayedReceiveTime)
{
    if (delayed)
    {
        uint32_t dtime;
        dtime =  (uint32_t) (delayedReceiveTime>>8);
        dwt_setdelayedtrxtime(dtime) ;
    }

    inst->lateRX -= dwt_rxenable(delayed) ;  //- as when fails -1 is returned             // turn receiver on, immediate/delayed

} // end instancerxon()


int instancesendpacket(instance_data_t *inst, int delayedTx)
{
    int result = 0;

    dwt_writetxfctrl(inst->psduLength, 0);
    if(delayedTx)
    {
        uint32_t dtime;
        dtime = (uint32_t) (inst->delayedReplyTime>>8);
        dwt_setdelayedtrxtime(dtime) ;
    }

    //begin delayed TX of frame
    if (dwt_starttx(delayedTx | inst->wait4ack))  // delayed start was too late
    {
        result = 1; //late/error
        inst->lateTX++;
    }


    return result;                                              // state changes
    // after sending we should return to TX ON STATE ?
}

// -------------------------------------------------------------------------------------------------------------------
//
// the main instance state machine (all the instance modes Tag, Anchor or Listener use the same statemachine....)
//
// -------------------------------------------------------------------------------------------------------------------
//
int testapprun_s(instance_data_t *inst, int message)
{

    switch (inst->testAppState)
    {
        case TA_INIT :
            // printf("TA_INIT") ;
            switch (inst->mode)
            {
                case TAG:
                {
                	int mode = 0;

                    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN); //allow data, ack frames;
                    inst->frameFilteringEnabled = 1 ;
                    dwt_setpanid(inst->panid);
                    dwt_seteui(inst->eui64);
#if (USING_64BIT_ADDR==0)
				    //the short address is assigned by the anchor
#else
                    //set source address into the message structure
                    memcpy(&inst->msg.sourceAddr[0], inst->eui64, ADDR_BYTE_SIZE_L);
#endif

                    //change to next state - send a Poll message to 1st anchor in the list
#if (DR_DISCOVERY == 1)
                    inst->mode = TAG_TDOA ;
                    inst->testAppState = TA_TXBLINK_WAIT_SEND;
				    memcpy(inst->blinkmsg.tagID, inst->eui64, ADDR_BYTE_SIZE_L);
#else
                    inst->testAppState = TA_TXPOLL_WAIT_SEND;
#endif

                    dwt_setautorxreenable(inst->rxautoreenable); //not necessary to auto RX re-enable as the receiver is on for a short time (Tag knows when the response is coming)

                    dwt_setdblrxbuffmode(inst->doublebufferon); //disable double RX buffer

#if (ENABLE_AUTO_ACK == 1) //NOTE - Auto ACK only works if frame filtering is enabled!
                    dwt_enableautoack(ACK_RESPONSE_TIME); //wait for ACK_RESPONSE_TIME symbols (e.g. 5) before replying with the ACK
#endif

                    mode = (DWT_LOADUCODE|DWT_PRESRV_SLEEP|DWT_CONFIG|DWT_TANDV);

					if((dwt_getldotune() != 0)) //if we need to use LDO tune value from OTP kick it after sleep
							mode |= DWT_LOADLDO;

					if(inst->configData.txPreambLength == DWT_PLEN_64)  //if using 64 length preamble then use the corresponding OPSet
						mode |= DWT_LOADOPSET;

#if (DEEP_SLEEP == 1)
#if (DEEP_SLEEP_AUTOWAKEUP == 1)
                    dwt_configuresleep(mode, DWT_WAKE_SLPCNT|DWT_WAKE_CS|DWT_SLP_EN); //configure the on wake parameters (upload the IC config settings)
#else
                    //NOTE: on the EVK1000 the DEEPSLEEP is not actually putting the DW1000 into full DEEPSLEEP mode as XTAL is kept on
#if (DEEP_SLEEP_XTAL_ON == 1)
                    dwt_configuresleep(mode, DWT_WAKE_CS|DWT_SLP_EN|DWT_XTAL_EN); //configure the on wake parameters (upload the IC config settings)
#else
				    dwt_configuresleep(mode, DWT_WAKE_WK|DWT_WAKE_CS|DWT_SLP_EN); //configure the on wake parameters (upload the IC config settings)
#endif
#endif
#endif
                }
                break;
                case ANCHOR:
                {
#if (DR_DISCOVERY == 0)
                    uint8_t eui64[8] ;
                    memcpy(eui64, &inst->payload.anchorAddress, sizeof(uint64_t));
                    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN); //allow data, ack frames;
                    inst->frameFilteringEnabled = 1 ;
                    dwt_seteui(eui64);
#else
                    dwt_enableframefilter(DWT_FF_NOTYPE_EN); //disable frame filtering
                    inst->frameFilteringEnabled = 0 ;
                    dwt_seteui(inst->eui64);
#endif
                    dwt_setpanid(inst->panid);


#if (USING_64BIT_ADDR==0)
                    {
					uint16_t addr = inst->eui64[0] + (inst->eui64[1] << 8);
                        dwt_setaddress16(addr);
					//set source address into the message structure
					memcpy(&inst->msg.sourceAddr[0], inst->eui64, ADDR_BYTE_SIZE_S);
					//set source address into the message structure
					memcpy(&inst->rng_initmsg.sourceAddr[0], inst->eui64, ADDR_BYTE_SIZE_S);
                    }
#else
#if (DR_DISCOVERY == 0)
                    //set source address into the message structure
                	memcpy(&inst->msg.sourceAddr[0], &inst->payload.anchorAddress, ADDR_BYTE_SIZE_L);
#else
                    //set source address into the message structure
                	memcpy(&inst->msg.sourceAddr[0], inst->eui64, ADDR_BYTE_SIZE_L);
					//set source address into the message structure
					memcpy(&inst->rng_initmsg.sourceAddr[0], inst->eui64, ADDR_BYTE_SIZE_L);
#endif
#endif

                    // First time anchor listens we don't do a delayed RX
					dwt_setrxaftertxdelay(0);
                    //change to next state - wait to receive a message
                    inst->testAppState = TA_RXE_WAIT ;
#if (ENABLE_AUTO_ACK == 1) //NOTE - Auto ACK only works if frame filtering is enabled!
                    dwt_setrxaftertxdelay(WAIT_FOR_RESPONSE_DLY); //set the RX after TX delay time
#endif

//NOTE: auto rx re-enable does not stop the rx after sending an ACK in auto ACK mode - so not used here
//#if (DECA_BADF_ACCUMULATOR == 0) //can use RX auto re-enable when not logging/plotting errored frames
					//inst->rxautoreenable = 1;
//#endif
                    dwt_setautorxreenable(inst->rxautoreenable);

                    dwt_setdblrxbuffmode(inst->doublebufferon); //enable double RX buffer

                    dwt_setrxtimeout(0);
					inst->canprintinfo = 1;

                }
                break;
                case LISTENER:
                {
                    dwt_enableframefilter(DWT_FF_NOTYPE_EN); //disable frame filtering
                    inst->frameFilteringEnabled = 0 ;
                    // First time anchor listens we don't do a delayed RX
					dwt_setrxaftertxdelay(0);
                    //change to next state - wait to receive a message
                    inst->testAppState = TA_RXE_WAIT ;

//NOTE: auto rx re-enable does not stop the rx after sending an ACK in auto ACK mode - so not used here
//#if (DECA_BADF_ACCUMULATOR == 0) //can use RX auto re-enable when not logging/plotting errored frames
					//inst->rxautoreenable = 1;
//#endif
                    dwt_setautorxreenable(inst->rxautoreenable);


                    dwt_setdblrxbuffmode(inst->doublebufferon); //enable double RX buffer

                    dwt_setrxtimeout(0);

                }
                break ; // end case TA_INIT
                default:
                break;
            }
            break; // end case TA_INIT

        case TA_SLEEP_DONE :
        {
        	event_data_t* dw_event = instance_getevent(10); //clear the event from the queue
			// waiting for timout from application to wakup IC
			if (dw_event->type != DWT_SIG_RX_TIMEOUT)
			{
				// if no pause and no wake-up timeout continu waiting for the sleep to be done.
                inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT; //wait here for sleep timeout
                break;
            }

            inst->done = INST_NOT_DONE_YET;
            inst->instToSleep = 0;
            inst->testAppState = inst->nextState;
            inst->nextState = 0; //clear
#if (DEEP_SLEEP == 1)
            {
#ifdef _MSC_VER
                uint8_t buffer[1500];
                //wake up device from low power mode
                //1000 takes 400us, 1300 takes 520us, 1500 takes 600us (SPI @ 20MHz)
                //then the function will wait 5ms to make sure the XTAL has stabilised
                if(dwt_spicswakeup(buffer, 1500) == DWT_ERROR) //1000 takes 400us, 1300 takes 520us, 1500 takes 600us (SPI @ 20MHz)
                {
                    printf("FAILED to WAKE UP\n");
                }
#else
                //wake up device from low power mode
                //NOTE - in the ARM  code just drop chip select for 200us
                nSELECT = 0;  //CS low
                instance_data[0].dwIDLE = 0; //reset

                decamutexoff(1); //enable RSTn IRQ

                Delay(1);   //200 us to wake up then waits 5ms for DW1000 XTAL to stabilise
                nSELECT = 1;  //CS high
                //Delay(5);

                //need to poll to check when the DW1000 is in IDLE, the CPLL interrupt is not reliable
                while(instance_data[0].dwIDLE == 0); //wait for DW1000 to go to IDLE state RSTn pin to go high


                decamutexon(); //disable RSTn IRQ
                //add ~ Delay(80) to stabilise the XTAL
                Delay(85);
#endif
                //this is platform dependent - only program if DW EVK/EVB
                dwt_setleds(1);

                //MP bug - TX antenna delay needs reprogramming as it is not preserved
                dwt_settxantennadelay(inst->txantennaDelay) ;

                //set EUI as it will not be preserved unless the EUI is programmed and loaded from NVM
                /*if((inst->mode == TAG) || (inst->mode == TAG_TDOA))
                {
                    dwt_setpanid(inst->panid);
                    dwt_seteui(inst->eui64);
                }*/
                dwt_entersleepaftertx(0);
                dwt_setinterrupt(DWT_INT_TFRS, 1); //re-enable the TX/RX interrupts
            }
#endif
        }
            break;

        case TA_TXE_WAIT : //either go to sleep or proceed to TX a message
            // printf("TA_TXE_WAIT") ;
            //if we are scheduled to go to sleep before next sending then sleep first.
            if(((inst->nextState == TA_TXPOLL_WAIT_SEND)
                || (inst->nextState == TA_TXBLINK_WAIT_SEND))
                    && (inst->instToSleep)  //go to sleep before sending the next poll
                    )
            {
                //the app should put chip into low power state and wake up in tagSleepTime_ms time...
                //the app could go to *_IDLE state and wait for uP to wake it up...
                inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT_TO; //don't sleep here but kick off the TagTimeoutTimer (instancetimer)
                inst->testAppState = TA_SLEEP_DONE;
				if(inst->mode == TAG_TDOA) //once we start ranging we want to display the new range
					inst->canprintinfo = 1;
#if (DEEP_SLEEP == 1)
                //put device into low power mode
                dwt_entersleep(); //go to sleep
				//inst->deviceissleeping = 1; //this is to stop polling device status register (as it will wake it up)
#endif
            }
            else //proceed to configuration and transmission of a frame
            {
#if (DOUBLE_RX_BUFFER == 1)
                dwt_forcetrxoff(); //disable rx
#endif
                inst->testAppState = inst->nextState;
                inst->nextState = 0; //clear
            }
            break ; // end case TA_TXE_WAIT

        case TA_TXBLINK_WAIT_SEND :
            {
				int flength = (BLINK_FRAME_CRTL_AND_ADDRESS + FRAME_CRC);

                //blink frames with IEEE EUI-64 tag ID
                inst->blinkmsg.frameCtrl = 0xC5 ;
                inst->blinkmsg.seqNum = inst->frame_sn++;

				dwt_writetxdata(flength, (uint8_t *)  (&inst->blinkmsg), 0) ;	// write the frame data
				dwt_writetxfctrl(flength, 0);

#if (DEEP_SLEEP_AUTOWAKEUP == 1)
#if (DEEP_SLEEP == 1)
                dwt_entersleepaftertx(1);
				//inst->deviceissleeping = 1; //this is to stop polling device status register (as it will wake it up)
#endif
				inst->wait4ack = 0;
#else
				//using wait for response to do delayed receive
				inst->wait4ack = DWT_RESPONSE_EXPECTED;

				dwt_setrxtimeout((uint16_t)inst->fwtoTimeB_sy);  //units are symbols
				//set the delayed rx on time (the ranging init will be sent after this delay)
				dwt_setrxaftertxdelay((uint32_t)inst->rnginitW4Rdelay_sy);  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)

#endif
				dwt_starttx(DWT_START_TX_IMMEDIATE | inst->wait4ack); //always using immediate TX and enable dealyed RX

				inst->sentSN = inst->blinkmsg.seqNum;

#if (DEEP_SLEEP_AUTOWAKEUP == 0)
				inst->instToSleep = 1; //go to Sleep after this blink
                inst->testAppState = TA_TX_WAIT_CONF ; // wait confirmation
                inst->previousState = TA_TXBLINK_WAIT_SEND ;
                inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set below)
#else
                while(1);       //Tag will continue blinking at defined bilnk rate...
#endif
            }
            break ; // end case TA_TXBLINK_WAIT_SEND

        case TA_TXRANGINGINIT_WAIT_SEND :
                {
				int ack = !ACK_REQUESTED;  //as an option ACK can be requested...
				inst->psduLength = RANGINGINIT_MSG_LEN;

                //tell Tag what it's address will be for the ranging exchange
				inst->rng_initmsg.messageData[FCODE] = RTLS_DEMO_MSG_RNG_INIT;
				inst->rng_initmsg.messageData[RES_R1] = inst->tagShortAdd & 0xFF;
				inst->rng_initmsg.messageData[RES_R2] = (inst->tagShortAdd >> 8) & 0xFF; //
				inst->rng_initmsg.messageData[RES_T1] = ((int) (inst->fixedReplyDelay_ms)) & 0xFF;
				inst->rng_initmsg.messageData[RES_T2] = (((int) (inst->fixedReplyDelay_ms)) >> 8) & 0xFF; //

				inst->rng_initmsg.frameCtrl[0] = 0x41 | (ack << 5); //

#if (USING_64BIT_ADDR == 1)
				inst->rng_initmsg.frameCtrl[1] = 0xCC;
				inst->psduLength += FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC;
#else
				inst->rng_initmsg.frameCtrl[1] = 0x8C;
				inst->psduLength += FRAME_CRTL_AND_ADDRESS_LS + FRAME_CRC;
#endif
				inst->rng_initmsg.panID[0] = (inst->panid) & 0xff;
				inst->rng_initmsg.panID[1] = inst->panid >> 8;

				inst->rng_initmsg.seqNum = inst->frame_sn++;

				//if(ack == ACK_REQUESTED)				response is expected (the Poll message from the Tag)
					inst->wait4ack = DWT_RESPONSE_EXPECTED;

				inst->ackexpected = ack ; //used to ignore unexpected ACK frames
				inst->ackTO = 0; //not using TO

                inst->testAppState = TA_TX_WAIT_CONF;                                               // wait confirmation
                inst->previousState = TA_TXRANGINGINIT_WAIT_SEND ;

				dwt_writetxdata(inst->psduLength, (uint8_t *)  &inst->rng_initmsg, 0) ;	// write the frame data

				//anchor - we don't use timeout, if the ACK is missed we'll get a Poll or Blink
				/*if(inst->wait4ack)
				{
					//if the ACK is requested there is a 5ms timeout to stop RX if no ACK coming
					dwt_setrxtimeout(5000);  //units are us - wait for 5ms after RX on
				}*/

                if(instancesendpacket(inst, DWT_START_TX_DELAYED))
                {
					dwt_setrxaftertxdelay(0);
                    inst->testAppState = TA_RXE_WAIT ;  // wait to receive a new blink or poll message
					inst->wait4ack = 0; //clear the flag as the TX has failed the TRX is off
                }
                else
                {
					inst->sentSN = inst->rng_initmsg.seqNum;
                    inst->testAppState = TA_TX_WAIT_CONF ;                                               // wait confirmation
                    inst->previousState = TA_TXRANGINGINIT_WAIT_SEND ;
                    inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT;  //no timeout
                }

            }
            break;

        case TA_TXPOLL_WAIT_SEND :
            {

#if (DR_DISCOVERY == 1)
                //NOTE the anchor address is set after receiving the ranging initialisation message
				inst->instToSleep = 1; //go to Sleep after this poll
#else
                //set destination address
                if(destaddress(inst))
                {
                    break;
                }
#endif
#if (PUT_TEMP_VOLTAGE_INTO_POLL == 1)
                {
					inst->msg.messageData[POLL_TEMP] = dwt_readwakeuptemp() ;   // Temperature value set sampled at wakeup
					inst->msg.messageData[POLL_VOLT] = dwt_readwakeupvbat() ;   // (Battery) Voltage value set sampled at wakeup
                }
#else
					inst->msg.messageData[POLL_TEMP] = inst->msg.messageData[POLL_VOLT] = 0;
#endif
#if (USING_64BIT_ADDR==1)
				setupmacframedata(inst, TAG_POLL_MSG_LEN, FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC, RTLS_DEMO_MSG_TAG_POLL, !ACK_REQUESTED);
#else
				setupmacframedata(inst, TAG_POLL_MSG_LEN, FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC, RTLS_DEMO_MSG_TAG_POLL, !ACK_REQUESTED);
#endif
				//set the delayed rx on time (the response message will be sent after this delay)
				dwt_setrxaftertxdelay((uint32_t)inst->fixedReplyDelay_sy);  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)
				dwt_setrxtimeout((uint16_t)inst->fwtoTime_sy);  //units are us - wait for 7ms after RX on (but as using delayed RX this timeout should happen at response time + 7ms)

				dwt_writetxdata(inst->psduLength, (uint8_t *)  &inst->msg, 0) ;	// write the frame data

				//response is expected
				inst->wait4ack = DWT_RESPONSE_EXPECTED;

				dwt_writetxfctrl(inst->psduLength, 0);

				dwt_starttx(DWT_START_TX_IMMEDIATE | inst->wait4ack);
				inst->sentSN = inst->msg.seqNum;

                inst->testAppState = TA_TX_WAIT_CONF ;                                               // wait confirmation
                inst->previousState = TA_TXPOLL_WAIT_SEND ;
                inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set below)

            }
            break;

        case TA_TXRESPONSE_WAIT_SEND :
        {
			   //printf("TA_TXRESPONSE\n") ;

				//program option octet and parameters (not used currently)
				inst->msg.messageData[RES_R1] = 0x2; // "activity"
				inst->msg.messageData[RES_R2] = inst->sendTOFR2Tag; //0x0; (this tells the Tag that the ToF report will be sent to it)
				inst->msg.messageData[RES_R3] = 0x0;

#if (USING_64BIT_ADDR==1)
				setupmacframedata(inst, ANCH_RESPONSE_MSG_LEN, FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC, RTLS_DEMO_MSG_ANCH_RESP, !ACK_REQUESTED);
#else
				setupmacframedata(inst, ANCH_RESPONSE_MSG_LEN, FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC, RTLS_DEMO_MSG_ANCH_RESP, !ACK_REQUESTED);
#endif

                inst->testAppState = TA_TX_WAIT_CONF;                                               // wait confirmation
                inst->previousState = TA_TXRESPONSE_WAIT_SEND ;

				//set the delayed rx on time (the final message will be sent after this delay)
				dwt_setrxaftertxdelay((uint32_t)inst->fixedReplyDelay_sy);  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)

				//response is expected
				inst->wait4ack = DWT_RESPONSE_EXPECTED;

				dwt_writetxdata(inst->psduLength, (uint8_t *)  &inst->msg, 0) ;	// write the frame data

                if(instancesendpacket(inst, DWT_START_TX_DELAYED))
                {
                    inst->testAppState = TA_RXE_WAIT ;  // wait to receive a new poll
					dwt_setrxaftertxdelay(0);
					inst->wait4ack = 0; //clear the flag as the TX has failed the TRX is off
                }
                else
                {
					inst->sentSN = inst->msg.seqNum;
                    inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT;  //no timeout
                }
            }
            break;

        case TA_TXFINAL_WAIT_SEND :
            {
                uint64_t tagCalculatedFinalTxTime ;

                // Embbed into Final message: 40-bit pollTXTime,  40-bit respRxTime,  40-bit finalTxTime
                // Write Poll TX time field of Final message
				memcpy(&(inst->msg.messageData[PTXT]), (uint8_t *)&inst->txu.tagPollTxTime, 5);

                // Write Response RX time field of Final message
				memcpy(&(inst->msg.messageData[RRXT]), (uint8_t *)&inst->anchorRespRxTime, 5);

                // Calculate Time Final message will be sent and write this field of Final message
                // Sending time will be delayedReplyTime, snapped to ~125MHz or ~250MHz boundary by
                // zeroing its low 9 bits, and then having the TX antenna delay added
                tagCalculatedFinalTxTime = inst->delayedReplyTime & MASK_TXDTS; // 9 lower bits mask

                // getting antenna delay from the device and add it to the Calculated TX Time
                tagCalculatedFinalTxTime = tagCalculatedFinalTxTime + inst->txantennaDelay;

                tagCalculatedFinalTxTime &= MASK_40BIT;

                // Write Calculated TX time field of Final message
				memcpy(&(inst->msg.messageData[FTXT]), (uint8_t *)&tagCalculatedFinalTxTime, 5);

#if (USING_64BIT_ADDR==1)
				setupmacframedata(inst, TAG_FINAL_MSG_LEN, FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC, RTLS_DEMO_MSG_TAG_FINAL, !ACK_REQUESTED);
#else
				setupmacframedata(inst, TAG_FINAL_MSG_LEN, FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC, RTLS_DEMO_MSG_TAG_FINAL, !ACK_REQUESTED);
#endif

                if(inst->tag2rxReport==0) //if not going to wait for report, go to sleep after TX is complete
                {
#if (DEEP_SLEEP == 1)
                    dwt_entersleepaftertx(1);
					//inst->deviceissleeping = 1; //this is to stop polling device status register
#endif
				}
				else //turn on the receiver to receive the report... as it is coming after the final
				{
					inst->wait4ack = DWT_RESPONSE_EXPECTED;
					dwt_setrxaftertxdelay(0); //the report will come XXX ms so we could use short delay to save power
				}

				dwt_writetxdata(inst->psduLength, (uint8_t *)  &inst->msg, 0) ;	// write the frame data

                if(instancesendpacket(inst, DWT_START_TX_DELAYED))
                {
                    // initiate the re-transmission
                    inst->testAppState = TA_TXE_WAIT ;
                    inst->nextState = TA_TXPOLL_WAIT_SEND ;
#if (DEEP_SLEEP == 1)
					//printf("dealyed TX failed ??!! %08X\n", dwt_read32bitreg(0xf));
                    dwt_entersleepaftertx(0);
#endif
					inst->wait4ack = 0; //clear the flag as the TX has failed the TRX is off
                    break; //exit this switch case...
                }
                else
                {
					inst->sentSN = inst->msg.seqNum;
                    inst->testAppState = TA_TX_WAIT_CONF;                                               // wait confirmation
                    inst->previousState = TA_TXFINAL_WAIT_SEND;
                    inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out  (set below)
					//inst->responseTimeouts = 0; //reset response timeout count
                }

                if(inst->tag2rxReport) //if waiting for report - set timeout to be same as a Sleep timer... if no report coming time out and send another poll
                {
                    dwt_setrxtimeout(0);
					//RX will be turned on 80ms after final is sent, then use Sleep timer to timeout this is longer than needed but as DW1000 timeout is only 65ms long it is too short
					//for the PC application, as the Anchor will send up to 2 reports (if it does not get the ACK) the first of which comes 90-105 ms after the Final TX, and the second
					//will come about 150ms after the Final is sent. The Tag could use a different timer (shorter than the Sleep) to timeout sooner and go to Sleep
                    inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT_TO;
                }
                else //if Tag is not waiting for report - it will go to sleep automatically after the final is sent
                {
#if (DEEP_SLEEP == 1)
                    inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT_TO; //kick off the TagTimeoutTimer (instancetimer) to initiate wakeup
                    inst->nextState = TA_TXPOLL_WAIT_SEND;
                    inst->testAppState = TA_SLEEP_DONE; //we are going automatically to sleep so no TX confirm interrupt (next state will be TA_SLEEP_DONE)
                    inst->txmsgcount ++;
#endif
                }

            }
            break;

        case TA_TXREPORT_WAIT_SEND :
            {

                if(inst->newReportSent == 0) //keep the same message if re-sending the same report
                {
                    // Write calculated TOF into report message
					memcpy(&(inst->msg.messageData[TOFR]), &inst->tof, 5);
                }
                else
                {
                    //re-send the old report
                    inst->msg.seqNum--;
                }

//NOTE - Auto ACK only works if frame filtering is enabled!
#if (USING_64BIT_ADDR==1)
				setupmacframedata(inst, TOF_REPORT_MSG_LEN, FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC, RTLS_DEMO_MSG_ANCH_TOFR, ((ENABLE_AUTO_ACK == 1)?(ACK_REQUESTED):(!ACK_REQUESTED)));
#else
				setupmacframedata(inst, TOF_REPORT_MSG_LEN, FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC, RTLS_DEMO_MSG_ANCH_TOFR, ((ENABLE_AUTO_ACK == 1)?(ACK_REQUESTED):(!ACK_REQUESTED)));
#endif

				dwt_writetxdata(inst->psduLength, (uint8_t *)  &inst->msg, 0) ;	// write the frame data

				//set the delayed rx on time (the ranging init will be sent after this delay)
				//subtract 1ms to make sure the receiver is on before the message comes in
				dwt_setrxaftertxdelay(0);  //units are ~us - wait for wait4respTIM before RX on (delay RX)

				//response is expected
				inst->wait4ack = DWT_RESPONSE_EXPECTED;
				//anchor - we don't use timeout, if the ACK is missed we'll get a Poll or Blink
				if(inst->wait4ack)
                    {
					//if the ACK is requested there is a 5ms timeout to stop RX if no ACK coming
					dwt_setrxtimeout(5000);  //units are us - wait for 5ms after RX on
                }

				dwt_writetxfctrl(inst->psduLength, 0);

				dwt_starttx(DWT_START_TX_IMMEDIATE | inst->wait4ack);

				inst->sentSN = inst->msg.seqNum;
                    inst->testAppState = TA_TX_WAIT_CONF ;                                               // wait confirmation
                    inst->previousState = TA_TXREPORT_WAIT_SEND ;

                    inst->newReportSent++;

				//use anchor rx timeout to timeout and re-send the ToF report
                    inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT_TO;

            }
            break;


        case TA_TX_WAIT_CONF :
		   //printf("TA_TX_WAIT_CONF %d m%d %d states %08x %08x\n", inst->previousState, message, inst->newReportSent, dwt_read32bitreg(0x19), dwt_read32bitreg(0x0f)) ;

                {
				event_data_t* dw_event = instance_getevent(11); //get and clear this event

                //NOTE: Can get the ACK before the TX confirm event for the frame requesting the ACK
                //this happens because if polling the ISR the RX event will be processed 1st and then the TX event
                //thus the reception of the ACK will be processed before the TX confirmation of the frame that requested it.
				if(dw_event->type != DWT_SIG_TX_DONE) //wait for TX done confirmation
                {
					if(dw_event->type == DWT_SIG_RX_TIMEOUT) //got RX timeout - i.e. did not get the response (e.g. ACK)
					{
						//printf("RX timeout in TA_TX_WAIT_CONF (%d)\n", inst->previousState);
						//we need to wait for SIG_TX_DONE and then process the timeout and re-send the frame if needed
						inst->gotTO = 1;
					}
					if(dw_event->type == SIG_RX_ACK)
                    {
                        inst->wait4ack = 0 ; //clear the flag as the ACK has been received
						inst_processackmsg(inst, dw_event->msgu.rxackmsg.seqNum);
						//printf("RX ACK in TA_TX_WAIT_CONF... wait for TX confirm before changing state (%d)\n", inst->previousState);
                    }

                    inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT;
                        break;

                }

                inst->done = INST_NOT_DONE_YET;

                if((inst->previousState == TA_TXFINAL_WAIT_SEND) //tag will do immediate receive when waiting for report (as anchor sends it without delay)
                        && (inst->tag2rxReport == 0)) //anchor is not sending the report to tag
                {
                    inst->testAppState = TA_TXE_WAIT ;
                    inst->nextState = TA_TXPOLL_WAIT_SEND ;
                    break;
                }
                else if (inst->gotTO) //timeout
                {
					//printf("got TO in TA_TX_WAIT_CONF\n");
                    inst_processrxtimeout(inst);
                    inst->gotTO = 0;
					inst->wait4ack = 0 ; //clear this
					break;
                }
                else
                {
					inst->txu.txTimeStamp = dw_event->timeStamp;

                    inst->testAppState = TA_RXE_WAIT ;                      // After sending, tag expects response/report, anchor waits to receive a final/new poll
                    //fall into the next case (turn on the RX)
					message = 0;
                }

            }

            //break ; // end case TA_TX_WAIT_CONF


        case TA_RXE_WAIT :
        // printf("TA_RXE_WAIT") ;
        {

            if(inst->wait4ack == 0) //if this is set the RX will turn on automatically after TX
            {
                //turn RX on
				instancerxon(inst, 0, 0) ;   // turn RX on, with/without delay
            }
            else
            {
                inst->wait4ack = 0 ; //clear the flag, the next time we want to turn the RX on it might not be auto
            }

            if (inst->mode != LISTENER)
            {
                if (inst->previousState != TA_TXREPORT_WAIT_SEND) //we are going to use anchor timeout and re-send the report
                    inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT; //using RX FWTO
            }

            inst->testAppState = TA_RX_WAIT_DATA;   // let this state handle it

            // end case TA_RXE_WAIT, don't break, but fall through into the TA_RX_WAIT_DATA state to process it immediately.
            if(message == 0) break;
        }

        case TA_RX_WAIT_DATA :                                                                     // Wait RX data
		   //printf("TA_RX_WAIT_DATA %d", message) ;

            switch (message)
            {
                case SIG_RX_BLINK :
                {
					event_data_t* dw_event = instance_getevent(12); //get and clear this event
                    //printf("we got blink message from %08X\n", ( tagaddr& 0xFFFF));
                    if((inst->mode == LISTENER) || (inst->mode == ANCHOR))
                    {
						inst->canprintinfo = 1;

                        //add this Tag to the list of Tags we know about
						instaddtagtolist(inst, &(dw_event->msgu.rxblinkmsg.tagID[0]));

                        //initiate ranging message
                        if(inst->tagToRangeWith < TAG_LIST_SIZE)
                        {
                            //initiate ranging message this is a Blink from the Tag we would like to range to
							if(memcmp(&inst->tagList[inst->tagToRangeWith][0],  &(dw_event->msgu.rxblinkmsg.tagID[0]), BLINK_FRAME_SOURCE_ADDRESS) == 0)
                            {
                                inst->tagShortAdd = (dwt_getpartid() & 0xFF);
								inst->tagShortAdd =  (inst->tagShortAdd << 8) + dw_event->msgu.rxblinkmsg.tagID[0] ;

								if(inst->fixedReplyDelay_ms != FIXED_REPLY_DELAY)
                                {
									inst->delayedReplyTime = dw_event->timeStamp + convertmicrosectodevicetimeu(FIXED_LONG_BLINK_RESPONSE_DELAY * 1000.0) ;  // time we should send the blink response
                                }
                                else
                                {
									inst->delayedReplyTime = dw_event->timeStamp + inst->fixedReplyDelay ;  // time we should send the blink response
                                }
                                inst->delayedReplyTime &= MASK_40BIT ;

								//set destination address
								memcpy(&inst->rng_initmsg.destAddr[0], &(dw_event->msgu.rxblinkmsg.tagID[0]), BLINK_FRAME_SOURCE_ADDRESS); //remember who to send the reply to

                                inst->testAppState = TA_TXE_WAIT;
                                inst->nextState = TA_TXRANGINGINIT_WAIT_SEND ;

                                break;
                            }

                            //else stay in RX
                        }
                    }

                    //else //not initiating ranging - continue to receive
                    {
                        //only enable receiver when not using double buffering
                        if(inst->doublebufferon == 0)
                        {
                            inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
                        }
                        inst->done = INST_NOT_DONE_YET;
                    }

                }
                break;

				case SIG_RX_BLINKDW :
				{
					instance_getevent(13); //get and clear this event
					//event_data_t dw_event = instance_getevent(); //get and clear this event
					//instancelogrxblinkdata(inst, &dw_event);
					inst->done = INST_NOT_DONE_YET;
					//only enable receiver when not using double buffering
					if(inst->doublebufferon == 0)
					{
						inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
					}
					dwt_setrxaftertxdelay(0);

				}
				break;

                case SIG_RX_ACK :
                {
					event_data_t* dw_event = instance_getevent(14); //get and clear this event
					inst_processackmsg(inst, dw_event->msgu.rxackmsg.seqNum);
                    //else we did not expect this ACK turn the RX on again
                    //only enable receiver when not using double buffering
                    if(inst->doublebufferon == 0)
                    {
                        inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
                    }
                    inst->done = INST_NOT_DONE_YET;
                }
                break;

				//if we have received a DWT_SIG_RX_OKAY event - this means that the message is IEEE data type - need to check frame control to know which addressing mode is used
                case DWT_SIG_RX_OKAY :
                {
					event_data_t* dw_event = instance_getevent(15); //get and clear this event
					uint8_t  srcAddr[8] = {0,0,0,0,0,0,0,0};
					int non_user_payload_len = 0;
					int uplen = 0;
                    int fcode = 0;
					int fn_code = 0;
					int srclen = 0;
					int fctrladdr_len;
					uint8_t *messageData;

					inst->stoptimer = 0; //clear the flag, as we have received a message

                    // 16 or 64 bit addresses
					switch(dw_event->msgu.frame[1])
					{
						case 0xCC: //
							memcpy(&srcAddr[0], &(dw_event->msgu.rxmsg_ll.sourceAddr[0]), ADDR_BYTE_SIZE_L);
							fn_code = dw_event->msgu.rxmsg_ll.messageData[FCODE];
							messageData = &dw_event->msgu.rxmsg_ll.messageData[0];
							srclen = ADDR_BYTE_SIZE_L;
							fctrladdr_len = FRAME_CRTL_AND_ADDRESS_L;
							break;
						case 0xC8: //
							memcpy(&srcAddr[0], &(dw_event->msgu.rxmsg_sl.sourceAddr[0]), ADDR_BYTE_SIZE_L);
							fn_code = dw_event->msgu.rxmsg_sl.messageData[FCODE];
							messageData = &dw_event->msgu.rxmsg_sl.messageData[0];
							srclen = ADDR_BYTE_SIZE_L;
							fctrladdr_len = FRAME_CRTL_AND_ADDRESS_LS;
							break;
						case 0x8C: //
							memcpy(&srcAddr[0], &(dw_event->msgu.rxmsg_ls.sourceAddr[0]), ADDR_BYTE_SIZE_S);
							fn_code = dw_event->msgu.rxmsg_ls.messageData[FCODE];
							messageData = &dw_event->msgu.rxmsg_ls.messageData[0];
							srclen = ADDR_BYTE_SIZE_S;
							fctrladdr_len = FRAME_CRTL_AND_ADDRESS_LS;
							break;
						case 0x88: //
							memcpy(&srcAddr[0], &(dw_event->msgu.rxmsg_ss.sourceAddr[0]), ADDR_BYTE_SIZE_S);
							fn_code = dw_event->msgu.rxmsg_ss.messageData[FCODE];
							messageData = &dw_event->msgu.rxmsg_ss.messageData[0];
							srclen = ADDR_BYTE_SIZE_S;
							fctrladdr_len = FRAME_CRTL_AND_ADDRESS_S;
							break;
					}

					if((inst->ackexpected) && (inst->ackTO)) //ACK frame was expected but we got a good frame - treat as ACK timeout
                    {
						//printf("got good frame instead of ACK in DWT_SIG_RX_OKAY - pretend TO\n");
                        inst_processrxtimeout(inst);
                        message = 0; //clear the message as we have processed the event
                    }
                    else
                    {
						inst->ackexpected = 0; //clear this as we got good frame (but as not using ACK TO) we prob missed the ACK - check if it has been addressed to us

#if (DR_DISCOVERY == 1)
                        if(inst->mode == ANCHOR)
                        {
#if (USING_64BIT_ADDR==1)
							if(memcmp(&inst->tagList[inst->tagToRangeWith][0], &srcAddr[0], BLINK_FRAME_SOURCE_ADDRESS) == 0) //if the Tag's address does not match (ignore the message)
#else
							//if using 16-bit addresses the ranging messages from tag are using the short address tag was given in the ranging init message
							if(inst->tagShortAdd == (srcAddr[0] + (srcAddr[1] << 8)))
#endif
							//only process messages from the associated tag
                            {
								fcode = fn_code;
                            }
                        }
                        else // LISTENER or TAG
                        {
							fcode = fn_code;
                        }
#else
						//non - discovery mode - association is not used, process all messages
						fcode = fn_code;
#endif
                        switch(fcode)
                        {
                            case RTLS_DEMO_MSG_RNG_INIT:
                            {
								non_user_payload_len = RANGINGINIT_MSG_LEN;
                                if(inst->mode == TAG_TDOA) //only start ranging with someone if not ranging already
                                {
                                    inst->testAppState = TA_TXE_WAIT;
                                    inst->nextState = TA_TXPOLL_WAIT_SEND ; // send next poll

									inst->tagShortAdd = messageData[RES_R1] + (messageData[RES_R2] << 8) ;
									instancesetreplydelay(messageData[RES_T1] + (messageData[RES_T2] << 8), 0); //set the new timeouts/delay times for RX

#if (USING_64BIT_ADDR == 1)
									memcpy(&inst->msg.destAddr[0], &srcAddr[0], ADDR_BYTE_SIZE_L); //set the anchor address for the reply (set destination address)
#else
									memcpy(&inst->msg.destAddr[0], &srcAddr[0], ADDR_BYTE_SIZE_S); //set anchor address for the reply (set destination address)
									inst->msg.sourceAddr[0] =  messageData[RES_R1]; //set tag short address
									inst->msg.sourceAddr[1] =  messageData[RES_R2];
									dwt_setaddress16(inst->tagShortAdd);
#endif

                                    inst->mode = TAG ;
									//inst->responseTimeouts = 0; //reset timeout count
									inst->instToSleep = 0; //don't go to sleep - start ranging instead and then sleep after 1 range is done or poll times out
                                }
								//printf("GOT RTLS_DEMO_MSG_RNG_INIT - start ranging - \n");
								//else we ignore this message if already associated... (not TAG_TDOA)
                            }
							break; //RTLS_DEMO_MSG_RNG_INIT

                            case RTLS_DEMO_MSG_TAG_POLL:
                            {
								non_user_payload_len = TAG_POLL_MSG_LEN;
								if(inst->mode == LISTENER) //don't process any ranging messages when in Listener mode
								{
									//only enable receiver when not using double buffering
									if(inst->doublebufferon == 0)
									{
										inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
									}
									break;
								}

                                if (!inst->frameFilteringEnabled)
                                {
                                    // if we missed the ACK to the ranging init message we may not have turned frame filtering on
                                    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN); //we are starting ranging - enable the filter....
                                    inst->frameFilteringEnabled = 1 ;
                                }

								inst->tagPollRxTime = dw_event->timeStamp ; //Poll's Rx time

                                inst->delayedReplyTime = inst->tagPollRxTime + inst->fixedReplyDelay ;  // time we should send the response
                                inst->delayedReplyTime &= MASK_TXDTS ;

								//printf("PollRx Timestamp: %4.15e\n", convertdevicetimetosecu(dw_event.timeStamp));
                                //printf("Delay: %4.15e\n", convertdevicetimetosecu(inst->delayedReplyTime));
                                if(inst->doublebufferon == 1)
                                {
                                    dwt_forcetrxoff();
                                }
                                inst->testAppState = TA_TXRESPONSE_WAIT_SEND ; // send our response
								inst->canprintinfo = 0;

#if (USING_64BIT_ADDR == 1)
								memcpy(&inst->msg.destAddr[0], &srcAddr[0], ADDR_BYTE_SIZE_L); //remember who to send the reply to (set destination address)
#else
								memcpy(&inst->msg.destAddr[0], &srcAddr[0], ADDR_BYTE_SIZE_S); //remember who to send the reply to (set destination address)
#endif
                            }
                            break; //RTLS_DEMO_MSG_TAG_POLL

                            case RTLS_DEMO_MSG_ANCH_RESP:
                            {
								non_user_payload_len = ANCH_RESPONSE_MSG_LEN;
                                if(inst->mode == LISTENER) //don't process any ranging messages when in Listener mode
                                {
                                    //only enable receiver when not using double buffering
                                    if(inst->doublebufferon == 0)
                                    {
                                        inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
                                    }
                                    break;
                                }

								inst->tag2rxReport = messageData[RES_R2]; //check if the anchor is going to send the report
                                                                                 //if no report coming, go to sleep before sending the next poll

								inst->anchorRespRxTime = dw_event->timeStamp ; //Response's Rx time

								inst->delayedReplyTime = inst->anchorRespRxTime + inst->fixedReplyDelay ;  // time we should send the response
                                inst->delayedReplyTime &= MASK_TXDTS ;

								//printf("RespRx Timestamp: %4.15e\n", convertdevicetimetosecu(dw_event.timeStamp));
                                //printf("Delay: %4.15e\n", convertdevicetimetosecu(inst->delayedReplyTime));
                                if(inst->doublebufferon == 1)
                                {
                                    dwt_forcetrxoff();
                                }
                                inst->testAppState = TA_TXFINAL_WAIT_SEND ; // send our response / the final
#if (USING_64BIT_ADDR == 1)
								memcpy(&inst->relpyAddress[0], &srcAddr[0], ADDR_BYTE_SIZE_L); //remember who to send the reply to (set destination address)
#else
								memcpy(&inst->relpyAddress[0], &srcAddr[0], ADDR_BYTE_SIZE_S); //remember who to send the reply to (set destination address)
#endif
								inst->respPSC = (dwt_read16bitoffsetreg(0x10, 2) >> 4);
								inst->canprintinfo = 2;
                            }
                            break; //RTLS_DEMO_MSG_ANCH_RESP

                            case RTLS_DEMO_MSG_ANCH_TOFR:
                            {
								non_user_payload_len = TOF_REPORT_MSG_LEN;
                                if(inst->mode == LISTENER) //don't process any ranging messages when in Listener mode
                                {
                                    //only enable receiver when not using double buffering
                                    if(inst->doublebufferon == 0)
                                    {
                                        inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
                                    }
                                    break;
                                }

								memcpy(&inst->tof, &(messageData[TOFR]), 5);

								if(dw_event->msgu.frame[2] != inst->lastReportSN) //compare sequence numbers
                                {
                                    reportTOF(inst);
                                    inst->newrange = 1;
									inst->lastReportSN = dw_event->msgu.frame[2];
									//inst->lastReportTime = time_ms;
                                }

								//printf("ToFRx Timestamp: %4.15e\n", convertdevicetimetosecu(dw_event.timeStamp));

                                    inst->testAppState = TA_TXE_WAIT;
                                    inst->nextState = TA_TXPOLL_WAIT_SEND ; // send next poll
                            }
                            break; //RTLS_DEMO_MSG_ANCH_TOFR

                            case RTLS_DEMO_MSG_TAG_FINAL:
                            {
                                uint64_t tRxT, tTxT, aRxT, aTxT ;
                                uint64_t tagFinalTxTime  = 0;
                                uint64_t tagFinalRxTime  = 0;
                                uint64_t tagPollTxTime  = 0;
                                uint64_t anchorRespRxTime  = 0;
                                uint64_t pollRespRTD  = 0;
                                uint64_t respFinalRTD  = 0;

								non_user_payload_len = TAG_FINAL_MSG_LEN;
                                if(inst->mode == LISTENER) //don't process any ranging messages when in Listener mode
                                {
                                    //only enable receiver when not using double buffering
                                    if(inst->doublebufferon == 0)
                                    {
                                        inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
                                    }
                                    break;
                                }

                                // time of arrival of Final message
								tagFinalRxTime = dw_event->timeStamp ; //Final's Rx time

								//printf("FinalRx Timestamp: %4.15e\n", convertdevicetimetosecu(dw_event.timeStamp));
                                inst->delayedReplyTime = 0 ;

                                // times measured at Tag extracted from the message buffer
                                // extract 40bit times
								memcpy(&tagPollTxTime, &(messageData[PTXT]), 5);
								memcpy(&anchorRespRxTime, &(messageData[RRXT]), 5);
								memcpy(&tagFinalTxTime, &(messageData[FTXT]), 5);

                                // poll response round trip delay time is calculated as
                                // (anchorRespRxTime - tagPollTxTime) - (anchorRespTxTime - tagPollRxTime)
                                aRxT = (anchorRespRxTime - tagPollTxTime) & MASK_40BIT;
                                aTxT = (inst->txu.anchorRespTxTime - inst->tagPollRxTime) & MASK_40BIT;
                                pollRespRTD = (aRxT - aTxT) & MASK_40BIT;


                                // response final round trip delay time is calculated as
                                // (tagFinalRxTime - anchorRespTxTime) - (tagFinalTxTime - anchorRespRxTime)
                                tRxT = (tagFinalRxTime - inst->txu.anchorRespTxTime) & MASK_40BIT;
                                tTxT = (tagFinalTxTime - anchorRespRxTime) & MASK_40BIT;
                                respFinalRTD = (tRxT - tTxT) & MASK_40BIT;

                                // add both round trip delay times
                                inst->tof = ((pollRespRTD + respFinalRTD) & MASK_40BIT);

								/*{ // work out clock offset
                                    double time = (inst->fixedReplyDelay_ms/1000.0); //convert to seconds
                                    double rtd1, rtd2, aveRTD, y;
                                    rtd1 = convertdevicetimetosec8((uint8_t*) &respFinalRTD) ;
                                    rtd2 = convertdevicetimetosec8((uint8_t*) &pollRespRTD) ;
                                    aveRTD = (rtd1 + rtd2) / 2.0 ; //average
                                    y = rtd1 - aveRTD ;
                                    inst->clockOffset = y / time ;
                                    inst->clockOffset *= 1e6 ; //in parts per million
								}*/
                                reportTOF(inst);
                                inst->newrange = 1;
								//inst->lastReportTime = time_ms;


								if(inst->sendTOFR2Tag == SEND_TOF_REPORT)
                                {
                                    #if (DOUBLE_RX_BUFFER == 1)
                                        dwt_forcetrxoff(); //disable rx
                                    #endif
                                    inst->testAppState = TA_TXREPORT_WAIT_SEND ; // send the report with the calculated time of flight
                                    inst->newReportSent = 0; //set the new report flag
                                }
                                else
                                {
                                    //only enable receiver when not using double buffering
                                    if(inst->doublebufferon == 0)
                                    {
                                    inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
                                    }
									dwt_setrxaftertxdelay(0);
                                }

                            }
                            break; //RTLS_DEMO_MSG_TAG_FINAL


                            default:
                            {
                                //only enable receiver when not using double buffering
                                if(inst->doublebufferon == 0)
                                {
                                    inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
                                }
								dwt_setrxaftertxdelay(0);

                            }
                            break;
						} //end switch (fcode)

						if(dw_event->msgu.frame[0] & 0x20)
						{
							//as we only pass the received frame with the ACK request bit set after the ACK has been sent
							instance_getevent(16); //get and clear the ACK sent event
						}
					} //end else

                        if((inst->instToSleep == 0) && (inst->mode == LISTENER) /*|| (inst->mode == ANCHOR)*/)//update received data, and go back to receiving frames
                        {

						//only enable receiver when not using double buffering
						if(inst->doublebufferon == 0)
						{
                            inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
                        }
						dwt_setrxaftertxdelay(0);
                    }


					uplen = dw_event->rxLength - (non_user_payload_len + fctrladdr_len + FRAME_CRC) ;
					// retrieve USER PAYLOAD into a status line report
					/*if(uplen)
					{
						messageData[uplen+non_user_payload_len] = 0 ;                // this terminates the string (possibly overwriting the first CRC byte) ;
						instancelogrxpayload(&inst->statusrep, &srcAddr[0], srclen, dw_event.rxLength, uplen, &messageData[non_user_payload_len]);
					}*/

                }
				break ; //end of DWT_SIG_RX_OKAY

                case DWT_SIG_RX_TIMEOUT :
					instance_getevent(17); //get and clear this event
					//printf("PD_DATA_TIMEOUT %d\n", inst->previousState) ;
                    inst_processrxtimeout(inst);
                    message = 0; //clear the message as we have processed the event
                break ;

                case DWT_SIG_TX_AA_DONE: //ignore this event - just process the rx frame that was received before the ACK response
				case 0:
				default :
                {
                    //if(DWT_SIG_TX_AA_DONE == message) printf("Got SIG_TX_AA_DONE in RX wait - ignore\n");
                    if(inst->done == INST_NOT_DONE_YET) inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT;
                }
                break;

            }
            break ; // end case TA_RX_WAIT_DATA
            default:
                //printf("\nERROR - invalid state %d - what is going on??\n", inst->testAppState) ;
            break;
    } // end switch on testAppState

    return inst->done;
} // end testapprun()

// -------------------------------------------------------------------------------------------------------------------
#if NUM_INST != 1
#error These functions assume one instance only
#else


// -------------------------------------------------------------------------------------------------------------------
// function to initialise instance structures
//
// Returns 0 on success and -1 on error
int instance_init_s(int mode)
{
    int instance = 0 ;

    instance_data[instance].mode =  mode;                                // assume anchor,
    instance_data[instance].testAppState = TA_INIT ;

    // if using auto CRC check (DWT_INT_RFCG and DWT_INT_RFCE) are used instead of DWT_INT_RDFR flag
    // other errors which need to be checked (as they disable receiver) are
    //dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_SFDT | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);

    dwt_setcallbacks(instance_txcallback, instance_rxcallback);

#if (DR_DISCOVERY == 1)
    instance_data[instance].sendTOFR2Tag = 1;
#else
    instance_data[instance].sendTOFR2Tag = 0;
#endif

    instance_data[instance].tag2rxReport = 0; //Tag is not expecting the report


    instance_setapprun(testapprun_s);

    instance_data[instance].anchorListIndex = 0 ;
    instance_data[instance].doublebufferon = DOUBLE_RX_BUFFER;

    //sample test calibration functions
    //xtalcalibration();
    //powertest();

    return 0 ;
}




// -------------------------------------------------------------------------------------------------------------------
// function to set the fixed blink reply delay time (in ms)
// NOTE: this is the delay between receiving the blink and sending the ranging init message
// The anchor ranging init response delay has to match the delay the tag expects
// thereafter the tag will use ranging response delay as specified in the ranging init message
void instancesetblinkreplydelay(int delayms) //delay in ms
{
#if (USING_64BIT_ADDR == 1)
	int msgdatalen = RANGINGINIT_MSG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC;
#else
	int msgdatalen = RANGINGINIT_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC;
#endif

	//msgdatalen = BLINK_FRAME_CRTL_AND_ADDRESS + BLINK_FRAME_CRC;

    instancesetreplydelay(delayms, msgdatalen) ;
}

// -------------------------------------------------------------------------------------------------------------------
// function to set the fixed reply delay time (in ms)
//
//extern uint8_t dwnsSFDlen[];

void instancesetreplydelay(int delayms, int datalength) //delay in ms
{
    int instance = 0;

    int margin = 3000; //2000 symbols

	//configure the rx delay receive delay time, it is dependent on the message length
	float msgdatalen = 0;
	float preamblelen = 0;
	int sfdlen = 0;
	int x = 0;

	if(datalength == 0)
        {
#if (USING_64BIT_ADDR == 1)
		msgdatalen = TAG_FINAL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC;
#else
		msgdatalen = TAG_FINAL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC;
#endif

		//msgdatalen += instance_data[instance].payload.payloadLen ;
        }
	else //blink message
        {
		msgdatalen = datalength;
    }

	x = (int) ceil(msgdatalen*8/330.0f);

	msgdatalen = msgdatalen*8 + x*48;

	//assume PHR length is 172308us for 110k and 21539us for 850k/6.81M
	if(instance_data[instance].configData.dataRate == DWT_BR_110K)
    {
		msgdatalen *= 8205.13f;
		msgdatalen += 172308;

                }
	else if(instance_data[instance].configData.dataRate == DWT_BR_850K)
                {
		msgdatalen *= 1025.64f;
		msgdatalen += 21539;
        }
	else
        {
		msgdatalen *= 128.21f;
		msgdatalen += 21539;
	}

	//SFD length is 64 for 110k (always)
	//SFD length is 8 for 6.81M, and 16 for 850k, but can vary between 8 and 16 bytes
	sfdlen = dwnsSFDlen[instance_data[instance].configData.dataRate];

	switch (instance_data[instance].configData.txPreambLength)
    {
    case DWT_PLEN_4096 : preamblelen = 4096.0f; break;
    case DWT_PLEN_2048 : preamblelen = 2048.0f; break;
    case DWT_PLEN_1536 : preamblelen = 1536.0f; break;
    case DWT_PLEN_1024 : preamblelen = 1024.0f; break;
    case DWT_PLEN_512  : preamblelen = 512.0f; break;
    case DWT_PLEN_256  : preamblelen = 256.0f; break;
    case DWT_PLEN_128  : preamblelen = 128.0f; break;
    case DWT_PLEN_64   : preamblelen = 64.0f; break;
            }

	//preamble  = plen * (994 or 1018) depending on 16 or 64 PRF
	if(instance_data[instance].configData.prf == DWT_PRF_16M)
	{
		preamblelen = (sfdlen + preamblelen) * 0.99359f;
        }
        else
        {
		preamblelen = (sfdlen + preamblelen) * 1.01763f;
        }


	if(datalength != 0)
        {
		//set the frame wait timeout time - total time the frame takes in symbols
		instance_data[instance].fwtoTimeB_sy = 16 + (int)((preamblelen + (msgdatalen/1000.0))/ 1.0256) + margin;

		instance_data[instance].rnginitW4Rdelay_sy = (int) (delayms * 1000 / 1.0256) - 16 - (int)((preamblelen + (msgdatalen/1000.0))/ 1.0256);
        }
        else
        {
		//set the frame wait timeout time - total time the frame takes in symbols
		instance_data[instance].fwtoTime_sy = 16 + (int)((preamblelen + (msgdatalen/1000.0))/ 1.0256) + margin;

		//this is the delay used for the delayed transmit (when sending the ranging init, response, and final messages)
		instance_data[instance].fixedReplyDelay = convertmicrosectodevicetimeu (delayms * 1e3) ;

		instance_data[instance].fixedReplyDelay_ms = (int) delayms ;

		//this it the delay used for configuring the receiver on delay (wait for response delay),
		instance_data[instance].fixedReplyDelay_sy = (int) (delayms * 1000 / 1.0256) - 16 - (int)((preamblelen + (msgdatalen/1000.0))/ 1.0256); //subtract 16 symbols, as receiver has a 16 symbol start up time
            }
	//printf("preamble %4.3fus, Final msg %4.3fus\n", preamblelen, msgdatalen/1000);
	//printf("Set response delay time to %d ms, %d sym payload %d\n", (int) delayms, instance_data[instance].fixedReplyDelay_sy, instance_data[instance].payload.payloadLen);
            }

// -------------------------------------------------------------------------------------------------------------------
// function to configure anchor instance whether to send TOF reports to Tag
//
void instancesetreporting(int anchorSendsTofReports)
        {
    int instance = 0 ;
    instance_data[instance].sendTOFR2Tag = anchorSendsTofReports ;        // Set whether TOF reports are sent
    }

#if (DR_DISCOVERY == 0)
// -------------------------------------------------------------------------------------------------------------------
//
// Set Payload parameters for the instance
//
// -------------------------------------------------------------------------------------------------------------------
void instancesetaddresses(instanceAddressConfig_t *plconfig)
    {
    int instance = 0 ;

    instance_data[instance].payload = *plconfig ;       // copy configurations

    instancesetreporting(instance_data[instance].payload.sendReport);

}
#endif

uint64_t instance_get_addr(void) //get own address
{
    int instance = 0;
    uint64_t x = (uint64_t) instance_data[instance].eui64[0];
    x |= (uint64_t) instance_data[instance].eui64[1] << 8;
    x |= (uint64_t) instance_data[instance].eui64[2] << 16;
    x |= (uint64_t) instance_data[instance].eui64[3] << 24;
    x |= (uint64_t) instance_data[instance].eui64[4] << 32;
    x |= (uint64_t) instance_data[instance].eui64[5] << 40;
    x |= (uint64_t) instance_data[instance].eui64[6] << 48;
    x |= (uint64_t) instance_data[instance].eui64[7] << 56;


    return (x);
}

uint64_t instance_get_tagaddr(void) //get own address
{
    int instance = 0;
    uint64_t x = (uint64_t) instance_data[instance].tagList[0][0];
    x |= (uint64_t) instance_data[instance].tagList[0][1] << 8;
    x |= (uint64_t) instance_data[instance].tagList[0][2] << 16;
    x |= (uint64_t) instance_data[instance].tagList[0][3] << 24;
    x |= (uint64_t) instance_data[instance].tagList[0][4] << 32;
    x |= (uint64_t) instance_data[instance].tagList[0][5] << 40;
    x |= (uint64_t) instance_data[instance].tagList[0][6] << 48;
    x |= (uint64_t) instance_data[instance].tagList[0][7] << 56;


    return (x);
}

uint64_t instance_get_anchaddr(void) //get anchor address (that sent the ToF)
{
    int instance = 0;
    uint64_t x = (uint64_t) instance_data[instance].relpyAddress[0];
    x |= (uint64_t) instance_data[instance].relpyAddress[1] << 8;
    x |= (uint64_t) instance_data[instance].relpyAddress[2] << 16;
    x |= (uint64_t) instance_data[instance].relpyAddress[3] << 24;
    x |= (uint64_t) instance_data[instance].relpyAddress[4] << 32;
    x |= (uint64_t) instance_data[instance].relpyAddress[5] << 40;
    x |= (uint64_t) instance_data[instance].relpyAddress[6] << 48;
    x |= (uint64_t) instance_data[instance].relpyAddress[7] << 56;
    return (x);
}

void instance_readaccumulatordata(void)
{
#if DECA_SUPPORT_SOUNDING==1
    int instance = 0;
    uint16_t len = 992 ; //default (16M prf)

    if (instance_data[instance].configData.prf == DWT_PRF_64M)  // Figure out length to read
        len = 1016 ;

    instance_data[instance].buff.accumLength = len ;                                       // remember Length, then read the accumulator data

    len = len*4+1 ;   // extra 1 as first byte is dummy due to internal memory access delay

    dwt_readaccdata((uint8_t*)&(instance_data[instance].buff.accumData->dummy), len, 0);
#endif  // support_sounding
}

#endif


/* ==========================================================

Notes:

Previously code handled multiple instances in a single console application

Now have changed it to do a single instance only. With minimal code changes...(i.e. kept [instance] index but it is always 0.

Windows application should call instance_init() once and then in the "main loop" call instance_run().

*/
