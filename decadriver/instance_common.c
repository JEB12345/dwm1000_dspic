/*! ----------------------------------------------------------------------------
 *  @file    instance_common.c
 *  @brief   DecaWave application level common instance functions
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include "deca_device_api.h"
#include "instance.h"
#include <stdlib.h>
#include <math.h>
#include <xc.h>

// -------------------------------------------------------------------------------------------------------------------
//      Data Definitions
// -------------------------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------------------------

double inst_idist = 0;
double inst_adist = 0;
double inst_ldist = 0;
extern instance_data_t instance_data[NUM_INST] ;

instance_localdata_t instance_localdata[NUM_INST] ;

//int eventOutcount = 0;
//int eventIncount = 0;

// -------------------------------------------------------------------------------------------------------------------
// Functions
// -------------------------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------------------------
// convert microseconds to device time
uint64_t convertmicrosectodevicetimeu (double microsecu)
{
    uint64_t dt;
    long double dtime;

    dtime = (microsecu / (double) DWT_TIME_UNITS) / 1e6 ;

    dt =  (uint64_t) (dtime) ;

    return dt;
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


double convertdevicetimetosec(int32_t dt)
{
    double f = 0;

    f =  dt * DWT_TIME_UNITS ;  // seconds #define TIME_UNITS          (1.0/499.2e6/128.0) = 15.65e-12

    return f ;
}

double convertdevicetimetosec8(uint8_t* dt)
{
    double f = 0;

    uint32_t lo = 0;
    int8_t hi = 0;

    memcpy(&lo, dt, 4);
    hi = dt[4] ;

    f = ((hi * 65536.00 * 65536.00) + lo) * DWT_TIME_UNITS ;  // seconds #define TIME_UNITS          (1.0/499.2e6/128.0) = 15.65e-12

    return f ;
}

void reportTOF_f(instance_data_t *inst)
{
        double distance ;
        double tof ;
        int32_t tofi ;

        // check for negative results and accept them making them proper negative integers
        tofi = (int32_t) inst->tof32 ;                          // make it signed
        if (tofi < 0)                          // close up TOF may be negative
        {
            tofi *= -1 ;                       // make it positive
        }

        // convert to seconds (as floating point)
        tof = convertdevicetimetosec(tofi) * 0.25;          //this is divided by 4 to get single time of flight
        distance = tof * SPEED_OF_LIGHT;

        if ((distance < -0.5) || (distance > 20000.000))    // discard any results less than <50 cm or >20km
            return;

#if (CORRECT_RANGE_BIAS == 1)
        distance = distance - dwt_getrangebias(inst->configData.chan, (float) distance, inst->configData.prf);
#endif

        distance = fabs(distance) ;                         // make any (small) negatives positive.

        inst_idist = distance;

        inst->longTermRangeCount++ ;

        inst->adist[inst->tofindex++] = distance;

        if(inst->tofindex == RTD_MED_SZ) inst->tofindex = 0;

        if(inst->tofcount == RTD_MED_SZ)
        {
            int i;
            double sumdiff, avg;

            avg = 0;
            sumdiff = 0;
            for(i = 0; i < inst->tofcount; i++)
            {
                avg += inst->adist[i];
            }
            avg /= inst->tofcount;

            inst_adist = avg ;

        }
        else
            inst->tofcount++;
    return ;
}// end of reportTOF_f


void reportTOF(instance_data_t *inst)
{
        double distance ;
        double tof ;
        double ltave;
        int64_t tofi ;

        // check for negative results and accept them making them proper negative integers
        tofi = (int64_t) inst->tof ;                          // make it signed
        if (tofi > 0x007FFFFFFFFF)                          // MP counter is 40 bits,  close up TOF may be negative
        {
            tofi -= 0x010000000000 ;                       // subtract fill 40 bit range to mak it negative
        }

        // convert to seconds (as floating point)
        tof = convertdevicetimetosec(tofi) * 0.25;          //this is divided by 4 to get single time of flight
        distance = tof * SPEED_OF_LIGHT;

        if ((distance < -0.5) || (distance > 20000.000))    // discard any results less than <50 cm or >20km
            return;

#if (CORRECT_RANGE_BIAS == 1)
        distance = distance - dwt_getrangebias(inst->configData.chan, (float) distance, inst->configData.prf);
#endif

        distance = fabs(distance) ;                         // make any (small) negatives positive.

        inst_idist = distance;

        inst->longTermRangeSum+= distance ;
        inst->longTermRangeCount++ ;                          // for computing a long term average
        ltave = inst->longTermRangeSum / inst->longTermRangeCount ;

        inst_ldist = ltave ;

        inst->adist[inst->tofindex++] = distance;

        if(distance < inst->idistmin)
            inst->idistmin = distance;

        if(distance > inst->idistmax)
            inst->idistmax = distance;

        if(inst->tofindex == RTD_MED_SZ) inst->tofindex = 0;

        if(inst->tofcount == RTD_MED_SZ)
        {
            int i;
            double sumdiff, avg;

            avg = 0;
            sumdiff = 0;
            for(i = 0; i < inst->tofcount; i++)
            {
                avg += inst->adist[i];
            }
            avg /= inst->tofcount;

            inst_adist = avg ;

        }
        else
            inst->tofcount++;
    return ;
}// end of reportTOF

// -------------------------------------------------------------------------------------------------------------------
//
// function to select the destination address (e.g. the address of the next anchor to poll)
//
// -------------------------------------------------------------------------------------------------------------------
//
int instaddtagtolist(instance_data_t *inst, uint8_t *tagAddr)
{
    uint8_t i;
    uint8_t blank[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    inst->blinkRXcount++ ;

    //add the new Tag to the list, if not already there and there is space
    for(i=0; i<TAG_LIST_SIZE; i++)
    {
        if(memcmp(&inst->tagList[i][0], &tagAddr[0], 8) != 0)
        {
            if(memcmp(&inst->tagList[i][0], &blank[0], 8) == 0) //blank entry
            {
                memcpy(&inst->tagList[i][0], &tagAddr[0], 8) ;
                inst->tagListLen = i + 1 ;
                break;
            }
        }
        else
        {
            break; //we already have this Tag in the list
        }
    }

    return 0;
}


// -------------------------------------------------------------------------------------------------------------------
#if NUM_INST != 1
#error These functions assume one instance only
#else

void instcleartaglist(void)
{
    int instance = 0 ;

    memset((uint8_t *) &instance_data[instance].tagList[0][0], 0, 8 * TAG_LIST_SIZE * sizeof(instance_data[instance].tagList[0][0]));
    instance_data[instance].tagListLen = 0 ;
    instance_data[instance].blinkRXcount = 0 ;
    instance_data[instance].tagToRangeWith = TAG_LIST_SIZE;
}


uint32_t instgetblinkrxcount(void)
{
    int instance = 0 ;
    return instance_data[instance].blinkRXcount ;
}




// -------------------------------------------------------------------------------------------------------------------
// Set this instance role as the Tag, Anchor or Listener
void instancesetrole(int inst_mode)
{
    // assume instance 0, for this
    instance_data[0].mode =  inst_mode;                   // set the role
}

int instancegetrole(void)
{
    return instance_data[0].mode;
}

int instancenewrange(void)
{
    if(instance_data[0].newrange)
    {
        instance_data[0].newrange = 0;
        return 1;
    }

    return 0;
}

int instanceanchorwaiting(void)
{
	return instance_data[0].canprintinfo;
}

int instancesleeping(void)
{
	if(instance_data[0].canprintinfo == 1)
	{
		instance_data[0].canprintinfo = 0; //clear flag
		return 1;
	}

	return 0 ;
}
// -------------------------------------------------------------------------------------------------------------------
// function to clear counts/averages/range values
//
void instanceclearcounts(void)
{
    int instance = 0 ;

    instance_data[instance].rxTimeouts = 0 ;

    instance_data[instance].frame_sn = 0;

    dwt_configeventcounters(1); //enable and clear - NOTE: the counters are not preserved when in DEEP SLEEP

    instance_data[instance].frame_sn = 0;
    instance_data[instance].lastReportSN = 0xff;

    instance_data[instance].tofcount = 0 ;
    instance_data[instance].tofindex = 0 ;

    instance_data[instance].txmsgcount = 0;
    instance_data[instance].rxmsgcount = 0;
    instance_data[instance].lateTX = 0;
    instance_data[instance].lateRX = 0;

    instance_data[instance].longTermRangeSum  = 0;
    instance_data[instance].longTermRangeCount  = 0;

    instance_data[instance].idistmax = 0;
    instance_data[instance].idistmin = 1000;

} // end instanceclearcounts()


// -------------------------------------------------------------------------------------------------------------------
// function to initialise instance structures
//
// Returns 0 on success and -1 on error
int instance_init(void)
{
    int instance = 0 ;
    int result;
    //uint16_t temp = 0;

    instance_data[instance].mode =  ANCHOR;                                // assume listener,

    instance_data[instance].instToSleep = 0;

    instance_data[instance].sentSN = 0;

    instance_data[instance].tofindex = 0;
    instance_data[instance].tofcount = 0;

    // Reset the IC (might be needed if not getting here from POWER ON)
    // ARM code: Remove soft reset here as using hard reset in the inittestapplication() in the main.c file
    //dwt_softreset();

#if (DEEP_SLEEP_AUTOWAKEUP == 1)
	uint16_t blinktime  = 0xf; //e.g. blink time to be used for the Tag auto wake up
    {
        double t;
		uint16_t lp_osc_cal = dwt_calibratesleepcnt(); //calibrate low power oscillator
        //the lp_osc_cal value is number of XTAL/2 cycles in one cycle of LP OSC
        //to convert into seconds (38.4MHz/2 = 19.2MHz (XTAL/2) => 1/19.2MHz ns)
        //so to get a sleep time of 5s we need to program 5 / period and then >> 12 as the register holds upper 16-bits of 28-bit counter
		t = ((double)5/((double)lp_osc_cal/19.2e6));
		blinktime = (int) t;
		blinktime >>= 12;

		dwt_configuresleepcnt(blinktime);//configure sleep time

    }
#endif

	//we can enable any configuration loding from OTP/ROM on initialisation
    result = dwt_initialise(DWT_LOADUCODE | DWT_LOADLDO | DWT_LOADTXCONFIG | DWT_LOADANTDLY| DWT_LOADXTALTRIM) ;
//        result = dwt_initialise(DWT_LOADLDOTUNE & DWT_LOADUCODE);

    //temp = dwt_readtempvbat();
	//  temp formula is: 1.13 * reading - 113.0
	// Temperature (°C )= (SAR_LTEMP - (OTP_READ(Vtemp @ 23 °C )) x 1.14) + 23
	//  volt formula is: 0.0057 * reading + 2.3
	// Voltage (volts) = (SAR_LVBAT- (OTP_READ(Vmeas @ 3.3 V )) /173) + 3.3
	//printf("Vbat = %d (0x%02x) %1.2fV\tVtemp = %d  (0x%02x) %2.2fC\n",temp&0xFF,temp&0xFF, ((0.0057 * (temp&0xFF)) + 2.3), (temp>>8)&0xFF,(temp>>8)&0xFF, ((1.13 * ((temp>>8)&0xFF)) - 113.0));

    //this is platform dependant - only program if DW EVK/EVB
    dwt_setleds(3) ; //configure the GPIOs which control the leds on EVBs

    if (DWT_SUCCESS != result)
    {
        return (-1) ;   // device initialise has failed
    }


    instanceclearcounts() ;

    instance_data[instance].panid = 0xdeca ;

    instance_data[instance].newReportSent = 0; //clear the flag
    instance_data[instance].wait4ack = 0;
    instance_data[instance].ackexpected = 0;
    instance_data[instance].stoptimer = 0;
    instance_data[instance].instancetimer_en = 0;

    instance_clearevents();

    instance_data[instance].rxautoreenable = 0;

    dwt_geteui(instance_data[instance].eui64);

    instance_localdata[instance].testapprun_fn = NULL;
    instance_data[instance].canprintinfo = 0;
    return 0 ;
}

// -------------------------------------------------------------------------------------------------------------------
//
// Return the Device ID register value, enables higher level validation of physical device presence
//

uint32_t instancereaddeviceid(void)
{
    return dwt_readdevid() ;
}


// -------------------------------------------------------------------------------------------------------------------
//
// function to allow application configuration be passed into instance and affect underlying device opetation
//
void instance_config(instanceConfig_t *config)
{
    int instance = 0 ;
    int use_otpdata = DWT_LOADANTDLY | DWT_LOADXTALTRIM;
    uint32_t power = 0;

    instance_data[instance].configData.chan = config->channelNumber ;
    instance_data[instance].configData.rxCode =  config->preambleCode ;
    instance_data[instance].configData.txCode = config->preambleCode ;
    instance_data[instance].configData.prf = config->pulseRepFreq ;
    instance_data[instance].configData.dataRate = config->dataRate ;
    instance_data[instance].configData.txPreambLength = config->preambleLen ;
    instance_data[instance].configData.rxPAC = config->pacSize ;
    instance_data[instance].configData.nsSFD = config->nsSFD ;
    instance_data[instance].configData.phrMode = DWT_PHRMODE_STD ;
    instance_data[instance].configData.sfdTO = config->sfdTO;

    //enable gating gain for 6.81Mbps data rate
    if(instance_data[instance].configData.dataRate == DWT_BR_6M8)
        instance_data[instance].configData.smartPowerEn = 1;
    else
        instance_data[instance].configData.smartPowerEn = 0;

    //configure the channel parameters
    dwt_configure(&instance_data[instance].configData, use_otpdata) ;

    instance_data[instance].configTX.PGdly = txSpectrumConfig[config->channelNumber].PGdelay ;

    //firstly check if there are calibrated TX power value in the DW1000 OTP
    power = dwt_getotptxpower(config->pulseRepFreq, instance_data[instance].configData.chan);

    if((power == 0x0) || (power == 0xFFFFFFFF)) //if there are no calibrated values... need to use defaults
    {
        power = txSpectrumConfig[config->channelNumber].txPwr[config->pulseRepFreq- DWT_PRF_16M];
    }

    //Configure TX power
    //if smart power is used then the value as read from OTP is used directly
    //if smart power is used the user needs to make sure to transmit only one frame per 1ms or TX spectrum power will be violated
    if(instance_data[instance].configData.smartPowerEn == 1)
    {
        instance_data[instance].configTX.power = power;
    }
	else //if the smart power is not used, then the low byte value (repeated) is used for the whole TX power register
    {
        uint8_t pow = power & 0xFF ;
        instance_data[instance].configTX.power = (pow | (pow << 8) | (pow << 16) | (pow << 24));
    }
    dwt_setsmarttxpower(instance_data[instance].configData.smartPowerEn);

    //configure the tx spectrum parameters (power and PG delay)
    dwt_configuretxrf(&instance_data[instance].configTX);

    //check if to use the antenna delay calibration values as read from the OTP
    if((use_otpdata & DWT_LOADANTDLY) == 0)
    {
        instance_data[instance].txantennaDelay = rfDelays[config->pulseRepFreq - DWT_PRF_16M];
        // -------------------------------------------------------------------------------------------------------------------
        // set the antenna delay, we assume that the RX is the same as TX.
        dwt_setrxantennadelay(instance_data[instance].txantennaDelay);
        dwt_settxantennadelay(instance_data[instance].txantennaDelay);
    }
    else
    {
        //get the antenna delay that was read from the OTP calibration area
        instance_data[instance].txantennaDelay = dwt_readantennadelay(config->pulseRepFreq) >> 1;

        // if nothing was actually programmed then set a reasonable value anyway
        if (instance_data[instance].txantennaDelay == 0)
        {
            instance_data[instance].txantennaDelay = rfDelays[config->pulseRepFreq - DWT_PRF_16M];
            // -------------------------------------------------------------------------------------------------------------------
            // set the antenna delay, we assume that the RX is the same as TX.
            dwt_setrxantennadelay(instance_data[instance].txantennaDelay);
            dwt_settxantennadelay(instance_data[instance].txantennaDelay);
        }


    }

    if(config->preambleLen == DWT_PLEN_64) //if preamble length is 64
	{
    	config_spi2_slow(); //reduce SPI to < 3MHz

	dwt_loadopsettabfromotp(0);

	config_spi2_fast(); //increase SPI to max
    }

	//config is needed before reply delays are configured
    //this is done in main.c in inittestapplication() function
	//instancesetreplydelay(FIXED_REPLY_DELAY);

}

// -------------------------------------------------------------------------------------------------------------------
// function to set the tag sleep time (in ms)
//
void instancesettagsleepdelay(int sleepdelay, int blinksleepdelay) //sleep in ms
{
    int instance = 0 ;
    instance_data[instance].tagSleepTime_ms = sleepdelay ;
    instance_data[instance].tagBlinkSleepTime_ms = blinksleepdelay ;
}



int instance_get_dly(void) //get antenna delay
{
    int x = instance_data[0].txantennaDelay;

    return (x);
}


// -------------------------------------------------------------------------------------------------------------------
double instance_get_ldist(void) //get long term average range
{
    double x = inst_ldist;

    return (x);
}

int instance_get_lcount(void) //get count of ranges used for calculation of lt avg
{
    int x = instance_data[0].longTermRangeCount;

    return (x);
}

double instance_get_idist(void) //get instantaneous range
{
    double x = inst_idist;

    return (x);
}

int instance_get_rxf(void) //get number of Rxed frames
{
    int x = instance_data[0].rxmsgcount;

    return (x);
}

int instance_get_txf(void) //get number of Txed frames
{
    int x = instance_data[0].txmsgcount;

    return (x);
}

int instance_get_txl(void) //get number of late Tx frames
{
    int x = instance_data[0].lateTX;

    return (x);
}

int instance_get_rxl(void) //get number of late Tx frames
{
    int x = instance_data[0].lateRX;

    return (x);
}

double instance_get_adist(void) //get average range
{
    double x = inst_adist;

    return (x);
}

int instance_get_respPSC(void)
{
	int x = instance_data[0].respPSC;

	instance_data[0].respPSC = 0;

	return x;
}



void inst_processrxtimeout(instance_data_t *inst)
{

	//inst->responseTimeouts ++ ;
    inst->rxTimeouts ++ ;
    inst->done = INST_NOT_DONE_YET;

    if(inst->mode == ANCHOR) //we did not receive the final - wait for next poll
    {
		if((inst->newReportSent) && (inst->newReportSent < MAX_NUMBER_OF_REPORT_RETRYS)) //no ACK send another report
		{
			inst->testAppState = TA_TXREPORT_WAIT_SEND ;
			//printf("Timeout while waiting for ACK -> send next report\n");
		}
		else //finished sending reports - wait for next poll message (can also get here if no ACK to a Ranging Init message, just wait for a Poll or next Blink)
		{
			//only enable receiver when not using double buffering
			inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
			dwt_setrxtimeout(0);
            inst->ackexpected = 0 ; //timeout... so no acks expected anymore
		}
    }
	else //if(inst->mode == TAG)
    {
		// initiate the re-transmission of the poll that was not responded to
		inst->testAppState = TA_TXE_WAIT ;
//NOTE: Do not go back to "discovery" mode -  needs a boards reset to go back to "discovery" mode once is starts ranging
#if 0
#if (DR_DISCOVERY == 1)
		if((inst->mode == TAG) && (inst->responseTimeouts >= MAX_NUMBER_OF_POLL_RETRYS)) //if no response after sending 20 Polls - go back to blink mode
		{
			inst->mode = TAG_TDOA ;
		}
#endif
#endif
		if(inst->mode == TAG)
		{
			inst->nextState = TA_TXPOLL_WAIT_SEND ;
		}
#if (DR_DISCOVERY == 1)
		else //TAG_TDOA
		{
			inst->nextState = TA_TXBLINK_WAIT_SEND ;
		}
#endif
    }

    //timeout - disable the radio (if using SW timeout the rx will not be off)
    dwt_forcetrxoff() ;
}


void inst_processackmsg(instance_data_t *inst, uint8_t seqNum)
{
    if(inst->ackexpected) //used to ignore unexpected ACK frames
    {
		if(seqNum == inst->sentSN) //this is the ACK we expect
		{
			//we got the ACK for the last sent frame
			if(inst->previousState == TA_TXREPORT_WAIT_SEND)
			{
				//we got the ACK for the report, wait for next poll message
				//printf("we got the ACK for the report\n");
				dwt_setrxtimeout(0);
			}
			else if(inst->previousState == TA_TXRANGINGINIT_WAIT_SEND) //the tag ACKed our ranging request
			{
				dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN); //we are starting ranging - enable the filter....
                inst->frameFilteringEnabled = 1 ;

				//dwt_setrxtimeout(0); - don't need to disable as it is not set in TA_TXRANGINGINIT_WAIT_SEND
			}

			inst->ackexpected = 0 ;
		}
    }
}


void instance_txcallback(const dwt_callback_data_t *txd)
{
	int instance = 0;
	uint8_t txTimeStamp[5] = {0, 0, 0, 0, 0};

	uint8_t txevent = txd->event;
	event_data_t dw_event;

	if(instance_data[instance].ackreq) //the ACK has been requested in the last RX frame - we got TX event, this means the ACK has been sent
	{
		txevent = DWT_SIG_TX_AA_DONE;
		instance_data[instance].ackreq = 0;
        //add the pending event to the event queue, so that the received frame can be processed
        instance_putevent(instance_getsavedevent());
	}

	if(txevent == DWT_SIG_TX_DONE)
	{
		//uint64_t txtimestamp = 0;

		//NOTE - we can only get TX good (done) while here
		//dwt_readtxtimestamp((uint8_t*) &instance_data[instance].txu.txTimeStamp);

		dwt_readtxtimestamp(txTimeStamp) ;
		dw_event.timeStamp32l = (uint32_t)txTimeStamp[0] + ((uint32_t)txTimeStamp[1] << 8) + ((uint32_t)txTimeStamp[2] << 16) + ((uint32_t)txTimeStamp[3] << 24);
		dw_event.timeStamp = txTimeStamp[4];
	    dw_event.timeStamp <<= 32;
		dw_event.timeStamp += dw_event.timeStamp32l;
		dw_event.timeStamp32h = ((uint32_t)txTimeStamp[4] << 24) + (dw_event.timeStamp32l >> 8);

		instance_data[instance].stoptimer = 0;

		dw_event.rxLength = 0;
		dw_event.type2 = dw_event.type = DWT_SIG_TX_DONE ;

		instance_putevent(dw_event);

#if (DEEP_SLEEP == 1)
		instance_data[instance].txmsgcount++;
#endif
		//printf("TX time %f ecount %d\n",convertdevicetimetosecu(instance_data[instance].txu.txTimeStamp), instance_data[instance].dweventCnt);
		//printf("TX Timestamp: %4.15e\n", convertdevicetimetosecu(instance_data[instance].txu.txTimeStamp));
#if (DECA_SUPPORT_SOUNDING==1)
	#if DECA_ACCUM_LOG_SUPPORT==1
		if ((instance_data[instance].dwlogdata.accumLogging == LOG_ALL_ACCUM ) || (instance_data[instance].dwlogdata.accumLogging == LOG_ALL_NOACCUM ))
		{
			uint32_t hi32 = dw_event.timeStamp >> 32;
			uint32_t low32 = dw_event.timeStamp & 0xffffffff;
			fprintf(instance_data[instance].dwlogdata.accumLogFile,"\nTX Frame TimeStamp Raw  = %02X %02X%02X%02X%02X\n",txTimeStamp[4], txTimeStamp[3], txTimeStamp[2], txTimeStamp[1], txTimeStamp[0]) ;
			fprintf(instance_data[instance].dwlogdata.accumLogFile,"   Adding Antenna Delay = %04X %08X\n", hi32, low32 ) ;
			fprintf(instance_data[instance].dwlogdata.accumLogFile,"%02X Tx time = %4.15e\n", instance_data[instance].msg.seqNum, convertdevicetimetosecu(dw_event.timeStamp)) ;
		}
    #endif
#endif
	}
	else if(txevent == DWT_SIG_TX_AA_DONE)
	{
		//auto ACK confirmation
		dw_event.rxLength = 0;
		dw_event.type2 = dw_event.type = DWT_SIG_TX_AA_DONE ;

		instance_putevent(dw_event);

		//printf("TX AA time %f ecount %d\n",convertdevicetimetosecu(instance_data[instance].txu.txTimeStamp), instance_data[instance].dweventCnt);
	}
}

void instance_rxcallback(const dwt_callback_data_t *rxd)
{
	int instance = 0;
	uint8_t rxTimeStamp[5]  = {0, 0, 0, 0, 0};

    uint8_t rxd_event = 0;
	uint8_t fcode_index  = 0;
	event_data_t dw_event;

	//if we got a frame with a good CRC - RX OK
    if(rxd->event == DWT_SIG_RX_OKAY)
	{
		//ACK request bit?
		instance_data[instance].ackreq = rxd->aatset;

        //if(instance_data[instance].ackreq) //this indicates that the ACK request has been set
        //{
            //need to hold onto the received frame until the ACK has been sent
            //configuration and sending of ACK frame can also be done here when not using auto-ACK feature.
			//	printf("ACK request\n");
        //}

        rxd_event = DWT_SIG_RX_OKAY;

		dw_event.rxLength = rxd->datalength;

		//need to process the frame control bytes to figure out what type of frame we have received
        switch(rxd->fctrl[0])
	    {
			//blink type frame
		    case 0xC5:
				if(rxd->datalength == 12)
				{
					rxd_event = SIG_RX_BLINK;
				}
				else if(rxd->datalength == 18)//blink with Temperature and Battery level indication
				{
					rxd_event = SIG_RX_BLINK;
				}
				else
					rxd_event = SIG_RX_UNKNOWN;
					break;

			//ACK type frame
			case 0x02:
				if(rxd->datalength == 5)
					rxd_event = SIG_RX_ACK;
			    break;

			//data type frames (with/without ACK request) - assume PIDC is on.
			case 0x41:
			case 0x61:
				//read the frame
				if(rxd->datalength > STANDARD_FRAME_SIZE)
					rxd_event = SIG_RX_UNKNOWN;

				//need to check the destination/source address mode
				if((rxd->fctrl[1] & 0xCC) == 0x88) //dest & src short (16 bits)
				{
					fcode_index = FRAME_CRTL_AND_ADDRESS_S; //function code is in first byte after source address
				}
				else if((rxd->fctrl[1] & 0xCC) == 0xCC) //dest & src long (64 bits)
				{
					fcode_index = FRAME_CRTL_AND_ADDRESS_L; //function code is in first byte after source address
				}
				else //using one short/one long
				{
					fcode_index = FRAME_CRTL_AND_ADDRESS_LS; //function code is in first byte after source address
				}
				break;

			//any other frame types are not supported by this application
			default:
				rxd_event = SIG_RX_UNKNOWN;
				break;
		}


		//read rx timestamp
		if((rxd_event == SIG_RX_ACK) || (rxd_event == SIG_RX_BLINK) || (rxd_event == DWT_SIG_RX_OKAY))
		{
			dwt_readrxtimestamp(rxTimeStamp) ;
			dw_event.timeStamp32l =  (uint32_t)rxTimeStamp[0] + ((uint32_t)rxTimeStamp[1] << 8) + ((uint32_t)rxTimeStamp[2] << 16) + ((uint32_t)rxTimeStamp[3] << 24);
			dw_event.timeStamp = rxTimeStamp[4];
			dw_event.timeStamp <<= 32;
			dw_event.timeStamp += dw_event.timeStamp32l;
			dw_event.timeStamp32h = ((uint32_t)rxTimeStamp[4] << 24) + (dw_event.timeStamp32l >> 8);
		
			dwt_readrxdata((uint8_t *)&dw_event.msgu.frame[0], rxd->datalength, 0);  // Read Data Frame
			//dwt_readdignostics(&instance_data[instance].dwlogdata.diag);

			if(dwt_checkoverrun()) //the overrun has occured while we were reading the data - dump the frame/data
			{
				rxd_event = DWT_SIG_RX_ERROR ;
			}

		}
		
		dw_event.type2 = dw_event.type = rxd_event;
		
		//printf("rx call back %d %d FI:%08X %02x%02x event%d\n", rxd_event, instance_data[instance].ackreq, dwt_read32bitreg(0x10), rxd->fctrl[0], rxd->fctrl[1], instance_data[instance].dweventCnt);

		//Process good frames
		if(rxd_event == DWT_SIG_RX_OKAY)
		{
			//if(rxd->dblbuff == 0)  instance_readaccumulatordata();     // for diagnostic display in DecaRanging PC window
			//instance_calculatepower();

	    	instance_data[instance].stoptimer = 1;

            if(instance_data[instance].ackreq == 0) //only notify there is event if no ACK pending
	    		instance_putevent(dw_event);
			else
				instance_saveevent(dw_event);

			#if DECA_LOG_ENABLE==1
			#if DECA_KEEP_ACCUMULATOR==1
			{
				instance_data[instance].dwacclogdata.newAccumData = 1 ;
				instance_data[instance].dwacclogdata.erroredFrame = DWT_SIG_RX_NOERR ;	//no error
				processSoundingData();
			}
			#endif
				logSoundingData(DWT_SIG_RX_NOERR, dw_event.msgu.frame[fcode_index], dw_event.msgu.frame[2], &dw_event);
			#endif

			//printf("RX OK %d %x\n",instance_data[instance].testAppState, instance_data[instance].rxmsg.messageData[FCODE]);
			//printf("RX OK %d ", instance_data[instance].testAppState);
			//printf("RX time %f ecount %d\n",convertdevicetimetosecu(dw_event.timeStamp), instance_data[instance].dweventCnt);

	#if (DEEP_SLEEP == 1)
			instance_data[instance].rxmsgcount++;
	#endif
		}
		else if (rxd_event == SIG_RX_ACK)
		{
			//printf("RX ACK %d (count %d) \n", instance_data[instance].testAppState, instance_data[instance].dweventCnt);
			instance_putevent(dw_event);
			//if(rxd->dblbuff == 0) instance_readaccumulatordata();     // for diagnostic display
	#if (DEEP_SLEEP == 1)
			instance_data[instance].rxmsgcount++;
	#endif
		}
		else if (rxd_event == SIG_RX_BLINK)
		{
			instance_putevent(dw_event);
			//if(rxd->dblbuff == 0) instance_readaccumulatordata();     // for diagnostic display

			//instance_calculatepower();
			//printf("RX BLINK %d (count %d) \n", instance_data[instance].testAppState, instance_data[instance].dweventCnt);

			#if DECA_LOG_ENABLE==1
			#if DECA_KEEP_ACCUMULATOR==1
			{
				instance_data[instance].dwacclogdata.newAccumData = 1 ;
				instance_data[instance].dwacclogdata.erroredFrame = DWT_SIG_RX_NOERR ;	//no error
				processSoundingData();
			}
			#endif
				logSoundingData(DWT_SIG_RX_NOERR, 0xC5, dw_event.msgu.rxblinkmsg.seqNum, &dw_event);
			#endif

	#if (DEEP_SLEEP == 1)
				instance_data[instance].rxmsgcount++;
	#endif
		}

		/*if(instance_data[instance].mode == LISTENER) //print out the message bytes when in Listener mode
		{
			int i;
			uint8_t buffer[1024];
			dwt_readrxdata(buffer, rxd->datalength, 0);  // Read Data Frame
			buffer[1023] = 0;
			instancelogrxdata(&instance_data[instance], buffer, rxd->datalength);


			printf("RX data(%d): ", rxd->datalength);
			for(i=0; i<rxd->datalength; i++)
			{
				printf("%02x", buffer[i]);
			}
			printf("\n");
		}*/

		if (rxd_event == SIG_RX_UNKNOWN) //need to re-enable the rx
		{
			//if(rxd->dblbuff == 0) instance_readaccumulatordata();     // for diagnostic display

			//dwt_readdignostics(&instance_data[instance].dwlogdata.diag);

			//instance_calculatepower();

			#if DECA_LOG_ENABLE==1
			#if DECA_KEEP_ACCUMULATOR==1
			{
				instance_data[instance].dwacclogdata.newAccumData = 1 ;
				instance_data[instance].dwacclogdata.erroredFrame = DWT_SIG_RX_NOERR ;	//no error
				processSoundingData();
			}
			#endif
				logSoundingData(DWT_SIG_RX_NOERR, dw_event.msgu.frame[fcode_index], dw_event.msgu.frame[2], &dw_event);
			#endif

			if(instance_data[instance].doublebufferon == 0)
				instancerxon(&instance_data[instance], 0, 0); //immediate enable
		}
	}
	else if (rxd->event == DWT_SIG_RX_TIMEOUT)
	{
		dw_event.type2 = dw_event.type = DWT_SIG_RX_TIMEOUT;
		dw_event.rxLength = 0;
		dw_event.timeStamp = 0;
		dw_event.timeStamp32l = 0;
		dw_event.timeStamp32h = 0;

		instance_putevent(dw_event);
		//printf("RX timeout while in %d\n", instance_data[instance].testAppState);
	}
	else //assume other events are errors
	{
		//printf("RX error %d \n", instance_data[instance].testAppState);
		if(instance_data[instance].rxautoreenable == 0)
		{
			//re-enable the receiver
#if (DECA_BADF_ACCUMULATOR == 1)
			instance_readaccumulatordata();

			dwt_readdignostics(&instance_data[instance].dwlogdata.diag);

			instance_calculatepower();
#endif
#if (DECA_ERROR_LOGGING == 1)
			#if DECA_LOG_ENABLE==1
			#if DECA_BADF_ACCUMULATOR==1
			{
				instance_data[instance].dwacclogdata.newAccumData = 1 ;
				instance_data[instance].dwacclogdata.erroredFrame = rxd->event ;	//no error
				processSoundingData();
			}
			#endif
				logSoundingData(rxd->event, 0, 0, &dw_event);
			#endif
#endif

			//for ranging application rx error frame is same as TO - as we are not going to get the expected frame
			if((instance_data[instance].mode == TAG) || (instance_data[instance].mode == TAG_TDOA))
			{
				dw_event.type = DWT_SIG_RX_TIMEOUT;
				dw_event.type2 = 0x40 | DWT_SIG_RX_TIMEOUT;
				dw_event.rxLength = 0;

				instance_putevent(dw_event);
			}
			else
			{
				instancerxon(&instance_data[instance], 0, 0); //immediate enable if anchor or listener
			}

		}
	}
}


int instance_peekevent(void)
{
	int instance = 0;
    return instance_data[instance].dwevent[instance_data[instance].dweventPeek].type; //return the type of event that is in front of the queue
}

void instance_saveevent(event_data_t newevent)
{
	int instance = 0;

	instance_data[instance].saved_dwevent = newevent;
}

event_data_t instance_getsavedevent(void)
{
	int instance = 0;

	return instance_data[instance].saved_dwevent;
}

void instance_putevent(event_data_t newevent)
{
	int instance = 0;
	uint8_t etype = newevent.type;

	newevent.type = 0;
	//newevent.eventtime = portGetTickCount();
	//newevent.gotit = newevent.eventtimeclr = 0;

	//copy event
	instance_data[instance].dwevent[instance_data[instance].dweventIdxIn] = newevent;

	//set type - this makes it a new event (making sure the event data is copied before event is set as new)
	//to make sure that the get event function does not get an incomplete event
	instance_data[instance].dwevent[instance_data[instance].dweventIdxIn].type = etype;

	instance_data[instance].dweventIdxIn++;

	if(MAX_EVENT_NUMBER == instance_data[instance].dweventIdxIn)
		instance_data[instance].dweventIdxIn = 0;

	//eventIncount++;

	//printf("put %d - in %d out %d @ %d\n", newevent.type, instance_data[instance].dweventCntIn, instance_data[instance].dweventCntOut, ptime);
}

event_data_t dw_event_g;

#pragma GCC optimize ("O0")
event_data_t* instance_getevent(int x)
{
	int instance = 0;
	int indexOut = instance_data[instance].dweventIdxOut;

	//dw_event_g = instance_data[instance].dwevent[instance_data[instance].dweventCntOut]; //this holds any TX/RX events

	//memcpy(&dw_event_g, &instance_data[instance].dwevent[instance_data[instance].dweventCntOut], sizeof(event_data_t));

	if(instance_data[instance].dwevent[indexOut].type == 0) //exit with "no event"
	{
		dw_event_g.type = 0;
		dw_event_g.type2 = 0;
		return &dw_event_g;
	}

	//copy the event
	dw_event_g.type2 = instance_data[instance].dwevent[indexOut].type2 ;
	dw_event_g.rxLength = instance_data[instance].dwevent[indexOut].rxLength ;
	dw_event_g.timeStamp = instance_data[instance].dwevent[indexOut].timeStamp ;
	dw_event_g.timeStamp32l = instance_data[instance].dwevent[indexOut].timeStamp32l ;
	dw_event_g.timeStamp32h = instance_data[instance].dwevent[indexOut].timeStamp32h ;
	//dw_event_g.eventtime = instance_data[instance].dwevent[indexOut].eventtime ;
	//dw_event_g.eventtimeclr = instance_data[instance].dwevent[indexOut].eventtimeclr ;
	//dw_event_g.gotit = instance_data[instance].dwevent[indexOut].gotit ;

	memcpy(&dw_event_g.msgu, &instance_data[instance].dwevent[indexOut].msgu, sizeof(instance_data[instance].dwevent[indexOut].msgu));

	dw_event_g.type = instance_data[instance].dwevent[indexOut].type ;


	//instance_data[instance].dwevent[indexOut].gotit = x;

	//instance_data[instance].dwevent[indexOut].eventtimeclr = portGetTickCount();

	instance_data[instance].dwevent[indexOut].type = 0; //clear the event

	instance_data[instance].dweventIdxOut++;
	if(MAX_EVENT_NUMBER == instance_data[instance].dweventIdxOut) //wrap the counter
		instance_data[instance].dweventIdxOut = 0;

	instance_data[instance].dweventPeek = instance_data[instance].dweventIdxOut; //set the new peek value

	//if(dw_event.type) printf("get %d - in %d out %d @ %d\n", dw_event.type, instance_data[instance].dweventCntIn, instance_data[instance].dweventCntOut, ptime);

	//eventOutcount++;


	return &dw_event_g;
}

void instance_clearevents(void)
{
	int i = 0;
	int instance = 0;

	for(i=0; i<MAX_EVENT_NUMBER; i++)
	{
        memset(&instance_data[instance].dwevent[i], 0, sizeof(event_data_t));
	}

	instance_data[instance].dweventIdxIn = 0;
	instance_data[instance].dweventIdxOut = 0;
	instance_data[instance].dweventPeek = 0;

	//eventOutcount = 0;
	//eventIncount = 0;
}

void instance_setapprun(int (*apprun_fn)(instance_data_t *inst, int message))
{
	int instance = 0 ;
	instance_localdata[instance].testapprun_fn = apprun_fn;
}

// -------------------------------------------------------------------------------------------------------------------
int instance_run(void)
{
    int instance = 0 ;
    int done = INST_NOT_DONE_YET;
    int message = instance_peekevent(); //get any of the received events from ISR


        while(done == INST_NOT_DONE_YET)
        {
            //int state = instance_data[instance].testAppState;
            done = instance_localdata[instance].testapprun_fn(&instance_data[instance], message) ;                                               // run the communications application

            //we've processed message
            message = 0;
        }



    if(done == INST_DONE_WAIT_FOR_NEXT_EVENT_TO) //we are in RX and need to timeout (Tag needs to send another poll if no Rx frame)
    {
        if(instance_data[instance].mode == TAG) //Tag (is either in RX or sleeping)
        {
            instance_data[instance].instancetimer = portGetTickCount() + instance_data[instance].tagSleepTime_ms; //set timeout time
            instance_data[instance].instancetimer_en = 1; //start timer
        }
        if(instance_data[instance].mode == TAG_TDOA)
        {
            instance_data[instance].instancetimer = portGetTickCount() + instance_data[instance].tagBlinkSleepTime_ms; //set timeout time
            instance_data[instance].instancetimer_en = 1; //start timer
        }
        instance_data[instance].stoptimer = 0 ; //clear the flag - timer can run if instancetimer_en set (set above)
        instance_data[instance].done = INST_NOT_DONE_YET;
    }

    //check if timer has expired
    if((instance_data[instance].instancetimer_en == 1) && (instance_data[instance].stoptimer == 0))
    {
        if(instance_data[instance].instancetimer < portGetTickCount())
        {
			event_data_t dw_event;
            instance_data[instance].instancetimer_en = 0;
			dw_event.rxLength = 0;
			dw_event.type = DWT_SIG_RX_TIMEOUT;
			dw_event.type2 = 0x80 | DWT_SIG_RX_TIMEOUT;
			//printf("PC timeout DWT_SIG_RX_TIMEOUT\n");
			instance_putevent(dw_event);
        }
    }

    return 0 ;
}


void instance_close(void)
{
    //wake up device from low power mode
    //NOTE - in the ARM  code just drop chip select for 200us
    nSELECT = 0;  //CS low
    Delay(1);   //200 us to wake up then waits 5ms for DW1000 XTAL to stabilise
    nSELECT = 1;   //CS high
    Delay(5);
    dwt_entersleepaftertx(0); // clear the "enter deep sleep after tx" bit

    dwt_setinterrupt(0xFFFFFFFF, 0); //don't allow any interrupts

}


void instance_notify_DW1000_inIDLE(int idle)
{
	instance_data[0].dwIDLE = idle;
}
#endif


/* ==========================================================

Notes:

Previously code handled multiple instances in a single console application

Now have changed it to do a single instance only. With minimal code changes...(i.e. kept [instance] index but it is always 0.

Windows application should call instance_init() once and then in the "main loop" call instance_run().

*/
