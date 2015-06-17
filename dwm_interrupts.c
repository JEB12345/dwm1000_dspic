/* 
 * File:   dwm_interrupts.c
 * Author: jonathan
 *
 * Created on June 15, 2015, 8:08 AM
 */

#include "dwm_interrupts.h"
#include "decadriver/instance.h"
#include <stdio.h>
#include <stdlib.h>
#include <xc.h>

extern instance_localdata_t instance_localdata[NUM_INST] ;

// -------------------------------------------------------------------------------------------------------------------
// function to initialise instance structures
//
// Returns 0 on success and -1 on error
uint8_t interrupt_init_s(uint8_t mode)
{
    int instance = 0 ;
    int result;
    //uint16 temp = 0;

    instance_data[instance].mode =  ANCHOR;                                // assume listener,

    instance_data[instance].instToSleep = 0;

    instance_data[instance].sentSN = 0;

    instance_data[instance].tofindex = 0;
    instance_data[instance].tofcount = 0;

    // Reset the IC (might be needed if not getting here from POWER ON)
    // ARM code: Remove soft reset here as using hard reset in the inittestapplication() in the main.c file
    //dwt_softreset();

#if (DEEP_SLEEP_AUTOWAKEUP == 1)
	uint16 blinktime  = 0xf; //e.g. blink time to be used for the Tag auto wake up
    {
        double t;
		uint16 lp_osc_cal = dwt_calibratesleepcnt(); //calibrate low power oscillator
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
