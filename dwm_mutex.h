/* 
 * File:   dwm_mutex.h
 * Author: jonathan
 *
 * Created on June 11, 2015, 4:24 PM
 */

#ifndef DWM_MUTEX_H
#define	DWM_MUTEX_H

#include "decadriver/deca_device_api.h"

#ifdef	__cplusplus
extern "C" {
#endif

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: decamutexon()
 *
 * Description: This function should disable interrupts. This is called at the start of a critical section
 * It returns the irq state before disable, this value is used to re-enable in decamutexoff call
 *
 * Note: The body of this function is defined in deca_mutex.c and is platform specific
 *
 * input parameters:
 *
 * output parameters
 *
 * returns the state of the DW1000 interrupt
 */
extern decaIrqStatus_t decamutexon(void);

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: decamutexoff()
 *
 * Description: This function should re-enable interrupts, or at least restore their state as returned(&saved) by decamutexon
 * This is called at the end of a critical section
 *
 * Note: The body of this function is defined in deca_mutex.c and is platform specific
 *
 * input parameters:
 * @param s - the state of the DW1000 interrupt as returned by decamutexon
 *
 * output parameters
 *
 * returns the state of the DW1000 interrupt
 */
extern void decamutexoff(decaIrqStatus_t s);


#ifdef	__cplusplus
}
#endif

#endif	/* DWM_MUTEX_H */

