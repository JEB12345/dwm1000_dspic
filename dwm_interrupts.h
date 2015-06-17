/* 
 * File:   dwm_interrupts.h
 * Author: jonathan
 *
 * Created on June 15, 2015, 8:08 AM
 */

#ifndef DWM_INTERRUPTS_H
#define	DWM_INTERRUPTS_H

#include "dwm_spi.h"
#include "decadriver/instance.h"
#include "decadriver/deca_param_types.h"
#include "decadriver/deca_device_api.h"

#ifdef	__cplusplus
extern "C" {
#endif

    /**
     * Initializes the interrupts used by the DWM1000
     * @param mode This sets what type of device the module is. Achor or Tag
     * @return 0: All good, -1: Error
     */
    uint8_t interrupt_init_s(uint8_t mode);

#ifdef	__cplusplus
}
#endif

#endif	/* DWM_INTERRUPTS_H */

