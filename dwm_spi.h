/* 
 * File:   dwm_spi.h
 * Author: jonathan
 *
 * Created on June 9, 2015, 11:20 AM
 */

#ifndef DWM_SPI_H
#define	DWM_SPI_H

#include "decadriver/deca_types.h"
#include "decadriver/deca_device_api.h"
#include "../sensor_spi2.h"
#include "../sensor_pindefs.h"

extern dwm_1000_status dwm_status;

typedef struct
{
    uint8_t channel ;
    uint8_t prf ;
    uint8_t datarate ;
    uint8_t preambleCode ;
    uint8_t preambleLength ;
    uint8_t pacSize ;
    uint8_t nsSFD ;
    uint16_t sfdTO ;
} chConfig_t ;

#ifdef	__cplusplus
extern "C" {
#endif

    uint8_t dwm_init(int mode);

    /**
     * This function takes the information provided by the decaWave DWM1000 API
     * and writes it out using the low SPI on the dsPIC33E
     * @param headerLength
     * @param headerBuffer
     * @param bodylength
     * @param bodyBuffer
     * @return
     */
    extern int writetospi( uint16_t headerLength,
                                const uint8_t *headerBuffer,
                                uint32_t bodylength,
                                const uint8_t *bodyBuffer
                                );

    /**
     * This function takes the information provided by the decaWave DWM1000 API
     * and reads it in using the low SPI on the dsPIC33E
     * @param headerLength
     * @param headerBuffer
     * @param readlength
     * @param readBuffer
     * @return
     */
    extern int readfromspi( uint16_t	headerLength,
                                const uint8_t *headerBuffer,
                                uint32_t readlength,
                                uint8_t *readBuffer
                                );

    /**
     * Reads a number of bytes over SPI. Ignores any information written to SPI
     * while reading
     * @param num_bytes: How many SPI reads to implement
     * @param rec: pointer which stores the read information
     */
    void read_data_SPI(uint16_t num_bytes, uint8_t* rec);

    /**
     * Writes a number of bytes over SPI. Ignores any information to read while
     * writing.
     * @param data: pointer to data to be written
     * @param num_bytes: length of the data to be written
     */
    void write_data_SPI(const uint8_t* data, uint16_t num_bytes);

#ifdef	__cplusplus
}
#endif

#endif	/* DWM_SPI_H */

