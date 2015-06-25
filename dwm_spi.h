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
#include "dwm_mutex.h"
#include "../sensor_spi2.h"
#include "../sensor_pindefs.h"

extern dwm_1000_status dwm_status;

#define TAG_SEND_POLL_DELAY_US 250
#define TAG_SEND_FINAL_DELAY_US 290
#define TX_ANTENNA_DELAY 0

typedef enum {
    tag_init, tag_poll, tag_wait_response, tag_final, tag_stall
}tag_states;

typedef enum {
    anchor_init, anchor_wait_receive, anchor_wait_final
}anchor_states;

/**
 * Code Copied directly or modified
 * from polypoint github code
 * https://github.com/lab11/polypoint
 */
#define DWT_PRF_64M_RFDLY 514.462f

#define TAG 1
#define ANCHOR 2

//#define DW_DEBUG
//#define DW_CAL_TRX_DELAY

// 4 packet types
#define MSG_TYPE_TAG_POLL   0x61
#define MSG_TYPE_ANC_RESP   0x50
#define MSG_TYPE_TAG_FINAL  0x69
#define MSG_TYPE_ANC_FINAL  0x51

#ifdef IS_ANCHOR
#define DW1000_ROLE_TYPE ANCHOR
#endif
#ifdef IS_TAG
#define DW1000_ROLE_TYPE TAG
#endif

#define ANCHOR_CAL_LEN (0.914-0.18) //0.18 is post-over-air calibration

#define TAG_EUI 0
#define ANCHOR_EUI 1
#define NUM_ANCHORS 1

#define DW1000_PANID 0xD100
#define SPEED_OF_LIGHT 299702547.0
#define NODE_DELAY_US 6500
#define ANC_RESP_DELAY 1000
#define DELAY_MASK 0x00FFFFFFFE00
#define NUM_ANTENNAS 1
#define NUM_CHANNELS 1


/*****https://github.com/lab11/polypoint******/

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

    uint8_t dwm_init();

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

    /**
     * Code Copied directly or modified
     * from polypoint github code
     * https://github.com/lab11/polypoint
     */
    void dw1000_populate_eui (uint8_t *eui_buf, uint8_t id);

    void app_dw1000_txcallback (const dwt_callback_data_t *txd);

    void app_dw1000_rxcallback (const dwt_callback_data_t *rxd);

    void send_poll();

    void instance_process();

    void incr_subsequence_counter();
    /*****https://github.com/lab11/polypoint******/

    uint32_t convertmicrosectodevicetimeu32 (double microsecu);

    void instancesettagsleepdelay(int sleepdelay, int blinksleepdelay);

#ifdef	__cplusplus
}
#endif

#endif	/* DWM_SPI_H */
