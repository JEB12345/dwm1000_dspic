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





#define DWM_RESET_ON DWM_RESET_TRIS = 0; DWM_RESET_OUT = 0
#define DWM_RESET_OFF DWM_RESET_TRIS = 1

#define TAG_SEND_POLL_DELAY_US 600 //600
#define TAG_SEND_FINAL_DELAY_US 800
#define FIXED_SEND_DISTANCES_DELAY_US 600
#define ANC_RESP_DELAY 600
#define TX_ANTENNA_DELAY 0

#define TWOPOWER40 1099511627776L // decimal value of 2^40 to correct timeroverflow between timestamps

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
//#define ANCHOR 2

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
#define SPEED_OF_LIGHT 299702548.0
#define NODE_DELAY_US 6500
#define DELAY_MASK 0x00FFFFFFFE00
#define NUM_ANTENNAS 1
#define NUM_CHANNELS 1

#define NUM_FLOATING_NODES  12 //nodes on robot
#define NUM_FIXED_NODES     9 //fixed nodes (reference) 8+1 to account for node 15 not working
#define NUM_TOTAL_NODES     (NUM_FLOATING_NODES + NUM_FIXED_NODES)//16 //sum of NUM_FLOATING_NODES and NUM_FIXED_NODES


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

enum dwm_tx_state {
        DWM_IDLE = 0,
        DWM_SEND_POLL = 1,
        DWM_SEND_RESPONSE = 2,
        DWM_SEND_FINAL = 3,
        DWM_COMPUTE_TOF = 4,
        DWM_SEND_DISTANCES = 5
    } ;

    typedef struct {
        uint8_t     irq_enable;
        double      distance[NUM_TOTAL_NODES]; //current distance measurement (raw)
        uint16_t    distance_mm[NUM_TOTAL_NODES]; //current distance measurement in mm (fixed offset removed to fit in uint16)
        void (*timer_func)(uint16_t microseconds,void (*cb)());
        uint16_t    distance_mm_fixed[NUM_FIXED_NODES][NUM_TOTAL_NODES]; //distances (mm) measured by the fixed nodes (in mm) 
        volatile unsigned timer_interrupt;
        enum dwm_tx_state   tx_state;
        uint8_t node_id;
        dwt_rxdiag_t rxdiag;
        uint8_t     new_data_flag; //this flag signifies that a measurement data point has been processed and the CAN PDO should be sent 
    } dwm_1000_status;

#ifdef	__cplusplus
extern "C" {
#endif

    uint8_t dwm_init(uint8_t node_id, void (*timer_func)(uint16_t microseconds,void (*cb)()));
    
    /**
     * Call this function when dwm_status.timer_interrupt = 1
     */
    void dwt_timer_interrupt();

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
    
    uint64_t convertmicrosectodevicetimeu (double microsecu);

    void instancesettagsleepdelay(int sleepdelay, int blinksleepdelay);

#ifdef	__cplusplus
}
#endif

#endif	/* DWM_SPI_H */

