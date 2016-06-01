/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef BLECUBE_V1_H
#define BLECUBE_V1_H

// LEDs definitions for BLECARD
//Define shift register pins
#define RCLK 	3      
#define SRCLK 	4
#define SRCLR 	6
#define OE      5
#define QA1 	0
#define QA2 	1
#define QA3 	2

//Define button pins
#define btn1 8
#define btn2 7
#define btn3 10
#define btn4 9
#define btn5 12
#define btn6 11

#define RX_PIN_NUMBER  13
#define TX_PIN_NUMBER  14
#define CTS_PIN_NUMBER 15
#define RTS_PIN_NUMBER 16
#define HWFC           true


// serialization APPLICATION board - temp. setup for running serialized MEMU tests
/*
#define SER_APP_RX_PIN              31    // UART RX pin number.
#define SER_APP_TX_PIN              30    // UART TX pin number.
#define SER_APP_CTS_PIN             28    // UART Clear To Send pin number.
#define SER_APP_RTS_PIN             29    // UART Request To Send pin number.
*/

// serialization CONNECTIVITY board
/*
#define SER_CON_RX_PIN              13    // UART RX pin number.
#define SER_CON_TX_PIN              12    // UART TX pin number.
#define SER_CON_CTS_PIN             14    // UART Clear To Send pin number. Not used if HWFC is set to false.
#define SER_CON_RTS_PIN             15    // UART Request To Send pin number. Not used if HWFC is set to false.
*/

#define SER_CONN_CHIP_RESET_PIN     26    // Pin used to reset connectivity chip

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#endif // PCA10040_H
