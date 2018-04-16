/*!
 * @file PN532_SSI.h
 * ----------
 * Inspired by examples in ValvanoWareTM4C123 by Dr. Jonathan Valvano
 * as well as his book Embedded Systems: Real-Time Interfacing to Arm Cortex-M Microcontrollers
 * You can find ValvanoWareTM4C123 at http://edx-org-utaustinx.s3.amazonaws.com/UT601x/ValvanoWareTM4C123.zip?dl=1
 * You can find his book at https://www.amazon.com/gp/product/1463590156/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1
 * You can find more of his work at http://users.ece.utexas.edu/~valvano/
 * ----------
 * @author Zee Livermorium
 * @date Apr 14, 2018
 */

#ifndef __PN532_SSI_H__
#define __PN532_SSI_H__

#include "PN532_Setting.h"

#ifdef SSI

/* Data Frame Byte */
#define PN532_PREAMBLE                      (0x00)
#define PN532_STARTCODE1                    (0x00)
#define PN532_STARTCODE2                    (0xFF)
#define PN532_POSTAMBLE                     (0x00)

#define PN532_HOSTTOPN532                   (0xD4)
#define PN532_PN532TOHOST                   (0xD5)

/* SPI/SSI */
#define PN532_SPI_STATREAD                  (0x02)
#define PN532_SPI_DATAWRITE                 (0x01)
#define PN532_SPI_DATAREAD                  (0x03)
#define PN532_SPI_READY                     (0x01)

/* Wait Time */
#define PN532_ACK_WAIT_TIME                 (10)

/* Error Code */
#define PN532_INVALID_ACK                   (-1)
#define PN532_TIMEOUT                       (-2)
#define PN532_INVALID_FRAME                 (-3)
#define PN532_NO_SPACE                      (-4)

/****************************************************
 *                                                  *
 *                    Properties                    *
 *                                                  *
 ****************************************************/
static uint8_t command;                 // variable to hold command sent
static const uint8_t ACK_frame[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};


/****************************************************
 *                                                  *
 *                   Initializer                    *
 *                                                  *
 ****************************************************/

/**
 * PN532_SSI_Init
 * ----------
 * @brief initialize SSI0 on Port A with corresponding setting parameters.
 */
void PN532_SSI_Init (void);


/****************************************************
 *                                                  *
 *                Internal Functions                *
 *                                                  *
 ****************************************************/
int writeCommand(uint8_t *cmd, uint8_t cmd_length);

int16_t readResponse(uint8_t *data_buffer, uint8_t data_length);


/****************************************************
 *                                                  *
 *                   I/O Functions                  *
 *                                                  *
 ****************************************************/

/**
 * SSI0_Read
 * ----------
 * @return: date read from another device.
 */
static uint8_t PN532_SSI_read (void);

/**
 * SSI0_write
 * ----------
 * @param  data  data to be written.
 */
static void PN532_SSI_write(uint8_t data);


#endif

#endif

