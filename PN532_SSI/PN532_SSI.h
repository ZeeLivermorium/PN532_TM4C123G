/*!
 * @file PN532_SSI.h
 * ----------
 * Adapted code from elechouse PN532 driver for Arduino.
 * You can find the elechouse PN532 driver here:
 * https://github.com/elechouse/PN532.git
 * ----------
 * For future development and updates, please follow this repository:
 * https://github.com/ZeeLivermorium/PN532_TM4C123
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
 *                     R/W API                      *
 *                                                  *
 ****************************************************/
int writeCommand(uint8_t *cmd, uint8_t cmd_length);

int16_t readResponse(uint8_t *data_buffer, uint8_t data_length);


#endif

#endif

