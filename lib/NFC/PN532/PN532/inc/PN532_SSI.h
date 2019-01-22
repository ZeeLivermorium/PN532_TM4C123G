/*!
 * @file PN532_SSI.h
 * @brief SSI communication implementation for PN532.
 * ----------
 * Adapted code from Seeed Studio PN532 driver for Arduino.
 * You can find the Seeed Studio PN532 driver here: https://github.com/Seeed-Studio/PN532
 * ----------
 * NXP PN532 Data Sheet: https://www.nxp.com/docs/en/nxp/data-sheets/PN532_C1.pdf
 * NXP PN532 User Manual: https://www.nxp.com/docs/en/user-guide/141520.pdf
 * ----------
 * For future development and updates, please follow this repository: https://github.com/ZeeLivermorium/PN532_TM4C123
 * ----------
 * If you find any bug or problem, please create new issue or a pull request with a fix in the repository.
 * Or you can simply email me about the problem or bug at zeelivermorium@gmail.com
 * Much Appreciated!
 * ----------
 * @author Zee Livermorium
 * @date Apr 14, 2018
 */

#ifndef __PN532_SSI_H__
#define __PN532_SSI_H__

#include "PN532_Setting.h"

/*
 *  SSI0 A Conncection | SSI1 D Conncection | SSI1 F Conncection | SSI2 B Conncection | SSI3 D Conncection
 *  ------------------ | ------------------ | ------------------ | ------------------ | ------------------
 *  SCK  --------- PA2 | SCK  --------- PD0 | SCK  --------- PF2 | SCK  --------- PB4 | SCK  --------- PD0
 *  SS   --------- PA3 | SS   --------- PD1 | SS   --------- PF3 | SS   --------- PB5 | SS   --------- PD1
 *  MISO --------- PA4 | MISO --------- PD2 | MISO --------- PF0 | MISO --------- PB6 | MISO --------- PD2
 *  MOSI --------- PA5 | MOSI --------- PD3 | MOSI --------- PF1 | MOSI --------- PB7 | MOSI --------- PD3
 */

#ifdef PN532_SSI

#include <stdint.h>

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

int16_t readResponse(uint8_t *data_buffer, uint8_t data_length, uint16_t wait_time);


#endif

#endif

