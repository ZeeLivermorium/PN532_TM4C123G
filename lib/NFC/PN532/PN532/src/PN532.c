/*!
 * @file  PN532.c
 * @brief NXP PN532 NFC Library.
 * ----------
 * Adapted code from Seeed Studio PN532 driver for Arduino.
 * You can find the Seeed Studio PN532 driver here: https://github.com/Seeed-Studio/PN532
 * ----------
 * NXP PN532 Data Sheet:  https://www.nxp.com/docs/en/nxp/data-sheets/PN532_C1.pdf
 * NXP PN532 User Manual: https://www.nxp.com/docs/en/user-guide/141520.pdf
 * ----------
 * For future development and updates, please follow this repository: https://github.com/ZeeLivermorium/PN532_TM4C123
 * ----------
 * If you find any bug or problem, please create new issue or a pull request with a fix in the repository.
 * Or you can simply email me about the problem or bug at zeelivermorium@gmail.com
 * Much Appreciated!
 * ----------
 * @author Zee Livermorium
 * @date   Dec 25, 2017
 */

#include <stdint.h>
#include <string.h>
#include "Serial.h"
#include "PN532.h"
#include "PN532_Setting.h"

#if defined PN532_SSI
#include "PN532_SSI.h"
#elif defined PN532_I2C
// #include "PN532_I2C.h" // NOT SUPPORTED
#elif defined PN532_UART
// #include "PN532_UART.h" // NOT SUPPORTED
#endif

/****************************************************
 *                                                  *
 *                    Properties                    *
 *                                                  *
 ****************************************************/

static uint8_t packet_buffer[255];      // packet buffer for data exchange
//static uint8_t inListedTag;             // Tg number of inlisted tag.
//static uint8_t _felicaIDm[8];           // FeliCa IDm (NFCID2)
//static uint8_t _felicaPMm[8];           // FeliCa PMm (PAD)

/****************************************************
 *                                                  *
 *                 Helper Functions                 *
 *                                                  *
 ****************************************************/

/**
 * delay
 * ----------
 * Description: delay N time unit
 */
void delay(uint32_t N) {
    for(int n = 0; n < N; n++)                         // N time unitss
        for(int msec = 10000; msec > 0; msec--);       // 1 time unit
}


/****************************************************
 *                                                  *
 *                   Initializer                    *
 *                                                  *
 ****************************************************/

/**
 * PN532_Init
 * ----------
 * @brief: initialize communication with PN532. Change settings in PN532_Setting.h.
 */
void PN532_Init(void) {
#ifdef PN532_SSI
    PN532_SSI_Init();
#elif defined PN532_I2C
    // PN532_I2C_Init(); NOT SUPPORTED
#elif defined PN532_UART
    // PN532_UART_Init(); NOT SUPPORTED
#endif
}


/****************************************************
 *                                                  *
 *                  PN532 Commands                  *
 *                                                  *
 ****************************************************/

/**
 * PN532_getFirmwareVersion
 * ----------
 * @return 32-bit firmware version number for PN532 or 0 for an error.
 * ----------
 * @note   details in user manual section 7.2.2 GetFirmwareVersion.
 */
uint32_t PN532_getFirmwareVersion(void) {
    /*-- prepare command --*/
    packet_buffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;
    
    /*-- write command and read response --*/
    if (!writeCommand(packet_buffer, 1)) return 0;     // write command to PN532, return 0 means write fail
    if (readResponse(packet_buffer, 12, PN532_WAITTIME_1K) < 0) return 0; // read response from PN532, return 0 for error
    
    /*-- organize the result to an unsigned 32 bit integer --*/
    uint32_t response;
    response = packet_buffer[0];
    response <<= 8;
    response |= packet_buffer[1];
    response <<= 8;
    response |= packet_buffer[2];
    response <<= 8;
    response |= packet_buffer[3];
    
    return response;
}

/**
 * PN532_SAMConfiguration
 * ----------
 * @return -1 for an error.
 * ----------
 * @brief  Configures the SAM (Secure Access Module).
 * ----------
 * @note   details in user manual section 7.2.10 SAMConfiguration.
 */
int8_t PN532_SAMConfiguration(void) {
    /*-- prepare command --*/
    packet_buffer[0] = PN532_COMMAND_SAMCONFIGURATION;
    packet_buffer[1] = 0x01;                           // normal mode;
    packet_buffer[2] = 0x14;                           // timeout 50ms * 20 = 1 second
    packet_buffer[3] = 0x01;                           // use IRQ pin!
    
    /*-- write command and read response --*/
    if (!writeCommand(packet_buffer, 4)) return -1;
    return readResponse(packet_buffer, sizeof(packet_buffer), PN532_WAITTIME_1K);  // return response status code
}

/**
 * PN532_inJumpForDEP
 * ----------
 * @return 0 for an error.
 * ----------
 * @brief  This command is used by a host controller to activate a target using either
 *         active or passive communication mode.
 * ----------
 * @note   details in user manual section 7.3.3 InJumpForDEP.
 */
uint8_t PN532_inJumpForDEP() {
    /** avoid resend command */
    packet_buffer[0] = PN532_COMMAND_INJUMPFORDEP;
    packet_buffer[1] = 0x01;      // avtive mode
    packet_buffer[2] = 0x02;      // 424Kbps
    packet_buffer[3] = 0x01;      // NFCID3i
    
    if (!writeCommand(packet_buffer, 4))
        return 0;   // return 0 for error
    
    return readResponse(packet_buffer, 25, PN532_WAITTIME_1K);
}

/**
 * PN532_inRelease
 * ----------
 * @param  relevant_target  targets to be released.
 * ----------
 * @return specific PN532 error code.
 * ----------
 * @brief  to release the target(s).
 * ----------
 * @note   details in user manual section 7.3.11 InRelease.
 */
int16_t PN532_inRelease(const uint8_t relevant_target) {
    
    packet_buffer[0] = PN532_COMMAND_INRELEASE;
    packet_buffer[1] = relevant_target;              // 0 inidicates all targets
    
    if (!writeCommand(packet_buffer, 2)) return 0;   // return 0 for error
    
    return readResponse(packet_buffer, sizeof(packet_buffer), PN532_WAITTIME_1K);
}

/**
 * PN532_tgInitAsTarget
 * ----------
 * @param  cmd         command to be sent.
 * @param  cmd_length  length of whole command.
 * @param  wait_time   wait time for PN532 to respond to this particular command.
 * ----------
 * @return specific PN532 error code.
 * ----------
 * @brief  to configure the PN532 as target.
 * ----------
 * @note   details in user manual section 7.3.14 TgInitAsTarget.
 */
int8_t PN532_tgInitAsTarget(uint8_t* cmd, uint8_t cmd_length, uint16_t wait_time) {
    /*-- write command and read response --*/
    if (!writeCommand(cmd, cmd_length)) return 0;   // return 0 for error
    return readResponse(                            // return 0 or less for error
                        packet_buffer,
                        sizeof(packet_buffer),
                        PN532_WAITTIME_30K
                        ) > 0;
}

/**
 * PN532_tgGetData
 * ----------
 * @param  data_buff    Container to obtain data.
 * @param  buff_length  Length of data to be obtained.
 * ----------
 * @return specific PN532 error code.
 * ----------
 * @brief  This command is used in case of the PN532 configured as target for Data Exchange Protocol (DEP)
 *         or for ISO/IEC14443-4 protocol when PN532 is activated in ISO/IEC14443-4 PICC emulated.
 * ----------
 * @note   details in user manual section 7.3.16 TgGetData.
 */
int16_t PN532_tgGetData(uint8_t *data_buff, uint8_t buff_length) {
    
    packet_buffer[0] = PN532_COMMAND_TGGETDATA;
    
    if (!writeCommand(packet_buffer, 1)) return 0;   // return 0 for error

    uint16_t response_status = readResponse (
                                             data_buff,
                                             buff_length + 1, // must + 1, to ignore status byte
                                             PN532_WAITTIME_3K
                                             );
    if ( response_status <= 0 ) return 0;                  // would return length of the data if no error
    uint16_t data_length = response_status - 1;
    
    if ( data_buff[0] != 0 ) return -1;                    // return status not right from PN532
    for (uint8_t i = 0; i < data_length; i++) {
        data_buff[i] = data_buff[i + 1];                   // move over the data
    }
    return data_length;
}

/**
 * PN532_tgSetData
 * ----------
 * @param  data    Container for data to be sent.
 * @param  length  Length of data to be sent.
 * ----------
 * @return 0 for error, 1 for success.
 * ----------
 * @brief  This command is used in case of the PN532 configured as target for Data Exchange Protocol (DEP)
 *         or for ISO/IEC14443-4 protocol when PN532 is activated in ISO/IEC14443-4 PICC emulated.
 * ----------
 * @note   details in user manual section 7.3.17 TgSetData.
 */
uint8_t PN532_tgSetData(const uint8_t* data, uint8_t length) {
    
    if (length > (sizeof(packet_buffer) - 1)) {
        return 0;
    } else {
        packet_buffer[0] = PN532_COMMAND_TGSETDATA;
        for (int8_t i = length - 1; i >= 0; i--){
            packet_buffer[i + 1] = data[i];
        }
        if (!writeCommand(packet_buffer, length + 1))
            return 0;           // return 0 for error
    }
    
    if (readResponse(packet_buffer, sizeof(packet_buffer), PN532_WAITTIME_1K) < 0)
        return 0;               // read response return negative is error
    
    if (0 != packet_buffer[0])  // status return non zero is error
        return 0;
    
    return 1;
}


/**
 * setPassiveActivationRetries
 * ----------
 * @param  maxRetries  0xFF to wait forever, 0x00..0xFE to timeout after mxRetries.
 * ----------
 * @return -1 for an error.
 * ----------
 * @brief Sets the MxRtyPassiveActivation uint8_t of the RFConfiguration register.
 * ----------
 * Data Sheet: section 7.3.1 RFConfiguration (page 101).
 */
int8_t setPassiveActivationRetries(uint8_t maxRetries) {
    /*-- prepare command --*/
    packet_buffer[0] = PN532_COMMAND_RFCONFIGURATION;
    packet_buffer[1] = 5;                              // Config item 5 (MaxRetries)
    packet_buffer[2] = 0xFF;                           // MxRtyATR (default = 0xFF)
    packet_buffer[3] = 0x01;                           // MxRtyPSL (default = 0x01)
    packet_buffer[4] = maxRetries;
    
    /*-- write command and read response --*/
    if (!writeCommand(packet_buffer, 5)) return -1;
    return readResponse(packet_buffer, sizeof(packet_buffer), PN532_WAITTIME_1K);  // return response status code
}


/****************************************************
 *                                                  *
 *               ISO14443A Functions                *
 *                                                  *
 ****************************************************/

/**
 * readPassiveTargetID
 * ----------
 * @param  cardBaudRate  Baud rate of the card.
 * @param  uid           Pointer to the array that will be populated with the card's UID (up to 7 bytes).
 * @param  uidLength     Pointer to the variable that will hold the length of the card's UID.
 * ----------
 * @return 1 if everything executed properly, 0 for an error.
 * ----------
 * @brief Waits for an ISO14443A target to enter the field.
 * ----------
 * Data Sheet: section 7.3.5 InListPassiveTarget (page 115).
 */
uint8_t readPassiveTargetID (uint8_t card_baudrate, uint8_t *uid, uint8_t *uid_length) {
    /*-- prepare command --*/
    packet_buffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
    packet_buffer[1] = 1;                              // max 1 card
    packet_buffer[2] = card_baudrate;                  // read datasheet to see card baudrate setting
    
    /*-- write command and read response --*/
    if (!writeCommand(packet_buffer, 3)) return 0;     // return 0 for error
    if (readResponse(packet_buffer, sizeof(packet_buffer), PN532_WAITTIME_1K) < 0) return 0;
    
    /*
     *         ISO14443A Card Formatt
     *  -------------------------------------
     *   byte          |   Description
     *  -------------  |  -------------------
     *   b0            |   Tags Found
     *   b1            |   Tag Number
     *   b2..3         |   SENS_RES
     *   b4            |   SEL_RES
     *   b5            |   NFCID Length
     *   b6..NFCIDLen  |   NFCID
     */
    
    /*-- authenticate and save data --*/
    /* byte 0 */
    if (packet_buffer[0] != 1) return 0;               // return 0, if no tags found
    
    /* byte 5 */
    *uid_length = packet_buffer[5];                    // save uid length
    
    /* UID */
    for (uint8_t i = 0; i < packet_buffer[5]; i++)
        uid[i] = packet_buffer[6 + i];                 // save uid byte
    
    return 1;
}


/****************************************************
 *                                                  *
 *             Mifare Classic functions             *
 *                                                  *
 ****************************************************/

/**
 * mifareClassic_isFirstBlock
 * ----------
 * @param  uiBlock  block number
 * ----------
 * @brief Indicates whether the specified block number is the first block
 *        in the sector (block 0 relative to the current sector).
 */
int mifareClassic_isFirstBlock (uint32_t uiBlock)
{
    /* test if we are in the small or big sectors */
    if (uiBlock < 128) return ((uiBlock) % 4 == 0);
    else return ((uiBlock) % 16 == 0);
}

/**
 * mifareClassic_isTrailerBlock
 * ----------
 * @param  uiBlock  block number
 * ----------
 * @brief Indicates whether the specified block number is the sector trailer
 */
int mifareClassic_isTrailerBlock (uint32_t uiBlock)
{
    /* test if we are in the small or big sectors */
    if (uiBlock < 128) return ((uiBlock + 1) % 4 == 0);
    else return ((uiBlock + 1) % 16 == 0);
}

/**
 * mifareClassic_authenticateBlock
 * ----------
 * @param  uid           Pointer to a byte array containing the card UID.
 * @param  uidLen        The length (in bytes) of the card's UID (Should be 4 for MIFARE Classic).
 * @param  blockNumber   The block number to authenticate.  (0..63 for 1KB cards, and 0..255 for 4KB cards).
 * @param  keyNumber     key type to use during authentication (0 = MIFARE_CMD_AUTH_A, 1 = MIFARE_CMD_AUTH_B).
 * @param  keyData       Pointer to a byte array containing the 6 bytes key value.
 * ----------
 * @return 1 if everything executed properly, 0 for an error.
 * ----------
 * @brief Tries to authenticate a block of memory on a MIFARE card using the INDATAEXCHANGE command.
 * ----------
 * Data Sheet: section 7.3.8 InDataExchange (page 127).
 */
uint8_t mifareClassic_authenticateBlock (
                                         uint8_t *uid,
                                         uint8_t uidLen,
                                         uint32_t blockNumber,
                                         uint8_t keyNumber,
                                         uint8_t *keyData
                                         ) {
    /* prepare the authentication command */
    packet_buffer[0] = PN532_COMMAND_INDATAEXCHANGE;   // data exchange command
    packet_buffer[1] = 1;                              // max card numbers
    packet_buffer[2] = (keyNumber) ? MIFARE_CMD_AUTH_B : MIFARE_CMD_AUTH_A;
    packet_buffer[3] = blockNumber;                    // Block Number (1K = 0..63, 4K = 0..255
    
    memcpy (packet_buffer + 4, keyData, 6);            // skip command and copy keyData
    for (uint8_t i = 0; i < uidLen; i++)
        packet_buffer[10 + i] = uid[i];                // copy uid
    
    /*-- write command and read response --*/
    if (!writeCommand(packet_buffer, 10 + uidLen)) return 0;
    readResponse(packet_buffer, sizeof(packet_buffer), PN532_WAITTIME_1K);
    
    if (packet_buffer[0] != 0x00) return 0;            // this is the status byte, 0x00 means success
    
    return 1;
}

/**
 * mifareClassic_readDataBlock
 * ----------
 * @param  blockNumber   The block number to authenticate.  (0..63 for 1KB cards, and 0..255 for 4KB cards).
 * @param  data          Pointer to the byte array that will hold the retrieved data (if any).
 * ----------
 * @return 1 if everything executed properly, 0 for an error
 * ----------
 * @brief Tries to read an entire 16-bytes data block at the specified block address.
 * ----------
 * Data Sheet: section 7.3.8 InDataExchange (page 127).
 */
uint8_t mifareClassic_readDataBlock (uint8_t blockNumber, uint8_t *data) {
    
    /*-- prepare the command --*/
    packet_buffer[0] = PN532_COMMAND_INDATAEXCHANGE;
    packet_buffer[1] = 1;                              // card number
    packet_buffer[2] = MIFARE_CMD_READ;                // Mifare read command
    packet_buffer[3] = blockNumber;                    // block number (0..63 for 1K, 0..255 for 4K) */
    
    /*-- write command and read response --*/
    if (!writeCommand(packet_buffer, 4)) return 0;
    readResponse(packet_buffer, sizeof(packet_buffer), PN532_WAITTIME_1K);
    
    if (packet_buffer[0] != 0x00) return 0;            // this is the status byte, 0x00 means success
    
    memcpy (data, packet_buffer + 1, 16);              // copy the 16 data bytes to the buffer
    
    return 1;
}


/**
 * mifareClassic_writeDataBlock
 * ----------
 * @param  blockNumber   The block number to authenticate.  (0..63 for 1KB cards, and 0..255 for 4KB cards).
 * @param  data          The byte array that contains the data to write.
 * ----------
 * @return 1 if everything executed properly, 0 for an error
 * ----------
 * @brief Tries to write an entire 16-bytes data block at the specified block address.
 * ----------
 * Data Sheet: section 7.3.8 InDataExchange (page 127).
 */
uint8_t mifareClassic_writeDataBlock (uint8_t blockNumber, uint8_t *data) {
    /*-- prepare the command --*/
    packet_buffer[0] = PN532_COMMAND_INDATAEXCHANGE;
    packet_buffer[1] = 1;                              // card number
    packet_buffer[2] = MIFARE_CMD_WRITE;               // Mifare write command
    packet_buffer[3] = blockNumber;                    // Block Number (0..63 for 1K, 0..255 for 4K)
    memcpy (packet_buffer + 4, data, 16);              // Data Payload
    
    /*-- write command and read response --*/
    if (!writeCommand(packet_buffer, 20)) return 0;
    return readResponse(packet_buffer, sizeof(packet_buffer), PN532_WAITTIME_1K) > 0;
}


/**
 * mifareClassic_formatNDEF
 * ----------
 * @return 1 if everything executed properly, 0 for an error.
 * ----------
 * @brief Formats a Mifare Classic card to store NDEF Records.
 */
uint8_t mifareClassic_formatNDEF (void) {
    uint8_t sectorbuffer1[16] = {0x14, 0x01, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1};
    uint8_t sectorbuffer2[16] = {0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1};
    uint8_t sectorbuffer3[16] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0x78, 0x77, 0x88, 0xC1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    
    if (!(mifareClassic_writeDataBlock (1, sectorbuffer1))) return 0;   // write block 1
    if (!(mifareClassic_writeDataBlock (2, sectorbuffer2))) return 0;   // write block 2
    // Note sectorbuffer3[0..5] 0xA0 0xA1 0xA2 0xA3 0xA4 0xA5 must be used for key A for the MAD sector in NDEF records (sector 0)
    if (!(mifareClassic_writeDataBlock (3, sectorbuffer3))) return 0;   // write block 3: key A and access rights
    
    return 1;
}

/**
 * mifareClassic_writeNDEFURI
 * ----------
 * @param  sectorNumber  The sector that the URI record should be written to (can be 1..15 for a 1K card).
 * @param  uriIdentifier The uri identifier code (0 = none, 0x01 = "http://www.", etc.).
 * @param  url           The uri text to write (max 38 characters).
 * ----------
 * @return 1 if everything executed properly, 0 for an error
 * ----------
 * @brief  Writes an NDEF URI Record to the specified sector (1..15). Note that this function assumes that
 *        the Mifare Classic card is already formatted to work as an "NFC Forum Tag" and uses a MAD1 file
 *        system. You can use the NXP TagWriter app on Android to properly format cards for this.
 */
uint8_t mifareClassic_writeNDEFURI (uint8_t sectorNumber, uint8_t uriIdentifier, const char *url) {
    uint8_t len = strlen(url);                                // Figure out how long the string is
    if ((len < 1) || (len > 38)) return 0;                    // Make sure the URI payload is between 1 and 38 chars
    if ((sectorNumber < 1) || (sectorNumber > 15)) return 0;  // Make sure we're within a 1K limit for the sector number
    
    /*-- setup the sector buffer (w/pre-formatted TLV wrapper and NDEF message) --*/
    uint8_t sectorbuffer1[16] = {0x00, 0x00, 0x03, len + 5, 0xD1, 0x01, len + 1, 0x55, uriIdentifier, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t sectorbuffer2[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t sectorbuffer3[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t sectorbuffer4[16] = {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7, 0x7F, 0x07, 0x88, 0x40, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    if (len <= 6) {
        /* unlikely we'll get a url this short, but why not ... */
        memcpy (sectorbuffer1 + 9, url, len);
        sectorbuffer1[len + 9] = 0xFE;
    } else if (len == 7) {
        /* 0xFE needs to be wrapped around to next block */
        memcpy (sectorbuffer1 + 9, url, len);
        sectorbuffer2[0] = 0xFE;
    } else if ((len > 7) && (len <= 22)) {
        /* url fits in two blocks */
        memcpy (sectorbuffer1 + 9, url, 7);
        memcpy (sectorbuffer2, url + 7, len - 7);
        sectorbuffer2[len - 7] = 0xFE;
    } else if (len == 23) {
        /* 0xFE needs to be wrapped around to final block */
        memcpy (sectorbuffer1 + 9, url, 7);
        memcpy (sectorbuffer2, url + 7, len - 7);
        sectorbuffer3[0] = 0xFE;
    } else {
        /* url fits in three blocks */
        memcpy (sectorbuffer1 + 9, url, 7);
        memcpy (sectorbuffer2, url + 7, 16);
        memcpy (sectorbuffer3, url + 23, len - 23);
        sectorbuffer3[len - 23] = 0xFE;
    }
    
    /*-- write all three blocks back to the card --*/
    if (!(mifareClassic_writeDataBlock (sectorNumber * 4, sectorbuffer1))) return 0;
    if (!(mifareClassic_writeDataBlock ((sectorNumber * 4) + 1, sectorbuffer2))) return 0;
    if (!(mifareClassic_writeDataBlock ((sectorNumber * 4) + 2, sectorbuffer3))) return 0;
    // Note sectorbuffer4[0..5] 0xD3 0xF7 0xD3 0xF7 0xD3 0xF7 must be used for key A in NDEF records
    if (!(mifareClassic_writeDataBlock ((sectorNumber * 4) + 3, sectorbuffer4))) return 0;
    
    return 1;
}


/****************************************************
 *                                                  *
 *           Mifare Ultralight functions            *
 *                                                  *
 ****************************************************/

/**
 * mifareUltralight_readPage
 * ----------
 * @param  page        The page number (0..63 in most cases)
 * @param  buffer      Pointer to the byte array that will hold the
 * ----------
 * @return 1 if everything executed properly, 0 for an error
 * ----------
 * @brief Tries to read an entire 4-bytes page at the specified address.
 */
uint8_t mifareUltralight_readPage (uint8_t page, uint8_t *buffer) {
    if (page > 63) return 0;                           // page value out of range
    
    /*-- prepare the command --*/
    packet_buffer[0] = PN532_COMMAND_INDATAEXCHANGE;
    packet_buffer[1] = 1;                              // card number
    packet_buffer[2] = MIFARE_CMD_READ;                // Mifare read command
    packet_buffer[3] = page;                           // page Number (0..63 in most cases)
    
    /*-- write command and read response --*/
    if (!writeCommand(packet_buffer, 4)) return 0;
    readResponse(packet_buffer, sizeof(packet_buffer), PN532_WAITTIME_1K);
    
    /* authenticate status byte */
    if (packet_buffer[0] == 0x00)
        memcpy (buffer, packet_buffer + 1, 4);         // read a page
    else return 0;                                     // status isn't 0x00, error
    
    return 1;
}

/**
 * mifareUltralight_writePage
 * ----------
 * @param  page     The page number to write into.  (0..63).
 * @param  buffer   The byte array that contains the data to write.
 * ----------
 * @return 1 if everything executed properly, 0 for an error
 * ----------
 * @brief Tries to write an entire 4-bytes data buffer at the specified page address.
 */
uint8_t mifareUltralight_writePage (uint8_t page, uint8_t *buffer) {
    /* Prepare the first command */
    packet_buffer[0] = PN532_COMMAND_INDATAEXCHANGE;
    packet_buffer[1] = 1;                              // card number
    packet_buffer[2] = MIFARE_CMD_WRITE_ULTRALIGHT;    // Mifare Ultralight write command
    packet_buffer[3] = page;                           // page Number (0..63)
    memcpy (packet_buffer + 4, buffer, 4);             // Data Payload
    
    /*-- write command and read response --*/
    if (!writeCommand(packet_buffer, 8)) return 0;
    return readResponse(packet_buffer, sizeof(packet_buffer), PN532_WAITTIME_1K) > 0;
}


/****************************************************
 *                                                  *
 *                  P2P Functions                   *
 *                                                  *
 ****************************************************/
/**
 * P2PInitiatorInit
 * ----------
 * @return 1 if everything executed properly, 0 for an error.
 * ----------
 * @brief Initiate a PN532 as a P2P initiator.
 */
uint8_t P2PInitiatorInit (void) {
    return PN532_inJumpForDEP();
}

/**
 * P2PInitiatorTxRx
 * ----------
 * @param  tx_buff          Buffer for data to be transfered.
 * @param  tx_buff_length   Length of bytes for data to be transfered.
 * @param  rx_buff          Buffer for data to be recieved.
 * ----------
 * @return 0 for error or length of data obtained if everything goes properly.
 * ----------
 * @brief  Let Initiator exchange data with a target.
 */
uint8_t P2PInitiatorTxRx (uint8_t* tx_buff, uint8_t tx_buff_length, uint8_t* rx_buff) {
    
    packet_buffer[0] = PN532_COMMAND_INDATAEXCHANGE;
    packet_buffer[1] = 1;
    
    memcpy(packet_buffer + 2, tx_buff, tx_buff_length);
    
    /*-- write command and read response --*/
    if (!writeCommand(packet_buffer, tx_buff_length + 2)) return 0;
    // data sent
    
    // read response from PN532, return 0 for error
    int LEN = readResponse(packet_buffer, sizeof(packet_buffer), PN532_WAITTIME_1K);
    // if LEN is nefgative, we got error code
    if (LEN < 0) return 0;
    
    // check status byte
    if (packet_buffer[0]) return 0;
    
    /*-- return read data --*/
    memcpy(rx_buff, packet_buffer + 1, LEN - 1);
    return LEN - 1;
}

/**
 * P2PTargetInit
 * ----------
 * @return 1 if everything executed properly, 0 for an error.
 * ----------
 * @brief Initiate a PN532 as a P2P Target.
 */
uint8_t P2PTargetInit () {
    
    uint8_t cmd[] = {
        PN532_COMMAND_TGINITASTARGET,
        0x00,                                                        // Mode
        0x04, 0x00,                                                  // SENS_RES, Mifare S50 (1k)
        0x12, 0x34, 0x56,                                            // NFCID1
        0x40,                                                        // SEL_RES, DEP only mode
        0x01, 0xFE, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7,              // NFCID2t
        0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7,              // PAD
        0xFF, 0xFF,                                                  // System Code
        0xAA, 0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11,  // NFCID3t
        0x00,                                                        // LEN Gt, number of general bytes
        0x00                                                         // LEN Tk, number of historical bytes
    };
    
    return PN532_tgInitAsTarget(cmd, sizeof(cmd), 0); 
}

/**
 * P2PTargetRxTx
 * ----------
 * @param  tx_buff          Buffer for data to be transfered.
 * @param  tx_buff_length   Length of bytes for data to be transfered.
 * @param  rx_buff          Buffer for data to be recieved.
 * @param  rx_buff_length   Length of bytes for data to be recieved.
 * ----------
 * @return 0 for error or length of data obtained if everything goes properly.
 * ----------
 * @brief  Let a target exchange data with an Initiator.
 */
uint16_t P2PTargetRxTx (uint8_t* tx_buff, uint8_t tx_buff_length, uint8_t* rx_buff, uint8_t rx_buff_length) {
    // get data from the initiator
    uint16_t rx_data_length = PN532_tgGetData(rx_buff, rx_buff_length);
    // sent data to the initiator
    if (!PN532_tgSetData(tx_buff, tx_buff_length)) return 0;
    // return the length of data obtained from Initiator
    return rx_data_length;
}
