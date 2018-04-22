/*!
 * @file PN532.c
 * @brief PN532 Library.
 * ----------
 * Adapted code from elechouse PN532 driver for Arduino.
 * You can find the elechouse PN532 driver here:
 * https://github.com/elechouse/PN532.git
 * ----------
 * NXP PN532 datasheet: https://www.nxp.com/docs/en/user-guide/141520.pdf
 * ----------
 * For future development and updates, please follow this repository:
 * https://github.com/ZeeLivermorium/PN532_TM4C123
 * ----------
 * @author Zee Livermorium
 * @date Dec 25, 2017
 */

#include <stdint.h>
#include <string.h>
#include "Serial.h"
#include "PN532.h"
#include "PN532_Setting.h"

#if defined SSI
#include "PN532_SSI.h"
#elif defined I2C
// maybe one day
#elif defined HSU
// maybe some day
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
 * PN532_dumpBlock
 * ----------
 * @param  data      Pointer to the data
 * @param  numBytes  Data length in bytes
 * ----------
 * @brief  Prints a hexadecimal value in plain characters, along with
 *         the char equivalents in the following format
 *
 *         00 00 00 00 00 00  ......
 */
void PN532_dumpBlock (const uint8_t *data, const uint32_t length) {
    for (uint8_t i = 0; i < length; i++) {
        if (data[i] < 0x10) Serial_print(" 0");
        else Serial_print(" ");
        Serial_print("%x", data[i]);
    }
    Serial_print("    ");
    for (uint8_t i = 0; i < length; i++) {
        char c = data[i];
        if (c <= 0x1f || c > 0x7f) Serial_print(".");
        else Serial_print("%c", c);
    }
    Serial_println("");
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
    #ifdef SSI
    PN532_SSI_Init();
    #elif defined I2C
    // PN532_I2C_Init(); NOT SUPPORTED
    #elif defined HSU
    // PN532_HSU_Init(); NOT SUPPORTED
    #endif
}


/****************************************************
 *                                                  *
 *                Generic Functions                 *
 *                                                  *
 ****************************************************/

/**
 * PN532_getFirmwareVersion
 * ----------
 * @return 32-bit firmware version number for PN532 or 0 for an error.
 * ----------
 * Data Sheet: section 7.2.2 GetFirmwareVersion (page 73).
 */
uint32_t PN532_getFirmwareVersion(void) {
    /*-- prepare command --*/
    packet_buffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;
    
    /*-- write command and read response --*/
    if (!writeCommand(packet_buffer, 1)) return 0;     // write command to PN532, return 0 means write fail
    if (readResponse(packet_buffer, 12) < 0) return 0; // read response from PN532, return 0 for error
    
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
 * SAMConfig
 * ----------
 * @return -1 for an error.
 * ----------
 * @brief Configures the SAM (Secure Access Module).
 * ----------
 * Data Sheet: section 7.2.10 SAMConfiguration (page 89).
 */
int8_t SAMConfig(void) {
    /*-- prepare command --*/
    packet_buffer[0] = PN532_COMMAND_SAMCONFIGURATION;
    packet_buffer[1] = 0x01;                           // normal mode;
    packet_buffer[2] = 0x14;                           // timeout 50ms * 20 = 1 second
    packet_buffer[3] = 0x01;                           // use IRQ pin!
    
    /*-- write command and read response --*/
    if (!writeCommand(packet_buffer, 4)) return -1;
    return readResponse(packet_buffer, sizeof(packet_buffer));  // return response status code
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
    return readResponse(packet_buffer, sizeof(packet_buffer));  // return response status code
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
    if (readResponse(packet_buffer, sizeof(packet_buffer)) < 0) return 0;
    
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
 * @return 1 if everything executed properly, 0 for an error
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
    readResponse(packet_buffer, sizeof(packet_buffer));

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
    readResponse(packet_buffer, sizeof(packet_buffer));

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
    return readResponse(packet_buffer, sizeof(packet_buffer)) > 0;
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
    readResponse(packet_buffer, sizeof(packet_buffer));
    
    /* authenticate status byte */
    if (packet_buffer[0] == 0x00)
        memcpy (buffer, packet_buffer + 1, 4);         // read a page
    else return 0;                                     // status isn't 0x00, error

    return 1;
}

/**
 * mifareUltralight_readPage
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
    return readResponse(packet_buffer, sizeof(packet_buffer)) > 0;
}



