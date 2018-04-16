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
#include "PN532.h"
#include "PN532_Setting.h"

#if defined SSI
#include "PN532_SSI.h"
#elif defined I2C
// hmmm maybe
#elif defined HSU
// maybe hmmm
#endif

/****************************************************
 *                                                  *
 *                   Initializer                    *
 *                                                  *
 ****************************************************/

/**
 * PN532_Init
 * ----------
 * Discription: initialize communication with PN532. Change settings in PN532_Setting.h.
 */
void PN532_Init(void) {
    #ifdef SSI
    PN532_SSI_Init();
    #elif defined I2C
    // hmmm maybe
    #elif defined HSU
    // maybe hmmm
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
 * mifareclassic_isFirstBlock
 * ----------
 * @param  uiBlock  block number
 * ----------
 * @brief Indicates whether the specified block number is the first block
 *        in the sector (block 0 relative to the current sector).
 */
int mifareclassic_isFirstBlock (uint32_t uiBlock)
{
    // Test if we are in the small or big sectors
    if (uiBlock < 128) return ((uiBlock) % 4 == 0);
    else return ((uiBlock) % 16 == 0);
}

/**
 * mifareclassic_isTrailerBlock
 * ----------
 * @param  uiBlock  block number
 * ----------
 * @brief Indicates whether the specified block number is the sector trailer
 */
int mifareclassic_isTrailerBlock (uint32_t uiBlock)
{
    // Test if we are in the small or big sectors
    if (uiBlock < 128) return ((uiBlock + 1) % 4 == 0);
    else return ((uiBlock + 1) % 16 == 0);
}

///**
// * mifareclassic_AuthenticateBlock
// * ----------
// * @param  uid           Pointer to a byte array containing the card UID
// * @param  uidLen        The length (in bytes) of the card's UID (Should
// *                       be 4 for MIFARE Classic)
// * @param  blockNumber   The block number to authenticate.  (0..63 for
// *                       1KB cards, and 0..255 for 4KB cards).
// * @param  keyNumber     Which key type to use during authentication
// *                       (0 = MIFARE_CMD_AUTH_A, 1 = MIFARE_CMD_AUTH_B)
// * @param  keyData       Pointer to a byte array containing the 6 bytes
// *                       key value
// * ----------
// * @return 1 if everything executed properly, 0 for an error
// * ----------
// * @brief Tries to authenticate a block of memory on a MIFARE card using the
// *        INDATAEXCHANGE command.  See section 7.3.8 of the PN532 User Manual
// *        for more information on sending MIFARE and other commands.
// */
//uint8_t mifareclassic_AuthenticateBlock (uint8_t *uid, uint8_t uidLen, uint32_t blockNumber, uint8_t keyNumber, uint8_t *keyData)
//{
//    uint8_t i;
//
//    // Hang on to the key and uid data
//    memcpy (_key, keyData, 6);
//    memcpy (_uid, uid, uidLen);
//    _uidLen = uidLen;
//
//    // Prepare the authentication command //
//    pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;   /* Data Exchange Header */
//    pn532_packetbuffer[1] = 1;                              /* Max card numbers */
//    pn532_packetbuffer[2] = (keyNumber) ? MIFARE_CMD_AUTH_B : MIFARE_CMD_AUTH_A;
//    pn532_packetbuffer[3] = blockNumber;                    /* Block Number (1K = 0..63, 4K = 0..255 */
//
//    memcpy (pn532_packetbuffer + 4, _key, 6);
//    for (i = 0; i < _uidLen; i++) pn532_packetbuffer[10 + i] = _uid[i];              /* 4 bytes card ID */
//
//
//    if (!writeCommand(pn532_packetbuffer, 10 + _uidLen)) return 0;
//
//    if (readResponse(pn532_packetbuffer, sizeof(pn532_packetbuffer) < 0) return 0;
//
//    // Check if the response is valid and we are authenticated???
//    // for an auth success it should be bytes 5-7: 0xD5 0x41 0x00
//    // Mifare auth error is technically byte 7: 0x14 but anything other and 0x00 is not good
//    if (pn532_packetbuffer[0] != 0x00) return 0;
//
//    return 1;
//}
//
///**
// * mifareclassic_readDataBlock
// * ----------
// * @param  blockNumber   The block number to authenticate.  (0..63 for
// *                       1KB cards, and 0..255 for 4KB cards).
// * @param  data          Pointer to the byte array that will hold the
// *                       retrieved data (if any).
// * ----------
// * @return 1 if everything executed properly, 0 for an error
// * ----------
// * @brief Tries to read an entire 16-bytes data block at the specified block address.
// */
//uint8_t mifareclassic_readDataBlock (uint8_t blockNumber, uint8_t *data)
//{
//    DMSG("Trying to read 16 bytes from block ");
//    DMSG_INT(blockNumber);
//
//    /* Prepare the command */
//    pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
//    pn532_packetbuffer[1] = 1;                      /* Card number */
//    pn532_packetbuffer[2] = MIFARE_CMD_READ;        /* Mifare Read command = 0x30 */
//    pn532_packetbuffer[3] = blockNumber;            /* Block Number (0..63 for 1K, 0..255 for 4K) */
//
//    /* Send the command */
//    if (HAL(writeCommand)(pn532_packetbuffer, 4)) {
//        return 0;
//    }
//
//    /* Read the response packet */
//    HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer));
//
//    /* If byte 8 isn't 0x00 we probably have an error */
//    if (pn532_packetbuffer[0] != 0x00) return 0;
//
//
//    /* Copy the 16 data bytes to the output buffer        */
//    /* Block content starts at byte 9 of a valid response */
//    memcpy (data, pn532_packetbuffer + 1, 16);
//
//    return 1;
//}




