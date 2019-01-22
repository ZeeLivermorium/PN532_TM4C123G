/*!
 * @file  PN532.h
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

#ifndef __PN532_H__
#define __PN532_H__

#include <stdint.h>

#define PN532_WAKEUP                        (0x55)
#define PN532_WAITTIME_1K                   (1000)
#define PN532_WAITTIME_3K                   (3000)
#define PN532_WAITTIME_30K                  (30000)

/* GPIO */
#define PN532_GPIO_VALIDATIONBIT            (0x80)
#define PN532_GPIO_P30                      (0)
#define PN532_GPIO_P31                      (1)
#define PN532_GPIO_P32                      (2)
#define PN532_GPIO_P33                      (3)
#define PN532_GPIO_P34                      (4)
#define PN532_GPIO_P35                      (5)

/* PN532 Commands */
#define PN532_COMMAND_DIAGNOSE              (0x00)
#define PN532_COMMAND_GETFIRMWAREVERSION    (0x02)
#define PN532_COMMAND_GETGENERALSTATUS      (0x04)
#define PN532_COMMAND_READREGISTER          (0x06)
#define PN532_COMMAND_WRITEREGISTER         (0x08)
#define PN532_COMMAND_READGPIO              (0x0C)
#define PN532_COMMAND_WRITEGPIO             (0x0E)
#define PN532_COMMAND_SETSERIALBAUDRATE     (0x10)
#define PN532_COMMAND_SETPARAMETERS         (0x12)
#define PN532_COMMAND_SAMCONFIGURATION      (0x14)
#define PN532_COMMAND_POWERDOWN             (0x16)
#define PN532_COMMAND_RFCONFIGURATION       (0x32)
#define PN532_COMMAND_RFREGULATIONTEST      (0x58)
#define PN532_COMMAND_INJUMPFORDEP          (0x56)
#define PN532_COMMAND_INJUMPFORPSL          (0x46)
#define PN532_COMMAND_INLISTPASSIVETARGET   (0x4A)
#define PN532_COMMAND_INATR                 (0x50)
#define PN532_COMMAND_INPSL                 (0x4E)
#define PN532_COMMAND_INDATAEXCHANGE        (0x40)
#define PN532_COMMAND_INCOMMUNICATETHRU     (0x42)
#define PN532_COMMAND_INDESELECT            (0x44)
#define PN532_COMMAND_INRELEASE             (0x52)
#define PN532_COMMAND_INSELECT              (0x54)
#define PN532_COMMAND_INAUTOPOLL            (0x60)
#define PN532_COMMAND_TGINITASTARGET        (0x8C)
#define PN532_COMMAND_TGSETGENERALBYTES     (0x92)
#define PN532_COMMAND_TGGETDATA             (0x86)
#define PN532_COMMAND_TGSETDATA             (0x8E)
#define PN532_COMMAND_TGSETMETADATA         (0x94)
#define PN532_COMMAND_TGGETINITIATORCOMMAND (0x88)
#define PN532_COMMAND_TGRESPONSETOINITIATOR (0x90)
#define PN532_COMMAND_TGGETTARGETSTATUS     (0x8A)

#define PN532_RESPONSE_INDATAEXCHANGE       (0x41)
#define PN532_RESPONSE_INLISTPASSIVETARGET  (0x4B)

#define PN532_MIFARE_ISO14443A              (0x00)

/* Mifare Commands */
#define MIFARE_CMD_AUTH_A                   (0x60)
#define MIFARE_CMD_AUTH_B                   (0x61)
#define MIFARE_CMD_READ                     (0x30)
#define MIFARE_CMD_WRITE                    (0xA0)
#define MIFARE_CMD_WRITE_ULTRALIGHT         (0xA2)
#define MIFARE_CMD_TRANSFER                 (0xB0)
#define MIFARE_CMD_DECREMENT                (0xC0)
#define MIFARE_CMD_INCREMENT                (0xC1)
#define MIFARE_CMD_STORE                    (0xC2)

/* FeliCa Commands */
#define FELICA_CMD_POLLING                  (0x00)
#define FELICA_CMD_REQUEST_SERVICE          (0x02)
#define FELICA_CMD_REQUEST_RESPONSE         (0x04)
#define FELICA_CMD_READ_WITHOUT_ENCRYPTION  (0x06)
#define FELICA_CMD_WRITE_WITHOUT_ENCRYPTION (0x08)
#define FELICA_CMD_REQUEST_SYSTEM_CODE      (0x0C)

/* FeliCa consts */
#define FELICA_READ_MAX_SERVICE_NUM         16
#define FELICA_READ_MAX_BLOCK_NUM           12 // for typical FeliCa card
#define FELICA_WRITE_MAX_SERVICE_NUM        16
#define FELICA_WRITE_MAX_BLOCK_NUM          10 // for typical FeliCa card
#define FELICA_REQ_SERVICE_MAX_NODE_NUM     32

/* Prefixes(type) for NDEF Records */
#define NDEF_URIPREFIX_NONE                 (0x00)
#define NDEF_URIPREFIX_HTTP_WWWDOT          (0x01)
#define NDEF_URIPREFIX_HTTPS_WWWDOT         (0x02)
#define NDEF_URIPREFIX_HTTP                 (0x03)
#define NDEF_URIPREFIX_HTTPS                (0x04)
#define NDEF_URIPREFIX_TEL                  (0x05)
#define NDEF_URIPREFIX_MAILTO               (0x06)
#define NDEF_URIPREFIX_FTP_ANONAT           (0x07)
#define NDEF_URIPREFIX_FTP_FTPDOT           (0x08)
#define NDEF_URIPREFIX_FTPS                 (0x09)
#define NDEF_URIPREFIX_SFTP                 (0x0A)
#define NDEF_URIPREFIX_SMB                  (0x0B)
#define NDEF_URIPREFIX_NFS                  (0x0C)
#define NDEF_URIPREFIX_FTP                  (0x0D)
#define NDEF_URIPREFIX_DAV                  (0x0E)
#define NDEF_URIPREFIX_NEWS                 (0x0F)
#define NDEF_URIPREFIX_TELNET               (0x10)
#define NDEF_URIPREFIX_IMAP                 (0x11)
#define NDEF_URIPREFIX_RTSP                 (0x12)
#define NDEF_URIPREFIX_URN                  (0x13)
#define NDEF_URIPREFIX_POP                  (0x14)
#define NDEF_URIPREFIX_SIP                  (0x15)
#define NDEF_URIPREFIX_SIPS                 (0x16)
#define NDEF_URIPREFIX_TFTP                 (0x17)
#define NDEF_URIPREFIX_BTSPP                (0x18)
#define NDEF_URIPREFIX_BTL2CAP              (0x19)
#define NDEF_URIPREFIX_BTGOEP               (0x1A)
#define NDEF_URIPREFIX_TCPOBEX              (0x1B)
#define NDEF_URIPREFIX_IRDAOBEX             (0x1C)
#define NDEF_URIPREFIX_FILE                 (0x1D)
#define NDEF_URIPREFIX_URN_EPC_ID           (0x1E)
#define NDEF_URIPREFIX_URN_EPC_TAG          (0x1F)
#define NDEF_URIPREFIX_URN_EPC_PAT          (0x20)
#define NDEF_URIPREFIX_URN_EPC_RAW          (0x21)
#define NDEF_URIPREFIX_URN_EPC              (0x22)
#define NDEF_URIPREFIX_URN_NFC              (0x23)


/****************************************************
 *                                                  *
 *                   Initializer                    *
 *                                                  *
 ****************************************************/

/**
 * PN532_SSI_Init
 * ----------
 * Discription: initialize SSI communication for PN532 Module.
 */
void PN532_Init(void);

// void PN532_I2C_Init(void);

// void PN532_HSU_Init(void);

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
uint32_t PN532_getFirmwareVersion(void);

/**
 * PN532_SAMConfiguration
 * ----------
 * @return -1 for an error.
 * ----------
 * @brief Configures the SAM (Secure Access Module).
 * ----------
 * Data Sheet: section 7.2.10 SAMConfiguration (page 89).
 */
int8_t PN532_SAMConfiguration(void);

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
uint8_t PN532_inJumpForDEP(void);

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
int16_t PN532_inRelease(const uint8_t relevant_target);

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
int8_t PN532_tgInitAsTarget(uint8_t* cmd, uint8_t cmd_length, uint16_t wait_time);

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
int16_t PN532_tgGetData(uint8_t *data_buff, uint8_t buff_length);

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
uint8_t PN532_tgSetData(const uint8_t* data, uint8_t length);

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
int8_t setPassiveActivationRetries(uint8_t maxRetries);


/****************************************************
 *                                                  *
 *               ISO14443A Functions                *
 *                                                  *
 ****************************************************/
// int inListPassiveTarget(void);

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
uint8_t readPassiveTargetID (uint8_t card_baudrate, uint8_t * uid, uint8_t * uid_length);

// int inDataExchange(uint8_t *send, uint8_t sendLength, uint8_t *response, uint8_t *responseLength);

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
int mifareClassic_isFirstBlock (uint32_t uiBlock);

/**
 * mifareClassic_isTrailerBlock
 * ----------
 * @param  uiBlock  block number
 * ----------
 * @brief Indicates whether the specified block number is the sector trailer
 */
int mifareClassic_isTrailerBlock (uint32_t uiBlock);

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
                                         );

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
uint8_t mifareClassic_readDataBlock (uint8_t blockNumber, uint8_t *data);

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
uint8_t mifareClassic_writeDataBlock (uint8_t blockNumber, uint8_t *data);

/**
 * mifareClassic_formatNDEF
 * ----------
 * @return 1 if everything executed properly, 0 for an error.
 * ----------
 * @brief Formats a Mifare Classic card to store NDEF Records.
 */
uint8_t mifareClassic_formatNDEF (void);

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
uint8_t mifareClassic_writeNDEFURI (uint8_t sectorNumber, uint8_t uriIdentifier, const char *url);

///****************************************************
// *                                                  *
// *           Mifare Ultralight functions            *
// *                                                  *
// ****************************************************/
//
//uint8_t mifareultralight_ReadPage (uint8_t page, uint8_t *buffer);
//
//uint8_t mifareultralight_WritePage (uint8_t page, uint8_t *buffer);

///****************************************************
// *                                                  *
// *                 FeliCa Functions                 *
// *                                                  *
// ****************************************************/
//
//int8_t felica_Polling(
//                      uint16_t systemCode,
//                      uint8_t requestCode,
//                      uint8_t *idm,
//                      uint8_t *pmm,
//                      uint16_t *systemCodeResponse,
//                      uint16_t timeout
//                      );
//
//int8_t felica_SendCommand (
//                           const uint8_t * command,
//                           uint8_t commandlength,
//                           uint8_t * response,
//                           uint8_t * responseLength
//                           );
//
//int8_t felica_RequestService(uint8_t numNode, uint16_t *nodeCodeList, uint16_t *keyVersions);
//
//int8_t felica_RequestResponse(uint8_t *mode);
//
//int8_t felica_ReadWithoutEncryption (
//                                     uint8_t numService,
//                                     const uint16_t *serviceCodeList,
//                                     uint8_t numBlock,
//                                     const uint16_t *blockList,
//                                     uint8_t blockData[][16]
//                                     );
//
//int8_t felica_WriteWithoutEncryption (
//                                      uint8_t numService,
//                                      const uint16_t *serviceCodeList,
//                                      uint8_t numBlock,
//                                      const uint16_t *blockList,
//                                      uint8_t blockData[][16]
//                                      );
//
//int8_t felica_RequestSystemCode(uint8_t *numSystemCode, uint16_t *systemCodeList);
//
//int8_t felica_Release(void);

/****************************************************
 *                                                  *
 *                  P2P Functions                   *
 *                                                  *
 ****************************************************/

/**
 * P2PInitiatorInit
 * ----------
 * @return 1 if everything executed properly, 0 for an error
 * ----------
 * @brief Initiate a PN532 as a P2P initiator.
 */
uint8_t P2PInitiatorInit (void);

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
uint8_t P2PInitiatorTxRx (uint8_t* tx_buff, uint8_t tx_buff_length, uint8_t* rx_buff);

/**
 * P2PTargetInit
 * ----------
 * @return 1 if everything executed properly, 0 for an error
 * ----------
 * @brief Initiate a PN532 as a P2P Target.
 */
uint8_t P2PTargetInit (void);

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
uint16_t P2PTargetRxTx (uint8_t* tx_buff, uint8_t tx_buff_length, uint8_t* rx_buff, uint8_t rx_buff_length);


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
void delay(uint32_t N);

#endif

