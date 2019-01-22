/*!
 * @file  PN532_cardEmulation.h
 * @brief PN532 card emulation APIs.
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
 * @date   Nov 13, 2018
 */

#ifndef __PN532_CARD_EMULATION_H__
#define __PN532_CARD_EMULATION_H__

#include "PN532.h"

#define NDEF_MAX_LENGTH 128  // altough ndef can handle up to 0xfffe in size, arduino cannot.

typedef enum {
    COMMAND_COMPLETE,
    TAG_NOT_FOUND,
    FUNCTION_NOT_SUPPORTED,
    MEMORY_FAILURE,
    END_OF_FILE_BEFORE_REACHED_LE_BYTES
} responseCommand;

/**
 * emulation_setNDEFFile
 * ----------
 * @param  data         data to be stored in the tag.
 * @param  data_length  length of data.
 * ----------
 * @brief  set NDEF data for the tag to be emulated.
 */
void emulation_setNDEFFile (uint8_t* data, const int16_t data_length);

/**
 * emulate_card
 * ----------
 * @param  wait_time  wait time for a response for emulation.
 * ----------
 * @return .
 * ----------
 * @brief  emulate PN532 as a tag.
 */
uint8_t emulate_card (const uint16_t wait_time);

/**
 * emulation_setUid
 * ----------
 * @param  uid  pointer to byte array of length 3 (uid is 4 bytes - first byte is fixed) or zero for uid.
 * ----------
 * @brief  set uid of the tag to be emulated.
 */
void emulation_setUid (uint8_t* uid);

/**
 * set_response
 * ----------
 * @param  cmd     t
 * @param  buffer
 * @param  length
 * @param  offset
 * ----------
 * @brief
 */
void set_response (responseCommand cmd, uint8_t* buffer, uint8_t* length, uint8_t offset);


//void attach(void (*func)(uint8_t *buf, uint16_t length)) {
//    updateNdefCallback = func;
//};

#endif

