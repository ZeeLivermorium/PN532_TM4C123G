/*!
 * @file  NDEF_Message.h
 * @brief
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
 * @date Nov 13, 2018
 */

#ifndef __NDEF_MESSAGE_H__
#define __NDEF_MESSAGE_H__

#include "NDEF_Record.h"

#define MAX_NDEF_RECORDS 4

typedef struct _NDEF_Message {
    NDEF_Record records[MAX_NDEF_RECORDS];
    uint32_t record_count;
} NDEF_Message;

/**
 *
 */
uint32_t NDEF_Message_getEncodedSize (NDEF_Message ndef_message);

/**
 *
 */
void NDEF_Message_encode (NDEF_Message ndef_message, uint8_t* data_buffer);

void add_uri_record (NDEF_Message* ndef_message, uint8_t prefix, char* uri, char* payload_buffer);

#endif
