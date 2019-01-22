/*!
 * @file  main.c
 * @brief emulate PN532 as NDEF tag.
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

#include <stdint.h>
#include <string.h>
#include "PLL.h"
#include "Serial.h"
#include "PN532.h"
#include "PN532_cardEmulation.h"
#include "NDEF_Message.h"

uint8_t  ndef_data_buffer[120];
uint16_t ndef_data_length;
uint8_t  uid[3] = { 0x12, 0x34, 0x56 };

int main (void) {
    /*-- TM4C123 Init --*/
    // bus clock at 80 MHz
    PLL_Init(Bus80MHz);
    // serial init for output anything to console
    Serial_Init();
    // NDEF Message struct to hold the NDEF records to be encoded and sent
    NDEF_Message ndef_message;
    // uri to be added
    char* uri = "zeelivermorium.com";
    // size of payload buffer = prefix byte (1) + size of uri + string termination byte (1)
    char payload_buffer[ 1 + strlen(uri) + 1 ];
    // append a new uri record to the ndef message
    add_uri_record (&ndef_message, NDEF_URIPREFIX_HTTP_WWWDOT, uri, payload_buffer);
    // get the size of encoded NDEF message
    uint32_t ndef_message_size = NDEF_Message_getEncodedSize(ndef_message);
    // encode ndef message and save the result to a buffer
    NDEF_Message_encode(ndef_message, ndef_data_buffer);
	  // set the ndef file from the encoded message
    emulation_setNDEFFile(ndef_data_buffer, ndef_message_size);
    // set uid to be sent
    emulation_setUid(uid);
    
    /*-- PN532 Init --*/
    // init communication and wake up PN532
    PN532_Init();
    
    uint32_t firmwareVersion = PN532_getFirmwareVersion();
    // if not able to read version number, quit
    if (!firmwareVersion) {
        Serial_println("Did not find PN532 board :(");
        // exit
        return 0;
    }
    
    /* output firmware info */
    Serial_println("");
    Serial_println("Found PN5%x", (firmwareVersion >> 24) & 0xFF);
    Serial_println("Firmware Version %u.%u", (firmwareVersion >> 16) & 0xFF, (firmwareVersion >> 8) & 0xFF);
    Serial_println("-------------------------------");
    
    PN532_SAMConfiguration();
    
    /*-- loop --*/
    while(1) {
        emulate_card(0);
    }
}

