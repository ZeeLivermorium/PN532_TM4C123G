/*!
 * @file MifareClassic_Update_NDEF.c
 * @brief Update URI Content of an NDEF formatted Mifare Classic card.
 * ----------
 * Adapted code from elechouse PN532 driver for Arduino.
 * You can find the elechouse PN532 driver here: https://github.com/elechouse/PN532.git
 * ----------
 * Inspired by examples in ValvanoWareTM4C123 by Dr. Jonathan Valvano
 * as well as his book Embedded Systems: Real-Time Interfacing to Arm Cortex-M Microcontrollers
 * You can find ValvanoWareTM4C123 at http://edx-org-utaustinx.s3.amazonaws.com/UT601x/ValvanoWareTM4C123.zip?dl=1
 * You can find his book at https://www.amazon.com/gp/product/1463590156/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1
 * You can find more of his work at http://users.ece.utexas.edu/~valvano/
 * ----------
 * NXP PN532 datasheet: https://www.nxp.com/docs/en/user-guide/141520.pdf
 * ----------
 * For future development and updates, please follow this repository: https://github.com/ZeeLivermorium/PN532_TM4C123
 * ----------
 * @author Zee Livermorium
 * @date Apr 19, 2018
 */

#include <stdint.h>
#include <string.h>
#include "../inc/PLL.h"
#include "../inc/PN532.h"
#include "../inc/UART.h"                         // for serial output
//#include "../inc/LED.h"                          // for debugging LED indication

uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };         // buffer to store the returned UID
uint8_t uidLength;                               // length of the UID (4 or 7 bytes depending on ISO14443A card type)
char* serial_buffer;                             // a buffer pointer for serial reading

// Use the default NDEF keys (these should have been set by mifareClassic_formatNDEF function)
uint8_t keyA[6] = { 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5 };
uint8_t keyB[6] = { 0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7 };

// Note: check out PN532.h for all NDEF prefixes.

const char * URIContent = "github.com/ZeeLivermorium/PN532_4C123";   // for a url
uint8_t URIPrefix = NDEF_URIPREFIX_HTTP_WWWDOT;

//const char * URIContent = "mail@example.com";    // for an email address
//uint8_t URIPrefix = NDEF_URIPREFIX_MAILTO;

//const char * URIContent = "+1 420 420 6969";     // for a phone number
//uint8_t URIPrefix = NDEF_URIPREFIX_TEL;

int main(void) {
    /*-- TM4C123 Init --*/
    PLL_Init(Bus80MHz);                   // bus clock at 80 MHz
    PN532_Init();                         // init and wake up PN532
    UART_Init();                          // UART for serial output
    //    LED_Init();                           // LED for debug
    
    /*-- PN532 Init --*/
    uint32_t firmwareVersion = PN532_getFirmwareVersion();
    
    if (!firmwareVersion) {               // if not able to read version number, quit
        UART_OutString("Did not find PN532 board :(");
        OutCRLF();
        return 0;                         // exit
    }
    
    /* output firmware info */
    OutCRLF();
    UART_OutString("Found PN5");
    UART_OutUHex((firmwareVersion >> 24) & 0xFF);
    OutCRLF();
    UART_OutString("Firmware Version ");
    UART_OutUDec((firmwareVersion >> 16) & 0xFF);
    UART_OutString(".");
    UART_OutUDec((firmwareVersion >> 8) & 0xFF);
    OutCRLF();
    UART_OutString("-------------------------------");
    OutCRLF();
    SAMConfig();                          // configure board to read RFID tags
    
    /*-- loop --*/
    while(1) {
        UART_OutString("Place NDEF formatted Mifare Classic card on the reader to update its content, ");
        OutCRLF();
        UART_OutString("and press [Enter] to continue ...");
        OutCRLF();
        UART_InString(serial_buffer, 0);  // wait for any key to be pressed
        
        if ( readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength) ) {
            UART_OutString("Found a card :) ");
            OutCRLF();
            /* output uid length */
            UART_OutString("UID Length: ");
            UART_OutUDec(uidLength);
            UART_OutString(" bytes.");
            OutCRLF();
            /* output uid */
            UART_OutString("UID: ");
            for (uint8_t i = 0; i < uidLength; i++) {
                UART_OutString(" 0x");
                UART_OutUHex(uid[i]);
            }
            OutCRLF();
            UART_OutString("-------------------------------");
            OutCRLF();
            
            /* make sure it's a Mifare Classic card */
            if(uidLength != 4) {          // not Mifare Classic
                UART_OutString("This doesn't seem to be a Mifare Classic card :(");
                OutCRLF();
                delay(1500);              // PN532(no netflix) and chill before continuing :)
                continue;
            }
            UART_OutString("Found a Mifare Classic card :)");
            OutCRLF();
            
            /* authenticate NDEF format */
            if (!mifareClassic_authenticateBlock (uid, uidLength, 4, 0, keyB)) {
                UART_OutString("Unable to authenticate block 4 ... this is not NDEF formatted :(");
                OutCRLF();
                UART_OutString("*******************************");
                OutCRLF();
                OutCRLF();
                delay(1500);              // PN532(no netflix) and chill before continuing :)
                continue;
            }
            
            UART_OutString("Authentication succeeded (seems to be an NDEF/NFC Forum tag) ...");
            OutCRLF();
            
            /* check URI content length */
            if (strlen(URIContent) > 38)
            {
                /*
                 * the length is also checked in the writeNDEFURI function, but lets
                 * warn users here just in case they change the value and it's bigger
                 * than it should be
                 */
                UART_OutString("URI content length is too long ... must be less than 38 characters");
                OutCRLF();
                UART_OutString("*******************************");
                OutCRLF();
                OutCRLF();
                delay(1500);              // PN532(no netflix) and chill before continuing :)
                continue;
            }
            
            UART_OutString("Updating sector 1 with URI as NDEF Message");
            OutCRLF();
            
            /* try to write an NDEF record to sector 1 */
            if (mifareClassic_writeNDEFURI(1, URIPrefix, URIContent)) {
                UART_OutString("NDEF URI Record written to sector 1");
                OutCRLF();
                UART_OutString("Job Done!");
            }
            else UART_OutString("NDEF Record creation failed! :(");
            
            OutCRLF();
            UART_OutString("*******************************");
            OutCRLF();
            OutCRLF();
            
            delay(1500);                  // PN532(no netflix) and chill before continuing :)
        }
        else {
            UART_OutString("No card is found :( ");
            OutCRLF();
        }
    }
}

