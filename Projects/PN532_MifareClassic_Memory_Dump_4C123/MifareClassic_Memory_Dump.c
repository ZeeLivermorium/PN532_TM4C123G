/*!
 * @file MifareClassic_Memory_Dump.c
 * @brief Dump data in all memory blocks of a Mifare Classic card to serial output.
 * ----------
 * Adapted code from Seeed Studio PN532 driver for Arduino.
 * You can find the Seeed Studio PN532 driver here: https://github.com/Seeed-Studio/PN532
 * ----------
 * Inspired by examples in ValvanoWareTM4C123 by Dr. Jonathan Valvano
 * as well as his book Embedded Systems: Real-Time Interfacing to Arm Cortex-M Microcontrollers
 * You can find ValvanoWareTM4C123 at http://edx-org-utaustinx.s3.amazonaws.com/UT601x/ValvanoWareTM4C123.zip?dl=1
 * You can find his book at https://www.amazon.com/gp/product/1463590156/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1
 * You can find more of his work at http://users.ece.utexas.edu/~valvano/
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
 * @date Apr 19, 2018
 */

#include <stdint.h>
#include <string.h>
#include "../inc/PLL.h"
#include "../inc/Serial.h"                       // for serial IO
#include "../inc/PN532.h"
//#include "../inc/LED.h"                          // for debugging LED indication

uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };         // buffer to store the returned UID
uint8_t uidLength;                               // length of the UID (4 or 7 bytes depending on ISO14443A card type)
char* serial_buffer;                             // a buffer pointer for serial reading
uint8_t data[16];                                // Array to store block data during reads
uint8_t keyB[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };


int main(void) {
    /*-- TM4C123 Init --*/
    PLL_Init(Bus80MHz);                   // bus clock at 80 MHz
    PN532_Init();                         // init and wake up PN532
    Serial_Init();                        // for serial IO
//    LED_Init();                           // LED for debug
    
    /*-- PN532 Init --*/
    uint32_t firmwareVersion = PN532_getFirmwareVersion();
    
    if (!firmwareVersion) {               // if not able to read version number, quit
        Serial_println("Did not find PN532 board :(");
        return 0;                         // exit
    }
    
    /* output firmware info */
    Serial_println("");
    Serial_println("Found PN5%x", (firmwareVersion >> 24) & 0xFF);
    Serial_println("Firmware Version %u.%u", (firmwareVersion >> 16) & 0xFF, (firmwareVersion >> 8) & 0xFF);
    Serial_println("-------------------------------");
    
    SAMConfig();                          // configure board to read RFID tags
    
    /*-- loop --*/
    while(1) {
        Serial_println("Place a Mifare Classic card on the reader. Press [Enter] to continue ...");
        Serial_getString(serial_buffer, 0);  // wait for any key to be pressed
        
        if ( readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength) ) {
            Serial_println("Found a card :) ");
            Serial_println("UID Length: %u bytes.", uidLength); // output uid length
            /* output uid */
            Serial_print("UID: ");
            for (uint8_t i=0; i < uidLength; i++) Serial_print(" 0x%x", uid[i]);
            Serial_println("");
            Serial_println("-------------------------------");
            
            /* make sure it's a Mifare Classic card */
            if(uidLength != 4) {          // not Mifare Classic
                Serial_println("This doesn't seem to be a Mifare Classic card :(");
                delay(1500);              // PN532(no netflix) and chill before continuing :)
                continue;
            }
            Serial_println("Found a Mifare Classic card :)");
            
            /* go through all 16 sectors, authenticate then dump */
            for (int blockIndex = 0; blockIndex < 64; blockIndex++) {
                /* check if this is a new block so that we can reauthenticate */
                if (mifareClassic_isFirstBlock(blockIndex)) {
                    /* print sector number */
                    Serial_println("------------------------Sector %u-------------------------", blockIndex/4);
                    /* authenticate key B */
                    if (!mifareClassic_authenticateBlock (uid, uidLength, blockIndex, 1, keyB)) {
                        Serial_println("Authentication error: Block %u unable to authenticate", blockIndex);
                        delay(1500);      // PN532(no netflix) and chill before continuing :)
                        continue;
                    }
                }
                
                /* read and dump the block  */
                if (mifareClassic_readDataBlock(blockIndex, data)) { // read succeed
                    Serial_print("Block %u", blockIndex);
                    if(blockIndex < 10) Serial_print("  ");
                    else Serial_print(" ");
                    Serial_PutHexAndASCII(data, 16);     // dump data
                } else {  // read fail
                    Serial_print("Block %u", blockIndex);
                    if(blockIndex < 10) Serial_print("  ");
                    else Serial_print(" ");
                    Serial_println(" unable to read this block");
                }
            }
            Serial_println("");
        } else Serial_println("No card is found :( ");
    }
}

