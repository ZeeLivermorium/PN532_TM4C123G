/*!
 * @file MifareClassic_Memory_Dump.c
 * @brief Dump data in all memory blocks of a Mifare Classic card to serial output.
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
#include "../inc/UART.h"                         // for serial output
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
        UART_OutString("Place a Mifare Classic card on the reader. Press [Enter] to continue ...");
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
            
            /* go through all 16 sectors, authenticate then dump */
            for (int blockIndex = 0; blockIndex < 64; blockIndex++) {
                /* check if this is a new block so that we can reauthenticate */
                if (mifareClassic_isFirstBlock(blockIndex)) {
                    /* print sector number */
                    UART_OutString("------------------------Sector ");
                    UART_OutUDec(blockIndex/4);
                    UART_OutString("-------------------------");
                    OutCRLF();
                    /* authenticate key B */
                    if (!mifareClassic_authenticateBlock (uid, uidLength, blockIndex, 1, keyB)) {
                        UART_OutString("Authentication error: ");
                        UART_OutString("Block ");
                        UART_OutUDec(blockIndex);
                        UART_OutString(" unable to authenticate");
                        OutCRLF();
                        delay(1500);      // PN532(no netflix) and chill before continuing :)
                        continue;
                    }
                }
                
                /* read and dump the block  */
                if (mifareClassic_readDataBlock(blockIndex, data)) { // read succeed
                    UART_OutString("Block ");
                    UART_OutUDec(blockIndex);
                    if(blockIndex < 10) UART_OutString("  ");
                    else UART_OutChar(' ');
                    UART_OutBlock(data, 16);     // dump data
                } else {  // read fail
                    UART_OutString("Block ");
                    UART_OutUDec(blockIndex);
                    if(blockIndex < 10) UART_OutString("  ");
                    else UART_OutChar(' ');
                    UART_OutString(" unable to read this block");
                    OutCRLF();
                }
            }
            OutCRLF();
        } else {
            UART_OutString("No card is found :( ");
            OutCRLF();
        }
    }
}

