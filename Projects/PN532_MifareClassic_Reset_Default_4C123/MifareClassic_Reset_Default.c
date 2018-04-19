/*!
 * @file MifareClassic_Format_NDEF.c
 * @brief detect Mifare/ISO14443A card and serial output(UART) the UID.
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
 * @date Apr 16, 2018
 */

#include <stdint.h>
#include <string.h>
#include "../inc/PLL.h"
#include "../inc/PN532.h"
#include "../inc/UART.h"                         // for serial output
//#include "../inc/LED.h"                          // for debugging LED indication

#define NR_SHORTSECTOR          (32)    // Number of short sectors on Mifare 1K/4K
#define NR_LONGSECTOR           (8)     // Number of long sectors on Mifare 4K
#define NR_BLOCK_OF_SHORTSECTOR (4)     // Number of blocks in a short sector
#define NR_BLOCK_OF_LONGSECTOR  (16)    // Number of blocks in a long sector

// Determine the sector trailer block based on sector number
#define BLOCK_NUMBER_OF_SECTOR_TRAILER(sector) (((sector)<NR_SHORTSECTOR)? \
((sector)*NR_BLOCK_OF_SHORTSECTOR + NR_BLOCK_OF_SHORTSECTOR-1):\
(NR_SHORTSECTOR*NR_BLOCK_OF_SHORTSECTOR + (sector-NR_SHORTSECTOR)*NR_BLOCK_OF_LONGSECTOR + NR_BLOCK_OF_LONGSECTOR-1))

// Determine the sector's first block based on the sector number
#define BLOCK_NUMBER_OF_SECTOR_1ST_BLOCK(sector) (((sector)<NR_SHORTSECTOR)? \
((sector)*NR_BLOCK_OF_SHORTSECTOR):\
(NR_SHORTSECTOR*NR_BLOCK_OF_SHORTSECTOR + (sector-NR_SHORTSECTOR)*NR_BLOCK_OF_LONGSECTOR))

uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };         // buffer to store the returned UID
uint8_t uidLength;                               // length of the UID (4 or 7 bytes depending on ISO14443A card type)
char* serial_buffer;                             // a buffer pointer for serial reading
uint8_t blockBuffer[16];
uint8_t blankAccessBits[3] = { 0xff, 0x07, 0x80 };
uint8_t sectorIndex = 0;
uint8_t numOfSector = 16;                        // Assume Mifare Classic 1K for now (16 4-block sectors)

// The default Mifare Classic key
static const uint8_t KEY_DEFAULT_KEYAB[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

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
        UART_OutString("Place your NDEF formatted Mifare Classic card on the reader, ");
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
            UART_OutString("Reformatting card back default format");
            OutCRLF();
            
            /* run through the card sector by sector */
            for (sectorIndex = 0; sectorIndex < numOfSector; sectorIndex++) {
                /* Step 1: Authenticate the current sector using key B 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF */
                if (!mifareClassic_authenticateBlock (
                                                      uid,
                                                      uidLength,
                                                      BLOCK_NUMBER_OF_SECTOR_TRAILER(sectorIndex),
                                                      1,
                                                      (uint8_t *)KEY_DEFAULT_KEYAB)
                    ) {
                    UART_OutString("Authentication failed for sector ");
                    UART_OutUDec(sectorIndex);
                    OutCRLF();
                    UART_OutString("*******************************");
                    OutCRLF();
                    OutCRLF();
                    delay(1500);              // PN532(no netflix) and chill before continuing :)
                    continue;
                }
                
                /* Step 2: Write to the other blocks */
                if (sectorIndex == 16) {
                    memset(blockBuffer, 0, sizeof(blockBuffer));
                    if (!(mifareClassic_writeDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(sectorIndex)) - 3, blockBuffer))) {
                        UART_OutString("Unable to write to sector ");
                        UART_OutUDec(sectorIndex);
                        OutCRLF();
                        UART_OutString("*******************************");
                        OutCRLF();
                        OutCRLF();
                        delay(1500);              // PN532(no netflix) and chill before continuing :)
                        continue;
                    }
                }
                if ((sectorIndex == 0) || (sectorIndex == 16)) {
                    memset(blockBuffer, 0, sizeof(blockBuffer));
                    if (!(mifareClassic_writeDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(sectorIndex)) - 2, blockBuffer))) {
                        UART_OutString("Unable to write to sector ");
                        UART_OutUDec(sectorIndex);
                        OutCRLF();
                        UART_OutString("*******************************");
                        OutCRLF();
                        OutCRLF();
                        delay(1500);              // PN532(no netflix) and chill before continuing :)
                        continue;
                    }
                }
                else {
                    memset(blockBuffer, 0, sizeof(blockBuffer));
                    if (!(mifareClassic_writeDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(sectorIndex)) - 3, blockBuffer))) {
                        UART_OutString("Unable to write to sector ");
                        UART_OutUDec(sectorIndex);
                        OutCRLF();
                        UART_OutString("*******************************");
                        OutCRLF();
                        OutCRLF();
                        delay(1500);              // PN532(no netflix) and chill before continuing :)
                        continue;
                    }
                    if (!(mifareClassic_writeDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(sectorIndex)) - 2, blockBuffer))) {
                        UART_OutString("Unable to write to sector ");
                        UART_OutUDec(sectorIndex);
                        OutCRLF();
                        UART_OutString("*******************************");
                        OutCRLF();
                        OutCRLF();
                        delay(1500);              // PN532(no netflix) and chill before continuing :)
                        continue;
                    }
                }
                memset(blockBuffer, 0, sizeof(blockBuffer));
                if (!(mifareClassic_writeDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(sectorIndex)) - 1, blockBuffer))) {
                    UART_OutString("Unable to write to sector ");
                    UART_OutUDec(sectorIndex);
                    OutCRLF();
                    UART_OutString("*******************************");
                    OutCRLF();
                    OutCRLF();
                    delay(1500);              // PN532(no netflix) and chill before continuing :)
                    continue;
                }
                
                /* Step 3: Reset both keys to 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF */
                memcpy(blockBuffer, KEY_DEFAULT_KEYAB, sizeof(KEY_DEFAULT_KEYAB));
                memcpy(blockBuffer + 6, blankAccessBits, sizeof(blankAccessBits));
                blockBuffer[9] = 0x69;
                memcpy(blockBuffer + 10, KEY_DEFAULT_KEYAB, sizeof(KEY_DEFAULT_KEYAB));
                
                /* Step 4: Write the trailer block */
                if (!(mifareClassic_writeDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(sectorIndex)), blockBuffer))) {
                    UART_OutString("Unable to write trailer block of sector ");
                    UART_OutUDec(sectorIndex);
                    OutCRLF();
                    UART_OutString("*******************************");
                    OutCRLF();
                    OutCRLF();
                    delay(1500);              // PN532(no netflix) and chill before continuing :)
                    continue;
                }
            }
            
            UART_OutString("Reformatting Completed :)");
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

