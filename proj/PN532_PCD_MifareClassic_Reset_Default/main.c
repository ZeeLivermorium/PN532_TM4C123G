/*!
 * @file main.c
 * @brief Reset an NDEF formatted Mifare Classic card back to default format.
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
 * @date Apr 16, 2018
 */

#include <stdint.h>
#include <string.h>
#include "PLL.h"
#include "Serial.h"
#include "PN532.h"

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
    Serial_Init();                        // for serial I/O
    
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
    
    PN532_SAMConfiguration();                          // configure board to read RFID tags
    
    /*-- loop --*/
    while(1) {
        Serial_println("Place your NDEF formatted Mifare Classic card on the reader, ");
        Serial_println("and press [Enter] to continue ...");
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
            Serial_println("Reformatting card back default format");
            
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
                    Serial_println("Authentication failed for sector %u", sectorIndex);
                    Serial_println("*******************************");
                    Serial_println("");
                    delay(1500);                  // PN532(no netflix) and chill before continuing :)
                    continue;
                }
                
                /* Step 2: Write to the other blocks */
                if (sectorIndex == 16) {
                    memset(blockBuffer, 0, sizeof(blockBuffer));
                    if (!(mifareClassic_writeDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(sectorIndex)) - 3, blockBuffer))) {
                        Serial_println("Unable to write to sector %u", sectorIndex);
                        Serial_println("*******************************");
                        Serial_println("");
                        delay(1500);              // PN532(no netflix) and chill before continuing :)
                        continue;
                    }
                }
                if ((sectorIndex == 0) || (sectorIndex == 16)) {
                    memset(blockBuffer, 0, sizeof(blockBuffer));
                    if (!(mifareClassic_writeDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(sectorIndex)) - 2, blockBuffer))) {
                        Serial_println("Unable to write to sector %u", sectorIndex);
                        Serial_println("*******************************");
                        Serial_println("");
                        delay(1500);              // PN532(no netflix) and chill before continuing :)
                        continue;
                    }
                }
                else {
                    memset(blockBuffer, 0, sizeof(blockBuffer));
                    if (!(mifareClassic_writeDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(sectorIndex)) - 3, blockBuffer))) {
                        Serial_println("Unable to write to sector %u", sectorIndex);
                        Serial_println("*******************************");
                        Serial_println("");
                        delay(1500);              // PN532(no netflix) and chill before continuing :)
                        continue;
                    }
                    if (!(mifareClassic_writeDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(sectorIndex)) - 2, blockBuffer))) {
                        Serial_println("Unable to write to sector %u", sectorIndex);
                        Serial_println("*******************************");
                        Serial_println("");
                        delay(1500);              // PN532(no netflix) and chill before continuing :)
                        continue;
                    }
                }
                memset(blockBuffer, 0, sizeof(blockBuffer));
                if (!(mifareClassic_writeDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(sectorIndex)) - 1, blockBuffer))) {
                    Serial_println("Unable to write to sector %u", sectorIndex);
                    Serial_println("*******************************");
                    Serial_println("");
                    delay(1500);                  // PN532(no netflix) and chill before continuing :)
                    continue;
                }
                
                /* Step 3: Reset both keys to 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF */
                memcpy(blockBuffer, KEY_DEFAULT_KEYAB, sizeof(KEY_DEFAULT_KEYAB));
                memcpy(blockBuffer + 6, blankAccessBits, sizeof(blankAccessBits));
                blockBuffer[9] = 0x69;
                memcpy(blockBuffer + 10, KEY_DEFAULT_KEYAB, sizeof(KEY_DEFAULT_KEYAB));
                
                /* Step 4: Write the trailer block */
                if (!(mifareClassic_writeDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(sectorIndex)), blockBuffer))) {
                    Serial_println("Unable to write trailer block of sector %u", sectorIndex);
                    Serial_println("*******************************");
                    Serial_println("");
                    delay(1500);                  // PN532(no netflix) and chill before continuing :)
                    continue;
                }
            }
            
            Serial_println("Reformatting Completed :)");
            Serial_println("*******************************");
            Serial_println("");
            delay(1500);                          // PN532(no netflix) and chill before continuing :)
        }
        else Serial_println("No card is found :( ");
    }
}

