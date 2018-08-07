/*!
 * @file ISO14443A_Detect.c
 * @brief Detect Mifare/ISO14443A card and serial output(UART) the UID.
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
 * @date Apr 12, 2018
 */

#include <stdint.h>
#include "PLL.h"
#include "Serial.h"       
#include "PN532.h"

uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

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
    
    setPassiveActivationRetries(0xFF);    // set the max number of retry attempts to read from a card
    SAMConfig();                          // configure board to read RFID tags
    
    /*-- loop --*/
    while(1) {                            // read and process
        if ( readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength) ) {
            Serial_println("Found a card :) ");
            Serial_println("UID Length: %u bytes.", uidLength); // output uid length
            /* output uid */
            Serial_print("UID: ");
            for (uint8_t i=0; i < uidLength; i++) Serial_print(" 0x%x", uid[i]);
            
            Serial_println("");           // new line mark the end of this read
            delay(1500);                  // PN532(no netflix) and chill before continuing :)
        }
        else Serial_println("Timed out waiting for a card :( ");
    }
}
