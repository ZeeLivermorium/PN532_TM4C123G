/*
 * File: ISO14443A_Card_Detect.c
 * Detect ISO14443A Card.
 * ----------
 * Adapted code from elechouse PN532 driver for Arduino.
 * You can find the elechouse PN532 driver here:
 * https://github.com/elechouse/PN532.git
 * ----------
 * Inspired by examples in ValvanoWareTM4C123 by Dr. Jonathan Valvano
 * as well as his book Embedded Systems: Real-Time Interfacing to Arm Cortex-M Microcontrollers
 * You can find ValvanoWareTM4C123 at http://edx-org-utaustinx.s3.amazonaws.com/UT601x/ValvanoWareTM4C123.zip?dl=1
 * You can find his book at https://www.amazon.com/gp/product/1463590156/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1
 * You can find more of his work at http://users.ece.utexas.edu/~valvano/
 * ----------
 * NXP PN532 datasheet: https://www.nxp.com/docs/en/user-guide/141520.pdf
 * ----------
 * For future development and updates, please follow this repository:
 * https://github.com/ZeeLivermorium/PN532_TM4C123G
 * ----------
 * Zee Livermorium
 * Apr 12, 2018
 */

#include <stdint.h>
#include <stdio.h>
#include "PLL.h"
#include "UART.h"                         // for serial output
#include "../inc/PN532_TM4C123.h"         // put this in the right path accordingly
#include "../inc/tm4c123gh6pm.h"          // put this in the right path accordingly

uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

int main(void) {
    /* TM4C123G Init */
    PLL_Init(Bus80MHz);                   // bus clock at 80 MHz
    UART_Init();                          // UART for serial output
    PN532_SSI_Init();                     // SSI communication init
    
    /* PN532 Init */
    uint32_t firmwareVersion = PN532_getFirmwareVersion();
    
    if (!firmwareVersion) {               // if not able to read version number, quit
        UART_OutString("Did not find PN532 board :(");
        return 0;                         // exit
    }

    // output firmware info
    UART_OutString("Found PN5");
    UART_OutUHex((firmwareVersion >> 24) & 0xFF);
    UART_OutString("\nFirmware Version ");
    UART_OutUDec((firmwareVersion >> 16) & 0xFF);
    UART_OutString(".");
    UART_OutUDec((firmwareVersion >> 8) & 0xFF);
    UART_OutString("\n--------------------");
    
    setPassiveActivationRetries(0xFF);    // set the max number of retry attempts to read from a card
    SAMConfig();                          // configure board to read RFID tags

    /* loop */
    while(1) {                            // read and process
        if ( readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength) ) {
            UART_OutString("Found a card :) \n");
            // output uid length
            UART_OutString("UID Length: ");
            UART_OutUDec(uidLength);
            UART_OutString(" bytes.\n");
            // output uid
            UART_OutString("UID: ");
            for (uint8_t i=0; i < uidLength; i++) {
                UART_OutString(" 0x");
                UART_OutUHex(uid[i]);
            }
            UART_OutString("\n");         // new line mark the end of this read
            
            delay(1000);                  // chill for a second before continuing
        }
        else UART_OutString("Timed out waiting for a card :( \n");
    }
}
