/*!
 * @file Mifare_UID_ST7735.c
 * @brief Detect Mifare/ISO14443A card and display UID on ST7735.
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
 * @author Zee Livermorium
 * @date Apr 12, 2018
 */

#include <stdint.h>
#include "../inc/PLL.h"
#include "../inc/PN532.h"
#include "../inc/ST7735.h"
// #include "../inc/LED.h"                   // for debugging LED indication

uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

int main(void) {
    /*-- TM4C123 Init --*/
    PLL_Init(Bus80MHz);                   // bus clock at 80 MHz
    PN532_Init();
    ST7735_InitR(INITR_REDTAB);
    // LED_Init();
    
    ST7735_SetCursor(0, 0);
    ST7735_FillScreen(ST7735_BLACK);
    
    /*-- PN532 Init --*/
    uint32_t firmwareVersion = PN532_getFirmwareVersion();
    
    if (!firmwareVersion) {               // if not able to read version number, quit
        ST7735_OutString("PN532 Not Found.");
        ST7735_OutChar('\n');
        return 0;                         // exit
    }
    
    /* output firmware info */
    ST7735_OutString("Found PN5");
    uint8_t chipModel = (firmwareVersion >> 24) & 0xFF;
    switch (chipModel/16) {
        case 10:
            ST7735_OutChar('A');
            break;
        case 11:
            ST7735_OutChar('B');
            break;
        case 12:
            ST7735_OutChar('C');
            break;
        case 13:
            ST7735_OutChar('D');
            break;
        case 14:
            ST7735_OutChar('E');
            break;
        case 15:
            ST7735_OutChar('F');
            break;
        default:
            ST7735_OutUDec(chipModel/16);
    }
    switch (chipModel%16) {
        case 10:
            ST7735_OutChar('A');
            break;
        case 11:
            ST7735_OutChar('B');
            break;
        case 12:
            ST7735_OutChar('C');
            break;
        case 13:
            ST7735_OutChar('D');
            break;
        case 14:
            ST7735_OutChar('E');
            break;
        case 15:
            ST7735_OutChar('F');
            break;
        default:
            ST7735_OutUDec(chipModel%16);
    }
    ST7735_OutString(" - V. ");
    ST7735_OutUDec((firmwareVersion >> 16) & 0xFF);
    ST7735_OutString(".");
    ST7735_OutUDec((firmwareVersion >> 8) & 0xFF);
    ST7735_OutChar('\n');
    ST7735_OutString("--------------------");
    ST7735_OutChar('\n');
    
    setPassiveActivationRetries(0xFF);    // set the max number of retry attempts to read from a card
    SAMConfig();                          // configure board to read RFID tags
    
    /*-- loop --*/
    while(1) {                            // read and process
        int8_t LastAttempt = -1;
        if ( readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength) ) {
            LastAttempt = 1;
            ST7735_OutString("Found Tag :)");
            ST7735_OutChar('\n');
            /* output uid length */
            ST7735_OutString("UID Length: ");
            ST7735_OutUDec(uidLength);
            ST7735_OutString(" bytes.");
            ST7735_OutChar('\n');
            /* output uid */
            ST7735_OutString("UID:");
            for (uint8_t i=0; i < uidLength; i++) {
                if (i == 4) ST7735_OutString("\n    ");
                ST7735_OutString(" x");
                switch (uid[i]/16) {
                    case 10:
                        ST7735_OutChar('A');
                        break;
                    case 11:
                        ST7735_OutChar('B');
                        break;
                    case 12:
                        ST7735_OutChar('C');
                        break;
                    case 13:
                        ST7735_OutChar('D');
                        break;
                    case 14:
                        ST7735_OutChar('E');
                        break;
                    case 15:
                        ST7735_OutChar('F');
                        break;
                    default:
                        ST7735_OutUDec(uid[i]/16);
                }
                switch (uid[i]%16) {
                    case 10:
                        ST7735_OutChar('A');
                        break;
                    case 11:
                        ST7735_OutChar('B');
                        break;
                    case 12:
                        ST7735_OutChar('C');
                        break;
                    case 13:
                        ST7735_OutChar('D');
                        break;
                    case 14:
                        ST7735_OutChar('E');
                        break;
                    case 15:
                        ST7735_OutChar('F');
                        break;
                    default:
                        ST7735_OutUDec(uid[i]%16);
                }
            }
            ST7735_OutChar('\n');                    // new line mark the end of this read
            
            delay(1500);                  // PN532(no netflix) and chill before continuing :)
        }
        else {
            if(LastAttempt) {
                ST7735_OutString("Time Out :( ");
                ST7735_OutChar('\n');
                ST7735_OutString("                    ");
                ST7735_OutChar('\n');
                ST7735_OutString("                    ");
                ST7735_OutChar('\n');
                ST7735_OutString("                    ");
            } else if (LastAttempt == -1) {
                ST7735_OutString("Time Out :( ");
            }
            LastAttempt = 0;
        }
        ST7735_SetCursor(0, 2);
    }
}
