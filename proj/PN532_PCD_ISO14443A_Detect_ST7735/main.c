/*!
 * @file main.c
 * @brief Detect Mifare/ISO14443A card and display UID on ST7735.
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
#include "ST7735.h"
#include "PN532.h"

uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

int main(void) {
    /*-- TM4C123 Init --*/
    PLL_Init(Bus80MHz);                   // bus clock at 80 MHz
    PN532_Init();
    
    /*-- ST7735 Init --*/
    ST7735_InitR(INITR_REDTAB);
    
    ST7735_SetCursor(0, 0);
    ST7735_FillScreen(ST7735_BLACK);
    
    ST7735_OutString("@author: Zee Lv");
    ST7735_OutChar('\n');
    ST7735_OutString("--------------------");
    ST7735_OutChar('\n');
    
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
    if(chipModel/16 < 10) ST7735_OutUDec(chipModel/16);
    /* first char */
    else ST7735_OutChar(chipModel/16 + 55);  // ASCII A - F
    /* second char */
    if(chipModel%16 < 10) ST7735_OutUDec(chipModel%16);
    else ST7735_OutChar(chipModel%16 + 55);  // ASCII A - F
    ST7735_OutString(" - V. ");
    ST7735_OutUDec((firmwareVersion >> 16) & 0xFF);
    ST7735_OutString(".");
    ST7735_OutUDec((firmwareVersion >> 8) & 0xFF);
    ST7735_OutChar('\n');
    ST7735_OutString("--------------------");
    ST7735_OutChar('\n');
    
    setPassiveActivationRetries(0xFF);    // set the max number of retry attempts to read from a card
    PN532_SAMConfiguration();                          // configure board to read RFID tags
    
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
                /* first char */
                if(uid[i]/16 < 10) ST7735_OutUDec(uid[i]/16);
                else ST7735_OutChar(uid[i]/16 + 55);  // ASCII A - F
                /* second char */
                if(uid[i]%16 < 10) ST7735_OutUDec(uid[i]%16);
                else ST7735_OutChar(uid[i]%16 + 55);  // ASCII A - F
            }
            ST7735_OutChar('\n');                     // new line mark the end of this read
            
            delay(1500);                              // PN532(no netflix) and chill before continuing :)
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
        ST7735_SetCursor(0, 4);
    }
}
