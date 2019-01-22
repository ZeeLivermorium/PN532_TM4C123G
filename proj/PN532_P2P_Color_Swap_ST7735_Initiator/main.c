/*!
 * @file  main.c
 * @brief set PN532 as P2P Initiator.
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
 * @date Jan 5, 2019
 */

#include <stdint.h>
#include <string.h>
#include "PLL.h"
#include "button.h"
#include "Serial.h"
#include "ST7735.h"
#include "PN532.h"

uint8_t tx_buff[1] = {3};
uint8_t rx_buff[1];

uint16_t color[] = {
    ST7735_BLACK   ,
    ST7735_BLUE    ,
    ST7735_RED     ,
    ST7735_GREEN   ,
    ST7735_CYAN    ,
    ST7735_MAGENTA ,
    ST7735_YELLOW  ,
    ST7735_WHITE
};

int main (void) {
    //    /*-- TM4C123 Init --*/
    //    PLL_Init(Bus80MHz);                   // bus clock at 80 MHz
    ////    button_init();
    //    PN532_Init();
    //
    //    /*-- ST7735 Init --*/
    //    ST7735_InitR(INITR_REDTAB);
    //
    //    ST7735_SetCursor(0, 0);
    //    ST7735_FillScreen(ST7735_BLACK);
    //
    //    ST7735_OutString("@author: Zee Lv");
    //    ST7735_OutChar('\n');
    //    ST7735_OutString("--------------------");
    //    ST7735_OutChar('\n');
    //
    //    /*-- PN532 Init --*/
    //    if (!PN532_getFirmwareVersion()) {    // if not able to read version number, quit
    //        ST7735_OutString("PN532 Not Found.");
    //        ST7735_OutChar('\n');
    //        return 0;                         // exit
    //    }
    //    // fill initiator screen to green
    //    ST7735_FillScreen(ST7735_GREEN);
    //
    //    PN532_SAMConfiguration();             // configure board to read RFID tags
    //
    //    /*-- loop --*/
    //    while (1) {                           // read and process
    //        // Initiator enters EDP mode
    //        if (P2PInitiatorInit()) {
    //            if (P2PInitiatorTxRx(tx_buff, 1, rx_buff) == 1) {
    //                tx_buff[0] = rx_buff[0];
    //                ST7735_FillScreen(color[tx_buff[0]]);
    //            }
    //        }
    //
    //        delay(2000);
    //
    //    }
    
    
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
    Serial_println("Found PN5%x", (firmwareVersion >> 24) & 0xFF);
    Serial_println("Firmware Version %u.%u", (firmwareVersion >> 16) & 0xFF, (firmwareVersion >> 8) & 0xFF);
    Serial_println("-------------------------------");
    
    PN532_SAMConfiguration();                          // configure board to read RFID tags
    
    /*-- loop --*/
    while(1) {                            // read and process
        if (P2PInitiatorInit()) {
            
            if ( P2PInitiatorTxRx(tx_buff, 1, rx_buff) ) {
                tx_buff[0] = rx_buff[0];
                Serial_println("Swaped rx_buff %d is , now is %x", rx_buff[0], color[tx_buff[0]]);
            }
        }
        
    }
    
    
}

