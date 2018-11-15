/*!
 * @file PN532_SSI.C
 * @brief SSI communication implementation for PN532.
 * ----------
 * Adapted code from Seeed Studio PN532 driver for Arduino.
 * You can find the Seeed Studio PN532 driver here: https://github.com/Seeed-Studio/PN532
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
 * @date Apr 14, 2018
 */


#include "PN532_Setting.h"

#ifdef PN532_SSI

#include <stdint.h>
#include <string.h>
#include "SSI.h"
#include "PN532_SSI.h"

/*
 *  SSI0 A Conncection | SSI1 D Conncection | SSI1 F Conncection | SSI2 B Conncection | SSI3 D Conncection
 *  ------------------ | ------------------ | ------------------ | ------------------ | ------------------
 *  SCK  --------- PA2 | SCK  --------- PD0 | SCK  --------- PF2 | SCK  --------- PB4 | SCK  --------- PD0
 *  SS   --------- PA3 | SS   --------- PD1 | SS   --------- PF3 | SS   --------- PB5 | SS   --------- PD1
 *  MISO --------- PA4 | MISO --------- PD2 | MISO --------- PF0 | MISO --------- PB6 | MISO --------- PD2
 *  MOSI --------- PA5 | MOSI --------- PD3 | MOSI --------- PF1 | MOSI --------- PB7 | MOSI --------- PD3
 */

/****************************************************
 *                                                  *
 *                    Properties                    *
 *                                                  *
 ****************************************************/

 uint8_t command;                 // variable to hold command sent
 const uint8_t ACK_frame[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};

/****************************************************
 *                                                  *
 *                 Helper Functions                 *
 *                                                  *
 ****************************************************/

/**
 * delay
 * ----------
 * Description: delay N time unit
 */
void delay(uint32_t N) {
    for(int n = 0; n < N; n++)                         // N time unitss
        for(int msec = 10000; msec > 0; msec--);       // 1 time unit
}

/****************************************************
 *                                                  *
 *                   Initializers                   *
 *                                                  *
 ****************************************************/

/**
 * PN532_SSI_Init
 * ----------
 * @brief initialize a SSI module with corresponding setting parameters.
 */
void PN532_SSI_Init (void) {
    
    SSI_Init();
    /*-- Wake Up PN532 --*/
    SS_LOW();
    delay(2);
    SS_HIGH();
}



/****************************************************
 *                                                  *
 *                Internal Functions                *
 *                                                  *
 ****************************************************/

/**
 * isReadyForResponse
 * ----------
 */
int isReadyForResponse(void) {
    SS_LOW();
    
    SSI_write(reverseBitOrder(PN532_SPI_STATREAD));           // write SPI starting command to PN532 module
    uint8_t status = reverseBitOrder(SSI_read());             // read response from PN532
    
    delay(1);                                // give time for the last read to finish
    SS_HIGH();
    
    return status == PN532_SPI_READY;        // check if PN532 is ready and return the result
}

/**
 * waitToBeReadyForResponset
 * ----------
 *
 *
 */
int waitToBeReadyForResponse(uint16_t wait_time) {
    while (!isReadyForResponse()) {
        delay(1);
        wait_time--;
        if (wait_time == 0) return 0;
    }
    return 1;
}

/**
 * readACK
 * ----------
 * see NXP PN532 data sheet page 30 for ACK frame
 */
int8_t readACK(void) {
    uint8_t ACK_buffer[6];                   // buffer for ACK signal
    
    SS_LOW();                                // start
    delay(1);                                // wake up PN532
    
    SSI_write(reverseBitOrder(PN532_SPI_DATAREAD));               // tell PN532 the host about to read data
    for (int i = 0; i < 6; i++)
        ACK_buffer[i] = reverseBitOrder(SSI_read());              // read ACK frame
    
    delay(1);                                // give time for the last write to finish
    SS_HIGH();                               // end
    
    return memcmp(ACK_buffer, ACK_frame, 6); // check ACK frame and return
}

/**
 *
 * data sheet page 28
 */
void writeFrame(uint8_t *cmd, uint8_t cmd_length) {
    SS_LOW();
    delay(1);
    
    SSI_write(reverseBitOrder(PN532_SPI_DATAWRITE));          // tell PN532 the host about to write data
    SSI_write(reverseBitOrder(PN532_PREAMBLE));               // write PREAMBLE
    SSI_write(reverseBitOrder(PN532_STARTCODE1));             // write first byte of START CODE
    SSI_write(reverseBitOrder(PN532_STARTCODE2));             // write second byte of START CODE
    
    cmd_length++;                                             // length of data field: TFI + DATA
    SSI_write(reverseBitOrder(cmd_length));                   // write command length to LEN
    SSI_write(reverseBitOrder(~cmd_length + 1));              // write the 2's complement of command length to LCS
    SSI_write(reverseBitOrder(PN532_HOSTTOPN532));            // a frame from the host controller to the PN532
    
    uint8_t DCS = PN532_HOSTTOPN532;                          // data checksum, see datasheet how it is used
    
    for (uint8_t i = 0; i < cmd_length - 1; i++) {
        SSI_write(reverseBitOrder(cmd[i]));                   // write data byte
        DCS += cmd[i];                                        // accumulate data checksum
    }
    
    SSI_write(reverseBitOrder(~DCS + 1));                     // write 2's complement of DCS
    SSI_write(reverseBitOrder(PN532_POSTAMBLE));              // write POSTAMBLE
    
    delay(1);                                                 // give time for the last write to finish
    SS_HIGH();
}


/****************************************************
 *                                                  *
 *                     R/W API                      *
 *                                                  *
 ****************************************************/

/**
 * writeCommand
 * ----------
 *
 */
int writeCommand(uint8_t *cmd, uint8_t cmd_length) {
    command = cmd[0];                                  // record command for response verification
    writeFrame(cmd, cmd_length);                       // write command
    if (!waitToBeReadyForResponse(PN532_ACK_WAIT_TIME)) return 0;
    if (readACK()) return 0;                           // read ACK
    return 1;
}

/**
 * readResponse
 * ----------
 * page 28
 *
 */
int16_t readResponse(uint8_t *data_buffer, uint8_t data_length, uint16_t wait_time) {
    if (!waitToBeReadyForResponse(wait_time))
        return PN532_TIMEOUT;                               // return time out error code as result
    
    SS_LOW();
    delay(1);
    
    SSI_write(reverseBitOrder(PN532_SPI_DATAREAD));         // tell PN532 the host about to read data
    
    /* read 1st (byte 0) to 3rd (byte 2) bytes */
    if (reverseBitOrder(SSI_read()) != PN532_PREAMBLE   ||  // first byte should be PREAMBLE
        reverseBitOrder(SSI_read()) != PN532_STARTCODE1 ||  // second byte should be STARTCODE1
        reverseBitOrder(SSI_read()) != PN532_STARTCODE2     // third byte should be STARTCODE2
        ) {
        SS_HIGH();                                          // pull SS high since we are exiting this function
        return PN532_INVALID_FRAME;                         // return invalid frame code as result
    }
    
    /* read 4th and 5th bytes */
    uint8_t LEN = reverseBitOrder(SSI_read());              // LEN: number of bytes in the data field
    uint8_t LCS = reverseBitOrder(SSI_read());              // LCS: Packet Length Checksum
    if ((uint8_t)(LEN + LCS) != 0x00 ) {
        SS_HIGH();                                          // pull SS high since we are exiting this function
        return PN532_INVALID_FRAME;                         // return invalid frame code as result
    }
    
    /* read 6th and 7th bytes */
    uint8_t PD0 = command + 1;                              // PD0 is command code
    if (PN532_PN532TOHOST != reverseBitOrder(SSI_read()) || PD0 != reverseBitOrder(SSI_read())) {
        SS_HIGH();                                          // pull SS high since we are exiting this function
        return PN532_INVALID_FRAME;                         // return invalid frame code as result
    }
    
    /* check buffer size before read actual data */
    LEN -= 2;                                               // subtract TFI and PD0(command) from DATA length
    if (LEN > data_length) {                                // if no enough space, dump bytes for synchronization
        for (uint8_t i = 0; i < LEN; i++) reverseBitOrder(SSI_read());      // dump data
        reverseBitOrder(SSI_read());                                        // dump DCS
        reverseBitOrder(SSI_read());                                        // dump POSTAMBLE
        SS_HIGH();                                          // pull SS high since we are exiting this function
        return PN532_NO_SPACE;                              // return (buffer) no space error code as result
    }
    
    /* read actual data */
    uint8_t SUM = PN532_PN532TOHOST + PD0;                  // SUM: TFI + DATA, DATA = PD0 + PD1 + ... + PDn
    for (uint8_t i = 0; i < LEN; i++) {
        data_buffer[i] = reverseBitOrder(SSI_read());       // get data
        SUM += data_buffer[i];                              // accumulate SUM
    }
    
    /* read data checksum byte */
    uint8_t DCS = reverseBitOrder(SSI_read());
    if ((uint8_t)(SUM + DCS) != 0) {
        SS_HIGH();                                          // pull SS high since we are exiting this function
        return PN532_INVALID_FRAME;                         // proper frame should result in SUM + DCS = 0
    }
    
    /* read POSTAMBLE */
    reverseBitOrder(SSI_read());                            // dump for synchronization
    
    delay(1);                                               // give time for the last write to finish
    SS_HIGH();
    
    return LEN;
}


#endif
