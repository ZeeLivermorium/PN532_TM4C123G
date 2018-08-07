/*!
 * @file PN532_I2C.C
 * @brief I2C communication implementation for TM4C123 to PN532.
 * ----------
 * Adapted code from Seeed Studio PN532 driver for Arduino.
 * You can find the Seeed Studio PN532 driver here: https://github.com/Seeed-Studio/PN532
 * ----------
 * Low level SSI interface functions are inspired by examples in ValvanoWareTM4C123 by Dr. Jonathan Valvano
 * as well as his book Embedded Systems: Real-Time Interfacing to Arm Cortex-M Microcontrollers
 * You can find ValvanoWareTM4C123 at http://edx-org-utaustinx.s3.amazonaws.com/UT601x/ValvanoWareTM4C123.zip?dl=1
 * You can find his book at https://www.amazon.com/gp/product/1463590156/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1
 * You can find more of his work at http://users.ece.utexas.edu/~valvano/
 * ----------
 * For future development and updates, please follow this repository: https://github.com/ZeeLivermorium/PN532_TM4C123
 * ----------
 * @author Zee Livermorium
 * @date May 4, 2018
 */

#include "PN532_Setting.h"

#ifdef I2C

#include <stdint.h>
#include <string.h>
#include "PN532_I2C.h"
#include "tm4c123gh6pm.h"

/****************************************************
 *                                                  *
 *                    Properties                    *
 *                                                  *
 ****************************************************/


/****************************************************
 *                                                  *
 *                 Helper Functions                 *
 *                                                  *
 ****************************************************/


/****************************************************
 *                                                  *
 *                   Initializers                   *
 *                                                  *
 ****************************************************/

/**
 * PN532_I2C_Init
 * ----------
 * @brief initialize a I2C module with corresponding setting parameters.
 */
void PN532_I2C_Init (void) {
#ifdef I2C0
    /*-- I2C0 and Port B Activation --*/
    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;                 // enable I2C Module 0 clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;               // enable GPIO Port B clock
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1) == 0) {};  // allow time for activating
    
    /*-- Port B Set Up --*/
    GPIO_PORTB_AFSEL_R |= 0x0C;                            // enable alt function on PB2, 3
    GPIO_PORTB_ODR_R |= 0x08;                              // enable open drain on PB3
    GPIO_PORTB_DEN_R |= 0x0C;                              // enable digital I/O on PB2,3
    GPIO_PORTB_PCTL_R &= ((~GPIO_PCTL_PB2_M) &             // clear bit fields for PB2
                          (~GPIO_PCTL_PB3_M));             // clear bit fields for PB3
    GPIO_PORTB_PCTL_R |= (GPIO_PCTL_PB2_I2C0SCL |          // configure PB2 as I2C0SCL
                          GPIO_PCTL_PB3_I2C0SDA);          // configure PB3 as I2C0SDA
    GPIO_PORTB_AMSEL_R &= ~0x0C;                           // disable analog functionality on PB2, 3
    
    /*-- I2C0 Set Up --*/
    I2C0_MCR_R = I2C_MCR_MFE;                              // master function enable
    I2C0_MTPR_R = 24;                                      // configure for 100 kbps clock
                                                           // 20 * (TPR + 1) * 20ns = 10us, with TPR=24
#endif
#ifdef I2C1
    /*-- I2C1 and Port A Activation --*/
    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R1;                 // enable I2C Module 1 clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;               // enable GPIO Port A clock
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R0) == 0) {};  // allow time for activating
    
    /*-- Port A Set Up --*/
    GPIO_PORTA_AFSEL_R |= 0xC0;                            // enable alt function on PA6, 7
    GPIO_PORTA_ODR_R |= 0x80;                              // enable open drain on PA7
    GPIO_PORTA_DEN_R |= 0xC0;                              // enable digital I/O on PA6, 7
    GPIO_PORTA_PCTL_R &= ((~GPIO_PCTL_PA6_M) &             // clear bit fields for PA6
                          (~GPIO_PCTL_PA7_M));             // clear bit fields for PA7
    GPIO_PORTA_PCTL_R |= (GPIO_PCTL_PA6_I2C1SCL |          // configure PA6 as I2C1SCL
                          GPIO_PCTL_PA7_I2C1SDA);          // configure PA7 as I2C1SDA
    GPIO_PORTA_AMSEL_R &= ~0xC0;                           // disable analog functionality on PA6, 7
    
    /*-- I2C1 Set Up --*/
    I2C1_MCR_R = I2C_MCR_MFE;                              // master function enable
    I2C1_MTPR_R = 24;                                      // configure for 100 kbps clock
                                                           // 20 * (TPR + 1) * 20ns = 10us, with TPR=24
#endif
#ifdef I2C2
    /*-- I2C2 and Port E Activation --*/
    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R2;                 // enable I2C Module 2 clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;               // enable GPIO Port E clock
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R4) == 0) {};  // allow time for activating
    
    /*-- Port E Set Up --*/
    GPIO_PORTE_AFSEL_R |= 0x30;                            // enable alt function on PE4, 5
    GPIO_PORTE_ODR_R |= 0x20;                              // enable open drain on PE5
    GPIO_PORTE_DEN_R |= 0x30;                              // enable digital I/O on PE4, 5
    GPIO_PORTE_PCTL_R &= ((~GPIO_PCTL_PE4_M) &             // clear bit fields for PE4
                          (~GPIO_PCTL_PE5_M));             // clear bit fields for PE5
    GPIO_PORTE_PCTL_R |= (GPIO_PCTL_PE4_I2C2SCL |          // configure PE4 as I2C2SCL
                          GPIO_PCTL_PE5_I2C2SDA);          // configure PE5 as I2C2SDA
    GPIO_PORTE_AMSEL_R &= ~0x30;                           // disable analog functionality on PE4, 5
    
    /*-- I2C2 Set Up --*/
    I2C2_MCR_R = I2C_MCR_MFE;                              // master function enable
    I2C2_MTPR_R = 24;                                      // configure for 100 kbps clock
                                                           // 20 * (TPR + 1) * 20ns = 10us, with TPR=24
#endif
#ifdef I2C3
    /*-- I2C3 and Port D Activation --*/
    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R3;                 // enable I2C Module 3 clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;               // enable GPIO Port D clock
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R3) == 0) {};  // allow time for activating
    
    /*-- Port D Set Up --*/
    GPIO_PORTD_AFSEL_R |= 0x03;                            // enable alt function on PD0, 1
    GPIO_PORTD_ODR_R |= 0x02;                              // enable open drain on PD1
    GPIO_PORTD_DEN_R |= 0x03;                              // enable digital I/O on PD0, 1
    GPIO_PORTD_PCTL_R &= ((~GPIO_PCTL_PD0_M) &             // clear bit fields for PD0
                          (~GPIO_PCTL_PD1_M));             // clear bit fields for PD1
    GPIO_PORTD_PCTL_R |= (GPIO_PCTL_PD0_I2C3SCL |          // configure PD0 as I2C3SCL
                          GPIO_PCTL_PD1_I2C3SDA);          // configure PD1 as I2C3SDA
    GPIO_PORTD_AMSEL_R &= ~0x03;                           // disable analog functionality on PD0, 1
    
    /*-- I2C3 Set Up --*/
    I2C3_MCR_R = I2C_MCR_MFE;                              // master function enable
    I2C3_MTPR_R = 24 ;                                     // configure for 100 kbps clock
                                                           // 20 * (TPR + 1) * 20ns = 10us, with TPR=24
#endif
}

/****************************************************
 *                                                  *
 *                   I/O Functions                  *
 *                                                  *
 ****************************************************/

/**
 * read
 * ----------
 * @return date read from PN532.
 */
static uint8_t read (void) {

}

/**
 * write
 * ----------
 * @param  data  data to be written.
 */
static void write(uint8_t data){

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
static int isReadyForResponse(void) {
    SS_LOW();
    
    write(PN532_SPI_STATREAD);               // write SPI starting command to PN532 module
    uint8_t status = read();                 // read response from PN532
    
    delay(1);                                          // give time for the last read to finish
    SS_HIGH();
    
    return status == PN532_SPI_READY;                  // check if PN532 is ready and return the result
}

/**
 * waitToBeReadyForResponse
 * ----------
 *
 *
 */
static int waitToBeReadyForResponse(uint16_t wait_time) {
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
static int8_t readACK(void) {
    uint8_t ACK_buffer[6];                             // buffer for ACK signal
    
    SS_LOW();                                          // start
    delay(1);                                          // wake up PN532
    
    write(PN532_SPI_DATAREAD);               // tell PN532 the host about to read data
    for (int i = 0; i < 6; i++)
        ACK_buffer[i] = read();              // read ACK frame
    
    delay(1);                                          // give time for the last write to finish
    SS_HIGH();                                         // end
    
    return memcmp(ACK_buffer, ACK_frame, 6);           // check ACK frame and return
}

/**
 *
 * data sheet page 28
 */
static void writeFrame(uint8_t *cmd, uint8_t cmd_length) {
    
    // ------ I2C stuff

    write(PN532_PREAMBLE);                   // write PREAMBLE
    write(PN532_STARTCODE1);                 // write first byte of START CODE
    write(PN532_STARTCODE2);                 // write second byte of START CODE
    
    cmd_length++;                                      // length of data field: TFI + DATA
    write(cmd_length);                       // write command length to LEN
    write(~cmd_length + 1);                  // write the 2's complement of command length to LCS
    write(PN532_HOSTTOPN532);                // a frame from the host controller to the PN532
    
    uint8_t DCS = PN532_HOSTTOPN532;                   // data checksum, see datasheet how it is used
    
    for (uint8_t i = 0; i < cmd_length - 1; i++) {
        write(cmd[i]);                       // write data byte
        DCS += cmd[i];                                 // accumulate data checksum
    }
    
    write(~DCS + 1);                         // write 2's complement of DCS
    write(PN532_POSTAMBLE);                  // write POSTAMBLE
    
    // ------ I2C stuff
    
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
int16_t readResponse(uint8_t *data_buffer, uint8_t data_length) {
    if (!waitToBeReadyForResponse(1000))
        return PN532_TIMEOUT;                          // return time out error code as result
    
    SS_LOW();
    delay(1);
    
    write(PN532_SPI_DATAREAD);               // tell PN532 the host about to read data
    
    /* read 1st (byte 0) to 3rd (byte 2) bytes */
    if (read() != PN532_PREAMBLE   ||        // first byte should be PREAMBLE
        read() != PN532_STARTCODE1 ||        // second byte should be STARTCODE1
        read() != PN532_STARTCODE2           // third byte should be STARTCODE2
        ) {
        SS_HIGH();                                     // pull SS high since we are exiting this function
        return PN532_INVALID_FRAME;                    // return invalid frame code as result
    }
    
    /* read 4th and 5th bytes */
    uint8_t LEN = read();                    // LEN: number of bytes in the data field
    uint8_t LCS = read();                    // LCS: Packet Length Checksum
    if ((uint8_t)(LEN + LCS) != 0x00 ) {
        SS_HIGH();                                     // pull SS high since we are exiting this function
        return PN532_INVALID_FRAME;                    // return invalid frame code as result
    }
    
    /* read 6th and 7th bytes */
    uint8_t PD0 = command + 1;                         // PD0 is command code
    if (PN532_PN532TOHOST != read() || PD0 != read()) {
        SS_HIGH();                                     // pull SS high since we are exiting this function
        return PN532_INVALID_FRAME;                    // return invalid frame code as result
    }
    
    /* check buffer size before read actual data */
    LEN -= 2;                                          // subtract TFI and PD0(command) from DATA length
    if (LEN > data_length) {                           // if no enough space, dump bytes for synchronization
        for (uint8_t i = 0; i < LEN; i++) read();  // dump data
        read();                                    // dump DCS
        read();                                    // dump POSTAMBLE
        SS_HIGH();                                     // pull SS high since we are exiting this function
        return PN532_NO_SPACE;                         // return (buffer) no space error code as result
    }
    
    /* read actual data */
    uint8_t SUM = PN532_PN532TOHOST + PD0;             // SUM: TFI + DATA, DATA = PD0 + PD1 + ... + PDn
    for (uint8_t i = 0; i < LEN; i++) {
        data_buffer[i] = read();             // get data
        SUM += data_buffer[i];                         // accumulate SUM
    }
    
    /* read data checksum byte */
    uint8_t DCS = read();
    if ((uint8_t)(SUM + DCS) != 0) {
        SS_HIGH();                                     // pull SS high since we are exiting this function
        return PN532_INVALID_FRAME;                    // proper frame should result in SUM + DCS = 0
    }
    
    /* read POSTAMBLE */
    read();                                  // dump for synchronization
    
    delay(1);                                          // give time for the last write to finish
    SS_HIGH();
    
    return LEN;
}


#endif
