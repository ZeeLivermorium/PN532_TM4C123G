/*
 * PN532_SSI.C
 * ----------
 * Inspired by examples in ValvanoWareTM4C123 by Dr. Jonathan Valvano
 * as well as his book Embedded Systems: Real-Time Interfacing to Arm Cortex-M Microcontrollers
 * You can find ValvanoWareTM4C123 at http://edx-org-utaustinx.s3.amazonaws.com/UT601x/ValvanoWareTM4C123.zip?dl=1
 * You can find his book at https://www.amazon.com/gp/product/1463590156/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1
 * You can find more of his work at http://users.ece.utexas.edu/~valvano/
 * ----------
 * Zee Livermorium
 * Apr 14, 2018
 */


#include "PN532_Setting.h"

#ifdef SSI

#include <stdint.h>
#include <string.h>
#include "PN532_SSI.h"
#include "../inc/tm4c123gh6pm.h"

/*
 *  SSI0 A Conncection | SSI1 D Conncection | SSI1 F Conncection | SSI2 B Conncection | SSI3 D Conncection
 *  ------------------ | ------------------ | ------------------ | ------------------ | ------------------
 *  SCK  --------- PA2 | SCK  --------- PD0 | SCK  --------- PF0 | SCK  --------- PB4 | SCK  --------- PD0
 *  SS   --------- PA3 | SS   --------- PD1 | SS   --------- PF1 | SS   --------- PB5 | SS   --------- PD1
 *  MISO --------- PA4 | MISO --------- PD2 | MISO --------- PF2 | MISO --------- PB6 | MISO --------- PD2
 *  MOSI --------- PA5 | MOSI --------- PD3 | MOSI --------- PF3 | MOSI --------- PB7 | MOSI --------- PD3
 */

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
static void delay(uint32_t N) {
    for(int n = 0; n < N; n++)                         // N time unitss
        for(int msec = 10000; msec > 0; msec--);       // 1 time unit
}

/**
 * reverseBitOrder
 * ----------
 * Discription: to output in the order of LSB first, we need to reverse all bits.
 */
static uint8_t reverseBitOrder(uint8_t byte) {
    return ((byte & 0x01) << 7) +
           ((byte & 0x02) << 5) +
           ((byte & 0x04) << 3) +
           ((byte & 0x08) << 1) +
           ((byte & 0x10) >> 1) +
           ((byte & 0x20) >> 3) +
           ((byte & 0x40) >> 5) +
           ((byte & 0x80) >> 7);
}

/****************************************************
 *                                                  *
 *                   SS Functions                   *
 *                                                  *
 ****************************************************/

static void SS_HIGH (void) {
#if defined SSI0
    GPIO_PORTA_DATA_R |= 0x08;
#elif defined SSI1D
    GPIO_PORTD_DATA_R |= 0x02;
#elif defined SSI1F
    GPIO_PORTF_DATA_R |= 0x02;
#elif defined SSI2
    GPIO_PORTB_DATA_R |= 0x20;
#elif defined SSI3
    GPIO_PORTD_DATA_R |= 0x02;
#endif
}

static void SS_LOW (void) {
#if defined SSI0
    GPIO_PORTA_DATA_R &= ~0x08;
#elif defined SSI1D
    GPIO_PORTD_DATA_R &= ~0x02;
#elif defined SSI1F
    GPIO_PORTF_DATA_R &= ~0x02;
#elif defined SSI2
    GPIO_PORTB_DATA_R &= ~0x20;
#elif defined SSI3
    GPIO_PORTD_DATA_R &= ~0x02;
#endif
    
}

/****************************************************
 *                                                  *
 *                   Initializers                   *
 *                                                  *
 ****************************************************/

/**
 * PN532_SSI_Init
 * ----------
 * @brief initialize SSI0 on Port A with corresponding setting parameters.
 */
void PN532_SSI_Init (void) {
#if defined SSI0
    /*-- SSI0 and Port A Activation --*/
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R0;                 // enable SSI Module 0 clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;               // enable GPIO Port A clock
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R0) == 0){};    // allow time for activating
    
    /*-- Port A Set Up --*/
    GPIO_PORTA_DIR_R |= 0x08;                              // make PA3 output
    GPIO_PORTA_AFSEL_R |= 0x34;                            // enable alt funct on PA2, 4, 5
    GPIO_PORTA_AFSEL_R &= ~0x08;                           // disable alt funct on PA3
    GPIO_PORTA_PCTL_R &= ((~GPIO_PCTL_PA2_M) &             // clear bit fields for PA2
                          (~GPIO_PCTL_PA3_M) &             // clear bit fields for PA3, PA3 will be used as GPIO
                          (~GPIO_PCTL_PA4_M) &             // clear bit fields for PA4
                          (~GPIO_PCTL_PA5_M));             // clear bit fields for PA5
    GPIO_PORTA_PCTL_R |= (GPIO_PCTL_PA2_SSI0CLK |          // configure PA2 as SSI0CLK
                          GPIO_PCTL_PA4_SSI0RX |           // configure PA4 as SSI0RX
                          GPIO_PCTL_PA5_SSI0TX);           // configure PA5 as SSI0TX
    GPIO_PORTA_AMSEL_R &= ~0x3C;                           // disable analog functionality on PA2-5
    GPIO_PORTA_DEN_R |= 0x3C;                              // enable digital I/O on PA2-5
    
    /*-- SSI0 Set Up --*/
    SSI0_CR1_R &= ~SSI_CR1_SSE;                            // disable SSI0 operation
    SSI0_CR1_R &= ~SSI_CR1_MS;                             // configure SSI0 as master mode
    SSI0_CC_R &= ~SSI_CC_CS_M;
    SSI0_CC_R |= SSI_CC_CS_SYSPLL;
    SSI0_CPSR_R &= ~SSI_CPSR_CPSDVSR_M;                    // clear bit fields for SSI Clock Prescale Divisor
    SSI0_CPSR_R += 40;                                     // /40 clock divisor, 2Mhz
    SSI0_CR0_R &= ~SSI_CR0_SCR_M;                          // clear bit fields for SSI0 Serial Clock Rate, SCR = 0
    SSI0_CR0_R &= ~SSI_CR0_SPO;                            // clear bit fields for SSI0 Serial Clock Polarity, SPO = 0
    SSI0_CR0_R &= ~SSI_CR0_SPH;                            // clear bit fields for SSI0 Serial Clock Phase, SPH = 0
    SSI0_CR0_R &= ~SSI_CR0_FRF_M;                          // clear bit fields for SSI0 Frame Format Select
    SSI0_CR0_R |= SSI_CR0_FRF_MOTO;                        // set frame format to Freescale SPI Frame Format
    SSI0_CR0_R &= ~SSI_CR0_DSS_M;                          // clear bit fields for SSI0 Data Size Select
    SSI0_CR0_R |= SSI_CR0_DSS_8;                           // set SSI data size to 8
    SSI0_CR1_R |= SSI_CR1_SSE;                             // enable SSI operation
    
#elif defined SSI1
    
#elif defined SSI2
    /* SSI2 and Port B Activation */
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;                 // enable SSI Module 2 clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;               // enable GPIO Port B clock
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1) == 0){};    // allow time for activating
    
    /* Port B Set Up */
    GPIO_PORTB_DIR_R |= 0x20;                              // make PB5 output
    GPIO_PORTB_AFSEL_R |= 0xF0;                            // enable alt funct on PB4, 6, 7
    GPIO_PORTB_AFSEL_R &= ~0x20;                           // disable alt funct on PB5
    GPIO_PORTB_PCTL_R &= ((~GPIO_PCTL_PB4_M) &             // clear bit fields for PB4
                          (~GPIO_PCTL_PB5_M) &             // clear bit fields for PB5, PB5 will be used as GPIO
                          (~GPIO_PCTL_PB6_M) &             // clear bit fields for PB6
                          (~GPIO_PCTL_PB7_M));             // clear bit fields for PB7
    GPIO_PORTB_PCTL_R |= (GPIO_PCTL_PB4_SSI2CLK |          // configure PB4 as SSI2CLK
                          GPIO_PCTL_PB6_SSI2RX  |          // configure PB6 as SSI2RX
                          GPIO_PCTL_PB7_SSI2TX);           // configure PB7 as SSI2TX
    GPIO_PORTB_AMSEL_R &= ~0xF0;                           // disable analog functionality on PB4-7
    GPIO_PORTB_DEN_R |= 0xF0;                              // enable digital I/O on PB4-7
    
    /* SSI2 Set Up */
    SSI2_CR1_R &= ~SSI_CR1_SSE;                            // disable SSI operation
    SSI2_CR1_R &= ~SSI_CR1_MS;                             // configure SSI2 as master mode
    SSI2_CPSR_R &= SSI_CPSR_CPSDVSR_M;                     // clear bit fields for SSI Clock Prescale Divisor
    SSI2_CPSR_R += 40;                                     // /40 clock divisor, 2Mhz
    SSI2_CR0_R &= ~SSI_CR0_SCR_M;                          // clear bit fields for SSI2 Serial Clock Rate, SCR = 0
    SSI2_CR0_R &= ~SSI_CR0_SPO;                            // clear bit fields for SSI2 Serial Clock Polarity, SPO = 0
    SSI2_CR0_R &= ~SSI_CR0_SPH;                            // clear bit fields for SSI2 Serial Clock Phase, SPH = 0
    SSI2_CR0_R &= ~SSI_CR0_FRF_M;                          // clear bit fields for SSI2 Frame Format Select
    SSI2_CR0_R |= SSI_CR0_FRF_MOTO;                        // set frame format to Freescale SPI Frame Format
    SSI2_CR0_R &= ~SSI_CR0_DSS_M;                          // clear bit fields for SSI2 Data Size Select
    SSI2_CR0_R |= SSI_CR0_DSS_8;                           // set SSI data size to 8
    SSI2_CR1_R |= SSI_CR1_SSE;                             // enable SSI operation
#elif defined SSI3
    
#endif
    
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
static int isReadyForResponse(void) {
    SS_LOW();
    
    PN532_SSI_write(PN532_SPI_STATREAD);               // write SPI starting command to PN532 module
    uint8_t status = PN532_SSI_read();                 // read response from PN532
    
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
    
    PN532_SSI_write(PN532_SPI_DATAREAD);               // tell PN532 the host about to read data
    for (int i = 0; i < 6; i++)
        ACK_buffer[i] = PN532_SSI_read();              // read ACK frame
    
    delay(1);                                          // give time for the last write to finish
    SS_HIGH();                                         // end
    
    return memcmp(ACK_buffer, ACK_frame, 6);           // check ACK frame and return
}

/**
 *
 * data sheet page 28
 */
static void writeFrame(uint8_t *cmd, uint8_t cmd_length) {
    SS_LOW();
    delay(1);
    
    PN532_SSI_write(PN532_SPI_DATAWRITE);              // tell PN532 the host about to write data
    PN532_SSI_write(PN532_PREAMBLE);                   // write PREAMBLE
    PN532_SSI_write(PN532_STARTCODE1);                 // write first byte of START CODE
    PN532_SSI_write(PN532_STARTCODE2);                 // write second byte of START CODE
    
    cmd_length++;                                      // length of data field: TFI + DATA
    PN532_SSI_write(cmd_length);                       // write command length to LEN
    PN532_SSI_write(~cmd_length + 1);                  // write the 2's complement of command length to LCS
    PN532_SSI_write(PN532_HOSTTOPN532);                // a frame from the host controller to the PN532
    
    uint8_t DCS = PN532_HOSTTOPN532;                   // data checksum, see datasheet how it is used
    
    for (uint8_t i = 0; i < cmd_length - 1; i++) {
        PN532_SSI_write(cmd[i]);                       // write data byte
        DCS += cmd[i];                                 // accumulate data checksum
    }
    
    PN532_SSI_write(~DCS + 1);                         // write 2's complement of DCS
    PN532_SSI_write(PN532_POSTAMBLE);                  // write POSTAMBLE
    
    delay(1);                                          // give time for the last write to finish
    SS_HIGH();
}

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
    
    PN532_SSI_write(PN532_SPI_DATAREAD);               // tell PN532 the host about to read data
    
    /* read 1st to 3rd bytes */
    if (PN532_SSI_read() != PN532_PREAMBLE   ||        // first byte should be PREAMBLE
        PN532_SSI_read() != PN532_STARTCODE1 ||        // second byte should be STARTCODE1
        PN532_SSI_read() != PN532_STARTCODE2           // third byte should be STARTCODE2
        ) {
        SS_HIGH();                                     // pull SS high since we are exiting this function
        return PN532_INVALID_FRAME;                    // return invalid frame code as result
    }
    
    /* read 4th and 5th bytes */
    uint8_t LEN = PN532_SSI_read();                    // LEN: number of bytes in the data field
    uint8_t LCS = PN532_SSI_read();                    // LCS: Packet Length Checksum
    if ((uint8_t)(LEN + LCS) != 0x00 ) {
        SS_HIGH();                                     // pull SS high since we are exiting this function
        return PN532_INVALID_FRAME;                    // return invalid frame code as result
    }
    
    /* read 6th and 7th bytes */
    uint8_t PD0 = command + 1;                         // PD0 is command code
    if (PN532_PN532TOHOST != PN532_SSI_read() || PD0 != PN532_SSI_read()) {
        SS_HIGH();                                     // pull SS high since we are exiting this function
        return PN532_INVALID_FRAME;                    // return invalid frame code as result
    }
    
    /* check buffer size before read actual data */
    LEN -= 2;                                          // subtract TFI and PD0(command) from DATA length
    if (LEN > data_length) {                           // if no enough space, dump bytes for synchronization
        for (uint8_t i = 0; i < LEN; i++) PN532_SSI_read();  // dump data
        PN532_SSI_read();                                    // dump DCS
        PN532_SSI_read();                                    // dump POSTAMBLE
        
        SS_HIGH();                                     // pull SS high since we are exiting this function
        return PN532_NO_SPACE;                         // return (buffer) no space error code as result
    }
    
    /* read actual data */
    uint8_t SUM = PN532_PN532TOHOST + PD0;             // SUM: TFI + DATA, DATA = PD0 + PD1 + ... + PDn
    for (uint8_t i = 0; i < LEN; i++) {
        data_buffer[i] = PN532_SSI_read();             // get data
        SUM += data_buffer[i];                         // accumulate SUM
    }
    
    /* read data checksum byte */
    uint8_t DCS = PN532_SSI_read();
    if ((uint8_t)(SUM + DCS) != 0) {
        SS_HIGH();                                     // pull SS high since we are exiting this function
        return PN532_INVALID_FRAME;                    // proper frame should result in SUM + DCS = 0
    }
    
    /* read POSTAMBLE */
    PN532_SSI_read();                                  // dump for synchronization
    
    delay(1);                                          // give time for the last write to finish
    SS_HIGH();
    
    return LEN;
}

/****************************************************
 *                                                  *
 *                   I/O Functions                  *
 *                                                  *
 ****************************************************/

/**
 * SSI0_Read
 * ----------
 * @return: date read from another device.
 */
static uint8_t PN532_SSI_read (void) {
#if defined SSI0
    while((SSI0_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};     // wait until SSI0 not busy/transmit FIFO empty
    SSI0_DR_R = 0x00;                                     // data out, garbage, just for synchronization
    while((SSI0_SR_R & SSI_SR_RNE) == 0) {};              // wait until response
    return reverseBitOrder(SSI0_DR_R);                    // LSB, reverse bit order after read
#elif defined SSI1
    while((SSI1_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};     // wait until SSI1 not busy/transmit FIFO empty
    SSI1_DR_R = 0x00;                                     // data out, garbage, just for synchronization
    while((SSI1_SR_R & SSI_SR_RNE) == 0) {};              // wait until response
    return reverseBitOrder(SSI1_DR_R);                    // LSB, reverse bit order after read
#elif defined SSI2
    while((SSI2_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};     // wait until SSI2 not busy/transmit FIFO empty
    SSI2_DR_R = 0x00;                                     // data out, garbage, just for synchronization
    while((SSI2_SR_R & SSI_SR_RNE) == 0) {};              // wait until response
    return reverseBitOrder(SSI2_DR_R);                    // LSB, reverse bit order after read
#elif defined SSI3
    while((SSI3_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};     // wait until SSI3 not busy/transmit FIFO empty
    SSI3_DR_R = 0x00;                                     // data out, garbage, just for synchronization
    while((SSI3_SR_R & SSI_SR_RNE) == 0) {};              // wait until response
    return reverseBitOrder(SSI3_DR_R);                    // LSB, reverse bit order after read
#endif
}

/**
 * SSI0_write
 * ----------
 * @param  data  data to be written.
 */
static void PN532_SSI_write(uint8_t data){
#if defined SSI0
    while ((SSI0_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};   // wait until SSI1 not busy/transmit FIFO empty
    SSI0_DR_R = reverseBitOrder(data);                   // LSB, write after reverse bit order
    while ((SSI0_SR_R & SSI_SR_RNE) == 0) {};            // wait until response
    uint16_t sync = SSI0_DR_R;                           // read byte of data, just for synchronization
#elif defined SSI1
    while ((SSI1_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};   // wait until SSI3 not busy/transmit FIFO empty
    SSI1_DR_R = reverseBitOrder(data);                   // LSB, write after reverse bit order
    while ((SSI1_SR_R & SSI_SR_RNE) == 0) {};            // wait until response
    uint16_t sync = SSI1_DR_R;                           // read byte of data, just for synchronization
#elif defined SSI2
    while ((SSI2_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};   // wait until SSI2 not busy/transmit FIFO empty
    SSI2_DR_R = reverseBitOrder(data);                   // LSB, write after reverse bit order
    while ((SSI2_SR_R & SSI_SR_RNE) == 0) {};            // wait until response
    uint16_t sync = SSI2_DR_R;                           // read byte of data, just for synchronization
#elif defined SSI3
    while ((SSI3_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};   // wait until SSI3 not busy/transmit FIFO empty
    SSI3_DR_R = reverseBitOrder(data);                   // LSB, write after reverse bit order
    while ((SSI3_SR_R & SSI_SR_RNE) == 0) {};            // wait until response
    uint16_t sync = SSI3_DR_R;                           // read byte of data, just for synchronization
#endif
}

#endif
