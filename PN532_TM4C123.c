/*
 * PN532_TM4C123.c
 * PN532 Driver for TM4C123 Microcontroller
 * ----------
 * NXP PN532 datasheet: https://www.nxp.com/docs/en/user-guide/141520.pdf
 * ----------
 * Zee Livermorium
 * Dec 25, 2017
 */

#include <stdint.h>
#include <string.h>
#include "tm4c123gh6pm.h"   // put tm4c123gh6pm.h in your project folder or change this line

char ACK_Frame[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};   // see NXP PN532 data sheet page 30

/****************************************************
 *                                                  *
 *                  Initializers                    *
 *                                                  *
 ****************************************************/

/**
 * PN532_SSI_Init
 * ----------
 * Discription: initialize SSI communication for PN532 Module. 
 */
void PN532_SSI_Init(void) {
    /* SSI and Port A Activation */
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R0;                 // enable SSI Module 0 clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;               // enable GPIO Port A clock
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R0) == 0){};    // allow time for activating
    
    /* Port A Set Up */
    GPIO_PORTA_AFSEL_R |= 0x2C;                            // enable alt funct on PA2-5
    GPIO_PORTA_PCTL_R &= ((~GPIO_PCTL_PA2_M) &             // clear bit fields for PA2
                          (~GPIO_PCTL_PA3_M) &             // clear bit fields for PA3
                          (~GPIO_PCTL_PA4_M) &             // clear bit fields for PA4
                          (~GPIO_PCTL_PA5_M));             // clear bit fields for PA5
    GPIO_PORTA_PCTL_R |= (GPIO_PCTL_PA2_SSI0CLK |          // configure PA2 as SSI0CLK
                          GPIO_PCTL_PA3_SSI0FSS |          // configure PA3 as SSI0FSS
                          GPIO_PCTL_PA4_SSI0RX |           // configure PA4 as SSI0RX
                          GPIO_PCTL_PA5_SSI0TX);           // configure PA5 as SSI0TX
    GPIO_PORTA_AMSEL_R &= ~0x2C;                           // disable analog functionality on PA2-5
    GPIO_PORTA_DEN_R |= 0x2C;                              // enable digital I/O on PA2-5
    
    /* SSI0 Set Up */
    SSI0_CR1_R &= ~SSI_CR1_SSE;                            // disable SSI operation
    SSI0_CR1_R &= ~SSI_CR1_MS;                             // configure SSI0 as master mode
    SSI0_CPSR_R &= SSI_CPSR_CPSDVSR_M;                     // clear bit fields for SSI Clock Prescale Divisor
    SSI0_CPSR_R += 2;                                      // /2 clock divisor, must be even number in [0, 254]
    SSI0_CR0_R &= ~SSI_CR0_SCR_M;                          // clear bit fields for SSI0 Serial Clock Rate, SCR = 0
    SSI0_CR0_R &= ~SSI_CR0_SPH;                            // clear bit fields for SSI0 Serial Clock Phase, SPH = 0
    SSI0_CR0_R &= ~SSI_CR0_SPO;                            // clear bit fields for SSI Serial Clock Polarity, SPO = 0
    SSI0_CR0_R &= ~SSI_CR0_FRF_M;                          // clear bit fields for SSI0 Frame Format Select
    SSI0_CR0_R |= SSI_CR0_FRF_MOTO;                        // set frame format to Freescale SPI Frame Format
    SSI0_CR0_R &= ~SSI_CR0_DSS_M;                          // clear bit fields for SSI0 Data Size Select
    SSI0_CR0_R |= SSI_CR0_DSS_8;                           // set SSI data size to 16
    SSI0_CR1_R |= SSI_CR1_SSE;                             // enable SSI operation
}


/****************************************************
 *                                                  *
 *                Generic Functions                 *
 *                                                  *
 ****************************************************/

/* 
 *
 */
uint32_t PN532_Firmware_Version(void) {
    uint32_t
}





/****************************************************
 *                                                  *
 *               ISO14443A Functions                *
 *                                                  *
 ****************************************************/









/****************************************************
 *                                                  *
 *             Mifare Classic functions             *
 *                                                  *
 ****************************************************/










/****************************************************
 *                                                  *
 *                Internal Functions                *
 *                                                  *
 ****************************************************/
/**
 * PN532_ACK
 * ----------
 */
bool PN532_ACK() {
    uint8_t ACK_buffer[6];                                           // buffer for ACK signal
    read_data(ACK_buffer, 6);                                        // read ACK signal
    return 0 == strncmp((char *)ACK_Frame, (char *)ACK_buffer, 6);
}


/**
 * is_ready_for_response
 * ----------
 */
static bool is_ready_for_response(void) {
    uint8_t byte;                                               // one byte of data to read
    SSI_write(PN532_SPI_STATREAD);                              // write SPI starting command to PN532 module
    byte = SSI_Read();                                          // read response from PN532
    if (byte == PN532_SPI_READY) return true;                   // if recieve ready response, return true
    return false;                                               // not ready return false
}

/**
 * wait_to_be_ready_for_response
 * ----------
 */
bool wait_to_be_ready_for_response(uint16_t wait_time) {
    for (uint16_t timer = 0; timer < wait_time; timer += 10) {  // count up to wait time
        if (is_ready_for_response()) return true;               // if ready, return true
        // delay 10                                             // delay before retry
    }
    return false;                                               // time out, return false
}





/**
 *
 * data sheet page 28
 */
bool write_command(uint8_t *cmd, uint8_t cmd_length) {
    
    uint8_t DCS;                                         // Data checksum, see datasheet how it is used
    
    SSI_write(PN532_SPI_DATAWRITE);                      //
    SSI_write(PN532_PREAMBLE);                           // write PREAMBLE
    SSI_write(PN532_STARTCODE1);                         // write first byte of START CODE
    SSI_write(PN532_STARTCODE2);                         // write second byte of START CODE
    SSI_write(cmd_length);                               // write command length to LEN
    SSI_write(~cmd_length + 1);                          // write the 2's complement of command length to LCS
    SSI_write(PN532_HOSTTOPN532);                        // a frame from the host controller to the PN532
    
    // according to datasheet the following line is wrong, but adafruit code has equivalent code
    // DCS = PN532_PREAMBLE + PN532_STARTCODE1 + PN532_STARTCODE2 + PN532_HOSTTOPN532;
    
    DCS = PN532_HOSTTOPN532;
    for (uint8_t i = 0; i < cmd_length; i++) {
        SSI_write(cmd[i]);                               // write data
        DCS += cmd[i];
    }
    
    SSI_write(~DCS + 1);                                 // write 2's complement of DCS
    SSI_write(PN532_POSTAMBLE);                          // write POSTAMBLE
}




/**
 * read_data
 * ----------
 *
 *
 */
void read_data(uint8_t *data_buff, uint8_t data_length) {
    SSI_write(PN532_SPI_DATAREAD);
    for (uint8_t i = 0; i < data_length; i++) {
        // delay 1
        data_buff[i] = SSI_Read();
    }
}


/**
 * PN532_SSI_Read
 * ----------
 * Return: byte of date read from PN532 module.
 * ----------
 * Discription: read one byte of data fromcPN532 module. 
 */
static uint8_t SSI_read(void) {
    return SSI0_DR_R & 0xFF;
} 
    

/**
 * PN532_SSI_write
 * ----------
 * Parameters:
 *   - byte: byte of data to be written.
 * ----------
 * Discription: write one byte of data to PN532 module.
 */
static void SSI_write(uint8_t byte) {
    while ((SSI0_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};       // wait until SSI0 not busy/transmit FIFO empty
    SSI0_DR_R = byte;                                        // write data
}

