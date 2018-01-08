/*
 * PN532_TM4C123.c
 * PN532 Driver for TM4C123 Microcontroller
 * ----------
 * Adapted code from Adafruit PN532 driver for Arduino. You can find the Adafruit driver for Arduino here:
 * https://github.com/adafruit/Adafruit-PN532.git
 * ----------
 * NXP PN532 datasheet: https://www.nxp.com/docs/en/user-guide/141520.pdf
 * ----------
 * Zee Livermorium
 * Dec 25, 2017
 */

#include <stdint.h>
#include <string.h>
#include "PN532_TM4C123.h"
#include "../inc/tm4c123gh6pm.h"   // put tm4c123gh6pm.h in your project folder or change this line

char ACK_Frame[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};   // see NXP PN532 data sheet page 30
uint8_t packet_buffer[64];

static uint8_t reverse_bit_order(uint8_t byte) {
    uint8_t reversed_byte = 0;
    reversed_byte |= (byte & 0x01) << 7;
    reversed_byte |= (byte & 0x02) << 5;
    reversed_byte |= (byte & 0x04) << 3;
    reversed_byte |= (byte & 0x08) << 1;
    reversed_byte |= (byte & 0x10) >> 1;
    reversed_byte |= (byte & 0x20) >> 3;
    reversed_byte |= (byte & 0x40) >> 5;
    reversed_byte |= (byte & 0x80) >> 7;
    return reversed_byte;
}

void lol(void) {
    SSI_write(PN532_SPI_STATREAD);                              // write SPI starting command to PN532 module
    for (int i = 0; i < 8000; i++);
    uint8_t byte = SSI_read();
    if (byte == PN532_SPI_READY) GPIO_PORTF_DATA_R = 0x08;
}

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
    GPIO_PORTA_AFSEL_R |= 0x3C;                            // enable alt funct on PA2-5
    GPIO_PORTA_PCTL_R &= ((~GPIO_PCTL_PA2_M) &             // clear bit fields for PA2
                          (~GPIO_PCTL_PA3_M) &             // clear bit fields for PA3
                          (~GPIO_PCTL_PA4_M) &             // clear bit fields for PA4
                          (~GPIO_PCTL_PA5_M));             // clear bit fields for PA5
    GPIO_PORTA_PCTL_R |= (GPIO_PCTL_PA2_SSI0CLK |          // configure PA2 as SSI0CLK
                          GPIO_PCTL_PA3_SSI0FSS |          // configure PA3 as SSI0FSS
                          GPIO_PCTL_PA4_SSI0RX |           // configure PA4 as SSI0RX
                          GPIO_PCTL_PA5_SSI0TX);           // configure PA5 as SSI0TX
    GPIO_PORTA_AMSEL_R &= ~0x3C;                           // disable analog functionality on PA2-5
    GPIO_PORTA_DEN_R |= 0x3C;                              // enable digital I/O on PA2-5
    
    /* SSI0 Set Up */
    SSI0_CR1_R &= ~SSI_CR1_SSE;                            // disable SSI0 operation
    SSI0_CR1_R &= ~SSI_CR1_MS;                             // configure SSI0 as master mode
    //SSI0_CC_R = (SSI0_CC_R&~SSI_CC_CS_M)+SSI_CC_CS_SYSPLL;
    SSI0_CPSR_R &= ~SSI_CPSR_CPSDVSR_M;                     // clear bit fields for SSI Clock Prescale Divisor
    SSI0_CPSR_R += 80;                                     // /80 clock divisor, must be even number in [0, 254]
    SSI0_CR0_R &= ~SSI_CR0_SCR_M;                          // clear bit fields for SSI0 Serial Clock Rate, SCR = 0
    SSI0_CR0_R &= ~SSI_CR0_SPH;                            // clear bit fields for SSI0 Serial Clock Phase, SPH = 0
    SSI0_CR0_R &= ~SSI_CR0_SPO;                            // clear bit fields for SSI0 Serial Clock Polarity, SPO = 0
    SSI0_CR0_R &= ~SSI_CR0_FRF_M;                          // clear bit fields for SSI0 Frame Format Select
    SSI0_CR0_R |= SSI_CR0_FRF_MOTO;                        // set frame format to Freescale SPI Frame Format
    SSI0_CR0_R &= ~SSI_CR0_DSS_M;                          // clear bit fields for SSI0 Data Size Select
    SSI0_CR0_R |= SSI_CR0_DSS_8;                           // set SSI data size to 8
    SSI0_CR1_R |= SSI_CR1_SSE;                             // enable SSI operation
    
    for (int i = 0; i < 8000000; i++);
    while(1) {
        lol();
    }
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
    
    packet_buffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;
    
    if (!write_command_ACK(packet_buffer, 1, 1000)) return 0;
    
    GPIO_PORTF_DATA_R = 0x08;
    // read data packet
    read_data(packet_buffer, 12);
    
    // check some basic stuff
    
    
    uint32_t response;
    
    int offset = 6;  // Skip a response byte when using I2C to ignore extra data.
    response = packet_buffer[offset++];
    response <<= 8;
    response |= packet_buffer[offset++];
    response <<= 8;
    response |= packet_buffer[offset++];
    response <<= 8;
    response |= packet_buffer[offset++];
    
    return response;
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
 * read_ACK
 * ----------
 */
int read_ACK() {
    uint8_t ACK_buffer[6];                                           // buffer for ACK signal
    read_data(ACK_buffer, 6);                                        // read ACK signal
    return 0 == strncmp((char *)ACK_Frame, (char *)ACK_buffer, 6);
}


/**
 * is_ready_for_response
 * ----------
 */
static int is_ready_for_response(void) {
    uint8_t byte;                                               // one byte of data to read
    SSI_write(PN532_SPI_STATREAD);                              // write SPI starting command to PN532 module
    byte = SSI_read();                                          // read response from PN532
    for(int i = 0; i < 800000; i++);
    if (byte == PN532_SPI_READY) return 1;                      // if recieve ready response, return 1
    return 0;                                                   // not ready return 0
}

/**
 * wait_to_be_ready_for_response
 * ----------
 */
int wait_to_be_ready_for_response(uint16_t wait_time) {
    for (uint16_t timer = 0; timer < wait_time; timer += 10) {  // count up to wait time
        if (is_ready_for_response()) return 1;               // if ready, return 1
        for(int i = 0; i < 800000; i++);                         // delay before retry
    }
    return 0;                                               // time out, return 0
}






int write_command_ACK(uint8_t *cmd, uint8_t cmd_length, uint16_t wait_time) {
    
    write_command(cmd, cmd_length);
    
    if (!wait_to_be_ready_for_response(wait_time)) return 0;
    GPIO_PORTF_DATA_R = 0x08;
    if (!read_ACK()) return 0;
    GPIO_PORTF_DATA_R = 0x04;
    if (!wait_to_be_ready_for_response(wait_time)) return 0;
    
    return 1;
}



/**
 *
 * data sheet page 28
 */
static void write_command(uint8_t *cmd, uint8_t cmd_length) {
    uint8_t DCS;                                         // Data checksum, see datasheet how it is used
    
    cmd_length++;                                        //
    
    SSI_write(PN532_SPI_DATAWRITE);                      //
    
    SSI_write(PN532_PREAMBLE);                           // write PREAMBLE
    SSI_write(PN532_STARTCODE1);                         // write first byte of START CODE
    SSI_write(PN532_STARTCODE2);                         // write second byte of START CODE
    SSI_write(cmd_length);                               // write command length to LEN
    SSI_write(~cmd_length + 1);                          // write the 2's complement of command length to LCS
    SSI_write(PN532_HOSTTOPN532);                        // a frame from the host controller to the PN532
    
    // according to datasheet the following line is wrong, but adafruit code has equivalent code
    // DCS = PN532_PREAMBLE + PN532_STARTCODE1 + PN532_STARTCODE2 + PN532_HOSTTOPN532;
    
    DCS = (uint8_t) PN532_HOSTTOPN532;
    
    for (uint8_t i = 0; i < cmd_length - 1; i++) {
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
        for(int i = 0; i < 800000; i++);
        data_buff[i] = SSI_read();
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
    uint8_t byte = SSI0_DR_R;
    return reverse_bit_order(byte);
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
    SSI0_DR_R = reverse_bit_order(byte);                                        // write data
}

