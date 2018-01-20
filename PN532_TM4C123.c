/*
 * File: PN532_TM4C123.c
 * PN532 Driver for TM4C123 Microcontroller.
 * ----------
 * Adapted code from elechouse PN532 driver for Arduino.
 * You can find the elechouse PN532 driver here:
 * https://github.com/elechouse/PN532.git
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

#ifdef DEBUG
#include "UART.h"
#endif

uint8_t packet_buffer[64];

static void delay(uint32_t N) {
    for(int n = 0; n < N; n++)                            // N msec
        for(int msec = 72724*2/91; msec > 0; msec--);     // 1 msec
}

static uint8_t reverseBitOrder(uint8_t byte) {
    
    return ((byte & 0x01) << 7) +
    ((byte & 0x02) << 5) +
    ((byte & 0x04) << 3) +
    ((byte & 0x08) << 1) +
    ((byte & 0x10) >> 1) +
    ((byte & 0x20) >> 3) +
    ((byte & 0x40) >> 5) +
    ((byte & 0x80) >> 7);
    
    //return ((byte & 0xF0) >> 4) + ((byte & 0x0F) << 4);
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
    
    GPIO_PORTA_CR_R |= 0x40;           // allow changes to PF4-0
    GPIO_PORTA_AMSEL_R &= ~0x40;       // disable analog functionality on PA2-5
    GPIO_PORTA_PCTL_R &= 0xF0FFFFFF;   // 4) PCTL GPIO on PF4-0
    GPIO_PORTA_DIR_R |= 0x40;          // 5) PF4,PF0 in, PF3-1 out
    GPIO_PORTA_AFSEL_R &= ~0x40;       // 6) disable alt funct on PF7-0
    GPIO_PORTA_DEN_R |= 0x40;          // enable digital I/O on PA2-5
    
    
    
    
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
    SSI0_CC_R &= ~SSI_CC_CS_M;
    SSI0_CC_R |= SSI_CC_CS_SYSPLL;
    SSI0_CPSR_R &= ~SSI_CPSR_CPSDVSR_M;                    // clear bit fields for SSI Clock Prescale Divisor
    SSI0_CPSR_R += 80;                                     // /40 clock divisor
    SSI0_CR0_R &= ~SSI_CR0_SCR_M;                          // clear bit fields for SSI0 Serial Clock Rate, SCR = 0
    SSI0_CR0_R &= ~SSI_CR0_SPH;                            // clear bit fields for SSI0 Serial Clock Phase, SPH = 0
    SSI0_CR0_R &= ~SSI_CR0_SPO;                            // clear bit fields for SSI0 Serial Clock Polarity, SPO = 0
    SSI0_CR0_R &= ~SSI_CR0_FRF_M;                          // clear bit fields for SSI0 Frame Format Select
    SSI0_CR0_R |= SSI_CR0_FRF_MOTO;                        // set frame format to Freescale SPI Frame Format
    SSI0_CR0_R &= ~SSI_CR0_DSS_M;                          // clear bit fields for SSI0 Data Size Select
    SSI0_CR0_R |= SSI_CR0_DSS_8;                           // set SSI data size to 8
    SSI0_CR1_R |= SSI_CR1_SSE;                             // enable SSI operation
    
    GPIO_PORTA_DATA_R &= ~0x40;
    delay(2);
    GPIO_PORTA_DATA_R |= 0x40;
    
}


/****************************************************
 *                                                  *
 *                Generic Functions                 *
 *                                                  *
 ****************************************************/

/*
 *
 */
uint32_t PN532_getFirmwareVersion(void) {
    
    packet_buffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;
    
    if (!writeCommandACK(packet_buffer, 1, 100)) return 0;
    
    GPIO_PORTF_DATA_R = 0x08;
    // read data packet
    readData(packet_buffer, 12);
    
    // check some basic stuff
    
    
    uint32_t response;
    
    int offset = 6;
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

/**
 *  Waits for an ISO14443A target to enter the field
 *  @param  cardBaudRate  Baud rate of the card
 *  @param  uid           Pointer to the array that will be populated
 *                        with the card's UID (up to 7 bytes)
 *  @param  uidLength     Pointer to the variable that will hold the
 *                        length of the card's UID.
 *  @return 1 if everything executed properly, 0 for an error
 */
int readPassiveTargetID (uint8_t card_baudrate, uint8_t * uid, uint8_t * uid_length, uint16_t timeout) {
    packet_buffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
    packet_buffer[1] = 1;  // max 1 cards at once (we can set this to 2 later)
    packet_buffer[2] = card_baudrate;
    
    
    return 0;
}








/****************************************************
 *                                                  *
 *             Mifare Classic functions             *
 *                                                  *
 ****************************************************/

/*
 *
 */
int mifareclassic_isTrailerBlock (uint32_t uiBlock)
{
    // Test if we are in the small or big sectors
    if (uiBlock < 128) return ((uiBlock + 1) % 4 == 0);
    else return ((uiBlock + 1) % 16 == 0);
}









/****************************************************
 *                                                  *
 *                Internal Functions                *
 *                                                  *
 ****************************************************/
/**
 * readACK
 * ----------
 */
int8_t readACK() {
    
    const uint8_t ACK_frame[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00}; // see NXP PN532 data sheet page 30
    uint8_t ACK_buffer[6];                                            // buffer for ACK signal
    
    GPIO_PORTA_DATA_R &= ~0x40;
    //delay(1);
    //SSI_write(PN532_SPI_DATAREAD);
    readData(ACK_buffer, 6);                                         // read ACK signal
    delay(1);
    GPIO_PORTA_DATA_R |= 0x40;
    return memcmp(ACK_buffer, ACK_frame, 6);
    
}


/**
 * isReadyForResponse
 * ----------
 */
static int isReadyForResponse(void) {
    GPIO_PORTA_DATA_R &= ~0x40;
    SSI_write(PN532_SPI_STATREAD);                              // write SPI starting command to PN532 module
    uint8_t status = SSI_read();                                          // read response from PN532
    delay(1);
    GPIO_PORTA_DATA_R |= 0x40;
    
    return status == PN532_SPI_READY;
    
}

/**
 * waitToBeReadyForResponse
 * ----------
 */
int waitToBeReadyForResponse(uint16_t wait_time) {
    //for (uint16_t timer = 0; timer < wait_time; timer += 10) {  // count up to wait time
    //    if (isReadyForResponse()) return 1;               // if ready, return 1
    //   for(int i = 0; i < 800000; i++);                         // delay before retry
    //}
    //return 0;                                               // time out, return 0
    
    while(!isReadyForResponse()) {
        delay(1);
        wait_time--;
        if (wait_time == 0) return 0;
    }
    return 1;
}






int writeCommandACK(uint8_t *cmd, uint8_t cmd_length, uint16_t wait_time) {
    
    writeCommand(cmd, cmd_length);
    
    if (!waitToBeReadyForResponse(wait_time)) return 0;
    GPIO_PORTF_DATA_R |= 0x08;
    if (readACK()) return 0;
    GPIO_PORTF_DATA_R |= 0x02;
    if (!waitToBeReadyForResponse(wait_time)) return 0;
    
    return 1;
}



/**
 *
 * data sheet page 28
 */
static void writeCommand(uint8_t *cmd, uint8_t cmd_length) {
    GPIO_PORTA_DATA_R &= ~0x40;
    
    SSI_write(PN532_SPI_DATAWRITE);                      // tell PN532 the host about to write data
    SSI_write(PN532_PREAMBLE);                           // write PREAMBLE
    SSI_write(PN532_STARTCODE1);                         // write first byte of START CODE
    SSI_write(PN532_STARTCODE2);                         // write second byte of START CODE
    
    cmd_length++;                                        // length of data field: TFI + DATA
    SSI_write(cmd_length);                               // write command length to LEN
    SSI_write(~cmd_length + 1);                          // write the 2's complement of command length to LCS
    SSI_write(PN532_HOSTTOPN532);                        // a frame from the host controller to the PN532
    
    uint8_t DCS = PN532_HOSTTOPN532;                     // Data checksum, see datasheet how it is used
    
    for (uint8_t i = 0; i < cmd_length - 1; i++) {
        SSI_write(cmd[i]);                               // write data
        DCS += cmd[i];
    }
    
    SSI_write(~DCS + 1);                                 // write 2's complement of DCS
    SSI_write(PN532_POSTAMBLE);                          // write POSTAMBLE
    
    delay(1);
    GPIO_PORTA_DATA_R |= 0x40;
}



/**
 * read_data
 * ----------
 *
 *
 */
void readData(uint8_t *data_buff, uint8_t data_length) {
    SSI_write(PN532_SPI_DATAREAD);
    
    for (uint8_t i = 0; i < data_length; i++) {
        data_buff[i] = SSI_read();
        if (data_buff[i] == 0xFF && i > 3)
            GPIO_PORTF_DATA_R |= 0x02;
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
    UART_OutString("\nSSI read data: 0x");
    UART_OutUDec(byte);
    return reverseBitOrder(byte);
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
    SSI0_DR_R = reverseBitOrder(byte);                     // write data
}

