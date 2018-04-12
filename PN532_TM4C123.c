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
 * For future development and updates, please follow this repository:
 * https://github.com/ZeeLivermorium/PN532_TM4C123G
 * ----------
 * Zee Livermorium
 * Dec 25, 2017
 */

#include <stdint.h>
#include <string.h>
#include "PN532_TM4C123.h"
#include "../inc/tm4c123gh6pm.h"   // put tm4c123gh6pm.h in the right path accordingly


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
    GPIO_PORTA_PUR_R |= 0x3C;                              // enable weak pullup on PA2-5
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
    SSI0_CPSR_R += 40;                                     // /40 clock divisor, 2Mhz
    SSI0_CR0_R &= ~SSI_CR0_SCR_M;                          // clear bit fields for SSI0 Serial Clock Rate, SCR = 0
    SSI0_CR0_R &= ~SSI_CR0_SPH;                            // clear bit fields for SSI0 Serial Clock Phase, SPH = 0
    SSI0_CR0_R &= ~SSI_CR0_SPO;                            // clear bit fields for SSI0 Serial Clock Polarity, SPO = 0
    SSI0_CR0_R &= ~SSI_CR0_FRF_M;                          // clear bit fields for SSI0 Frame Format Select
    SSI0_CR0_R |= SSI_CR0_FRF_MOTO;                        // set frame format to Freescale SPI Frame Format
    SSI0_CR0_R &= ~SSI_CR0_DSS_M;                          // clear bit fields for SSI0 Data Size Select
    SSI0_CR0_R |= SSI_CR0_DSS_8;                           // set SSI data size to 8
    SSI0_CR1_R |= SSI_CR1_SSE;                             // enable SSI operation
    
}


/****************************************************
 *                                                  *
 *                Generic Functions                 *
 *                                                  *
 ****************************************************/

/**
 * pg 73
 */
uint32_t PN532_getFirmwareVersion(void) {
    
    packet_buffer[0] = PN532_COMMAND_GETFIRMWAREVERSION; // get GETFIRMWAREVERSION command
    if (!writeCommand(packet_buffer, 1)) return 0;       // write command to PN532, return 0 means write fail
    if (readResponse(packet_buffer, 12) < 0) return 0;   // read response from PN532, negative return value means error
    
    // organize the result to an unsigned 32 bit integer
    uint32_t response;
    
    response = packet_buffer[0];
    response <<= 8;
    response |= packet_buffer[1];
    response <<= 8;
    response |= packet_buffer[2];
    response <<= 8;
    response |= packet_buffer[3];
    
    return response;
}


/**
 * SAMConfig
 * ----------
 * @return 1 if everything executed properly, 0 for an error.
 * ----------
 * @brief  Configures the SAM (Secure Access Module).
 */
int8_t SAMConfig(void) {
    packet_buffer[0] = PN532_COMMAND_SAMCONFIGURATION;
    packet_buffer[1] = 0x01;        // normal mode;
    packet_buffer[2] = 0x14;        // timeout 50ms * 20 = 1 second
    packet_buffer[3] = 0x01;        // use IRQ pin!
    
    if (!writeCommand(packet_buffer, 4)) return -1;  // write command to PN532, return 0 means write fail
    
    return readResponse(packet_buffer, sizeof(packet_buffer));
}



/**
 * setPassiveActivationRetries
 * ----------
 * @param  maxRetries    0xFF to wait forever, 0x00..0xFE to timeout after mxRetries.
 * ----------
 * @return 1 if everything executed properly, 0 for an error.
 * ----------
 * @brief Sets the MxRtyPassiveActivation uint8_t of the RFConfiguration register.
 */
int8_t setPassiveActivationRetries(uint8_t maxRetries) {
    packet_buffer[0] = PN532_COMMAND_RFCONFIGURATION;
    packet_buffer[1] = 5;           // Config item 5 (MaxRetries)
    packet_buffer[2] = 0xFF;        // MxRtyATR (default = 0xFF)
    packet_buffer[3] = 0x01;        // MxRtyPSL (default = 0x01)
    packet_buffer[4] = maxRetries;
    
    if (!writeCommand(packet_buffer, 5)) return 4;  // no ACK
    
    return readResponse(packet_buffer, sizeof(packet_buffer));
}


/****************************************************
 *                                                  *
 *               ISO14443A Functions                *
 *                                                  *
 ****************************************************/

/**
 * readPassiveTargetID
 * ----------
 * @param  cardBaudRate  Baud rate of the card
 * @param  uid           Pointer to the array that will be populated with the card's UID (up to 7 bytes)
 * @param  uidLength     Pointer to the variable that will hold the length of the card's UID.
 * ----------
 * @return 1 if everything executed properly, 0 for an error
 * ----------
 * @brief Waits for an ISO14443A target to enter the field
 */
uint8_t readPassiveTargetID (uint8_t card_baudrate, uint8_t *uid, uint8_t *uid_length) {
    packet_buffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
    packet_buffer[1] = 1;  // max 1 cards at once (we can set this to 2 later)
    packet_buffer[2] = card_baudrate;
    
    if (!writeCommand(packet_buffer, 3)) return 0;  // command failed
    if (readResponse(packet_buffer, sizeof(packet_buffer)) < 0) return 0;   // read data packet
    
    /*
     ISO14443A card response should be in the following format:
     
     byte            Description
     -------------   ------------------------------------------
     b0              Tags Found
     b1              Tag Number (only one used in this example)
     b2..3           SENS_RES
     b4              SEL_RES
     b5              NFCID Length
     b6..NFCIDLen    NFCID
     */
    
    if (packet_buffer[0] != 1) return 0;         // no tags found
    
    uint16_t sens_res = packet_buffer[2];
    sens_res <<= 8;
    sens_res |= packet_buffer[3];
    *uid_length = packet_buffer[5];               // record uid length
    
    for (uint8_t i = 0; i < packet_buffer[5]; i++) uid[i] = packet_buffer[6 + i];   // record uid
    
    return 1;
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










// MARK: - PRIVATE

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
    SSI_write(PN532_SPI_STATREAD);      // write SPI starting command to PN532 module
    uint8_t status = SSI_read() & 1;    // read response from PN532
    return status == PN532_SPI_READY;   // check if PN532 is ready and return the result
}

/**
 * waitToBeReadyForResponse
 * ----------
 *
 *
 */
static int waitToBeReadyForResponse(uint16_t wait_time) {
    while(!isReadyForResponse()) {
        delay(1);
        wait_time--;
        if (wait_time == 0) return 0;
    }
    return 1;
}


/**
 * readResponse
 * ----------
 * page 28
 *
 */
static int16_t readResponse(uint8_t *data_buffer, uint8_t data_length) {
    if(!waitToBeReadyForResponse(1000)) return 0;
    
    SSI_write(PN532_SPI_DATAREAD);     // tell PN532 the host about to read data
    
    if (SSI_read() != PN532_PREAMBLE   ||
        SSI_read() != PN532_STARTCODE1 ||
        SSI_read() != PN532_STARTCODE2
        )
        return PN532_INVALID_FRAME;    // return invalid frame code as result
    
    uint8_t LEN = SSI_read();          // LEN: number of bytes in the data field
    uint8_t LCS = SSI_read();          // LCS: Packet Length Checksum
    if ((uint8_t)(LEN + LCS) != 0x00 ) return PN532_INVALID_FRAME;
    
    uint8_t PD0 = command + 1;         // PD0 is command code
    if (PN532_PN532TOHOST != SSI_read() || PD0 != SSI_read())
        return PN532_INVALID_FRAME;
    
    LEN -= 2;                          // subtract TFI and PD0(command) from DATA length
    if (LEN > data_length) {
        for (uint8_t i = 0; i < LEN; i++) SSI_read();  // dump data
        SSI_read();                                    // dump DCS
        SSI_read();                                    // dump POSTAMBLE
        return PN532_NO_SPACE;         // return (buffer) no space error code as result
    }
    
    uint8_t SUM = PN532_PN532TOHOST + PD0;  // SUM: TFI + DATA, DATA = PD0 + PD1 + ... + PDn
    for (uint8_t i = 0; i < LEN; i++) {
        data_buffer[i] = SSI_read();        // get data
        SUM += data_buffer[i];              // accumulate SUM
    }
    
    uint8_t DCS = SSI_read();          // DCS: data checksum byte
    if ((uint8_t)(SUM + DCS) != 0)
        return PN532_INVALID_FRAME;    // proper frame should result in SUM + DCS = 0
    
    SSI_read();         // dump POSTAMBLE

    
    return LEN;
}


/**
 *
 * data sheet page 28
 */
static void writeFrame(uint8_t *cmd, uint8_t cmd_length) {
    
    SSI_write(PN532_SPI_DATAWRITE);                   // tell PN532 the host about to write data
    SSI_write(PN532_PREAMBLE);                        // write PREAMBLE
    SSI_write(PN532_STARTCODE1);                      // write first byte of START CODE
    SSI_write(PN532_STARTCODE2);                      // write second byte of START CODE
    
    cmd_length++;                                     // length of data field: TFI + DATA
    SSI_write(cmd_length);                            // write command length to LEN
    SSI_write(~cmd_length + 1);                       // write the 2's complement of command length to LCS
    SSI_write(PN532_HOSTTOPN532);                     // a frame from the host controller to the PN532
    
    uint8_t DCS = PN532_HOSTTOPN532;                  // data checksum, see datasheet how it is used
    
    for (uint8_t i = 0; i < cmd_length - 1; i++) {
        SSI_write(cmd[i]);                            // write data byte
        DCS += cmd[i];                                // accumulate data checksum
    }
    
    SSI_write(~DCS + 1);                              // write 2's complement of DCS
    SSI_write(PN532_POSTAMBLE);                       // write POSTAMBLE

}


/**
 * writeCommand
 * ----------
 *
 */
static int writeCommand(uint8_t *cmd, uint8_t cmd_length) {
    command = cmd[0];
    writeFrame(cmd, cmd_length);                                  // write command
    if (!waitToBeReadyForResponse(PN532_ACK_WAIT_TIME)) return 0; // wait for responce to be ready
    if (readACK()) return 0;                                      // read ACK
    
    return 1;
}


/**
 * readACK
 * ----------
 * see NXP PN532 data sheet page 30 for ACK frame
 */
static int8_t readACK() {
    const uint8_t ACK_frame[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};    // expected ACK frame
    uint8_t ACK_buffer[6];                                               // buffer for ACK signal
    
    SSI_write(PN532_SPI_DATAREAD);                                       // tell PN532 the host about to read data
    for (uint8_t i = 0; i < 6; i++)
        ACK_buffer[i] = SSI_read();                                      // read data byte
    
    return memcmp(ACK_buffer, ACK_frame, 6);                             // check ACK frame and return
}


/****************************************************
 *                                                  *
 *           Low Level Interface Functions          *
 *                                                  *
 ****************************************************/

/**
 * PN532_SSI_Read
 * ----------
 * Return: byte of date read from PN532 module.
 * ----------
 * Discription: read one byte of data fromcPN532 module.
 */
static uint8_t SSI_read(void) {
    while((SSI0_SR_R & SSI_SR_BSY) == SSI_SR_BSY){};  // wait until SSI0 not busy/transmit FIFO empty
    SSI0_DR_R = 0x00;                                 // data out, garbage
    while((SSI0_SR_R & SSI_SR_RNE) == 0){};           // wait until response
    uint8_t byte = SSI0_DR_R;                         // read byte of data
    return reverseBitOrder(byte);                     // reverse for LSB input
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
    while((SSI0_SR_R & SSI_SR_BSY) == SSI_SR_BSY){};  // wait until SSI0 not busy/transmit FIFO empty
    SSI0_DR_R = reverseBitOrder(byte);                // write data, reverse for LSB output
    while((SSI0_SR_R & SSI_SR_RNE) == 0){};           // wait until response
    uint8_t data = SSI0_DR_R;                         // read byte of data, just for synchronization
}


/****************************************************
 *                                                  *
 *                 Helper Functions                 *
 *                                                  *
 ****************************************************/

/**
 * delay
 * ----------
 * Description: delay N msec
 */
void delay(uint32_t N) {
    for(int n = 0; n < N; n++)                            // N msec
        for(int msec = 72724*2/91; msec > 0; msec--);     // 1 msec
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

