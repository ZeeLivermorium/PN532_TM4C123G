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

/* Signal select macros for SSI */
#define SS_HIGH() GPIO_PORTA_DATA_R |= 0x08
#define SS_LOW() GPIO_PORTA_DATA_R &= ~0x08

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
    GPIO_PORTA_DIR_R |= 0x08;                              // make PA3 output
    GPIO_PORTA_AFSEL_R |= 0x34;                            // enable alt funct on PA2, 4, 5
    GPIO_PORTA_AFSEL_R &= ~0x08;                           // disable alt funct on PA3
    GPIO_PORTA_PUR_R |= 0x3C;                              // enable weak pullup on PA2,3,4,5
    GPIO_PORTA_PCTL_R &= ((~GPIO_PCTL_PA2_M) &             // clear bit fields for PA2
                          (~GPIO_PCTL_PA3_M) &             // clear bit fields for PA3, PA3 will be used as GPIO
                          (~GPIO_PCTL_PA4_M) &             // clear bit fields for PA4
                          (~GPIO_PCTL_PA5_M));             // clear bit fields for PA5
    GPIO_PORTA_PCTL_R |= (GPIO_PCTL_PA2_SSI0CLK |          // configure PA2 as SSI0CLK
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
    
    /* Wake Up PN532 */
    SS_LOW();
    delay(4);
    SS_HIGH();
    
    
}


/****************************************************
 *                                                  *
 *                Generic Functions                 *
 *                                                  *
 ****************************************************/

/**
 * PN532_getFirmwareVersion
 * ----------
 * @return 32-bit firmware version number for PN532 or 0 for an error.
 * ----------
 * Data Sheet: section 7.2.2 GetFirmwareVersion (page 73).
 */
uint32_t PN532_getFirmwareVersion(void) {
    packet_buffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;
    if (!writeCommand(packet_buffer, 1)) return 0;     // write command to PN532, return 0 means write fail
    if (readResponse(packet_buffer, 12) < 0) return 0; // read response from PN532, return 0 for error
    
    /* organize the result to an unsigned 32 bit integer */
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
 * @return -1 for an error.
 * ----------
 * @brief Configures the SAM (Secure Access Module).
 * ----------
 * Data Sheet: section 7.2.10 SAMConfiguration (page 89).
 */
int8_t SAMConfig(void) {
    packet_buffer[0] = PN532_COMMAND_SAMCONFIGURATION;
    packet_buffer[1] = 0x01;                           // normal mode;
    packet_buffer[2] = 0x14;                           // timeout 50ms * 20 = 1 second
    packet_buffer[3] = 0x01;                           // use IRQ pin!
    if (!writeCommand(packet_buffer, 4)) return -1;    // write command to PN532, return 0 means write fail
    
    return readResponse(packet_buffer, sizeof(packet_buffer));
}



/**
 * setPassiveActivationRetries
 * ----------
 * @param  maxRetries  0xFF to wait forever, 0x00..0xFE to timeout after mxRetries.
 * ----------
 * @return -1 for an error.
 * ----------
 * @brief Sets the MxRtyPassiveActivation uint8_t of the RFConfiguration register.
 * ----------
 * Data Sheet: section 7.3.1 RFConfiguration (page 101).
 */
int8_t setPassiveActivationRetries(uint8_t maxRetries) {
    packet_buffer[0] = PN532_COMMAND_RFCONFIGURATION;
    packet_buffer[1] = 5;                              // Config item 5 (MaxRetries)
    packet_buffer[2] = 0xFF;                           // MxRtyATR (default = 0xFF)
    packet_buffer[3] = 0x01;                           // MxRtyPSL (default = 0x01)
    packet_buffer[4] = maxRetries;

    if (!writeCommand(packet_buffer, 5)) return -1;
    
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
 * @param  cardBaudRate  Baud rate of the card.
 * @param  uid           Pointer to the array that will be populated with the card's UID (up to 7 bytes).
 * @param  uidLength     Pointer to the variable that will hold the length of the card's UID.
 * ----------
 * @return 1 if everything executed properly, 0 for an error.
 * ----------
 * @brief Waits for an ISO14443A target to enter the field.
 * ----------
 * Data Sheet: section 7.3.5 InListPassiveTarget (page 115).
 */
uint8_t readPassiveTargetID (uint8_t card_baudrate, uint8_t *uid, uint8_t *uid_length) {
    
    packet_buffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
    packet_buffer[1] = 1;                              // max 1 card
    packet_buffer[2] = card_baudrate;                  // read datasheet to see card baudrate setting
    
    if (!writeCommand(packet_buffer, 3)) return 0;     // return 0 for error
    if (readResponse(packet_buffer, sizeof(packet_buffer)) < 0) return 0;
    
    /*
     |         ISO14443A Card Formatt          |
     |  -------------------------------------  |
     |   byte          |   Description         |
     |  -------------  |  -------------------  |
     |   b0            |   Tags Found          |
     |   b1            |   Tag Number          |
     |   b2..3         |   SENS_RES            |
     |   b4            |   SEL_RES             |
     |   b5            |   NFCID Length        |
     |   b6..NFCIDLen  |   NFCID               |
     */
    
    /* byte 0 */
    if (packet_buffer[0] != 1) return 0;               // return 0, if no tags found
    
    /* byte 5 */
    *uid_length = packet_buffer[5];                    // record uid length
    
    /* UID */
    for (uint8_t i = 0; i < packet_buffer[5]; i++)
        uid[i] = packet_buffer[6 + i];                 // record uid
    
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
    
    SSI_write(PN532_SPI_STATREAD);                     // write SPI starting command to PN532 module
    uint8_t status = SSI_read();                       // read response from PN532
    
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
    if (!waitToBeReadyForResponse(1000)) return 0;     // wait for PN532 to be ready

    SS_LOW();
    delay(1);
    
    SSI_write(PN532_SPI_DATAREAD);                     // tell PN532 the host about to read data
    
    /* read 1st to 3rd bytes */
    if (SSI_read() != PN532_PREAMBLE   ||              // first byte should be PREAMBLE
        SSI_read() != PN532_STARTCODE1 ||              // second byte should be STARTCODE1
        SSI_read() != PN532_STARTCODE2                 // third byte should be STARTCODE2
        ) {
        SS_HIGH();                                     // pull SS high since we are exiting this function
        return PN532_INVALID_FRAME;                    // return invalid frame code as result
    }
    
    /* read 4th and 5th bytes */
    uint8_t LEN = SSI_read();                          // LEN: number of bytes in the data field
    uint8_t LCS = SSI_read();                          // LCS: Packet Length Checksum
    if ((uint8_t)(LEN + LCS) != 0x00 ) {
        SS_HIGH();                                     // pull SS high since we are exiting this function
        return PN532_INVALID_FRAME;                    // return invalid frame code as result
    }
    
    /* read 6th and 7th bytes */
    uint8_t PD0 = command + 1;                         // PD0 is command code
    if (PN532_PN532TOHOST != SSI_read() || PD0 != SSI_read()) {
        SS_HIGH();                                     // pull SS high since we are exiting this function
        return PN532_INVALID_FRAME;                    // return invalid frame code as result
    }
    
    /* check buffer size before read actual data */
    LEN -= 2;                                          // subtract TFI and PD0(command) from DATA length
    if (LEN > data_length) {                           // if no enough space, dump bytes for synchronization
        for (uint8_t i = 0; i < LEN; i++) SSI_read();  // dump data
        SSI_read();                                    // dump DCS
        SSI_read();                                    // dump POSTAMBLE

        SS_HIGH();                                     // pull SS high since we are exiting this function
        return PN532_NO_SPACE;                         // return (buffer) no space error code as result
    }
    
    /* read actual data */
    uint8_t SUM = PN532_PN532TOHOST + PD0;             // SUM: TFI + DATA, DATA = PD0 + PD1 + ... + PDn
    for (uint8_t i = 0; i < LEN; i++) {
        data_buffer[i] = SSI_read();                   // get data
        SUM += data_buffer[i];                         // accumulate SUM
    }
    
    /* read data checksum byte */
    uint8_t DCS = SSI_read();
    if ((uint8_t)(SUM + DCS) != 0) {
        SS_HIGH();                                     // pull SS high since we are exiting this function
        return PN532_INVALID_FRAME;                    // proper frame should result in SUM + DCS = 0
    }
    
    /* read POSTAMBLE */
    SSI_read();                                        // dump for synchronization
    
    delay(1);                                          // give time for the last write to finish
    SS_HIGH();
    
    return LEN;
}


/**
 *
 * data sheet page 28
 */
static void writeFrame(uint8_t *cmd, uint8_t cmd_length) {
    SS_LOW();
    delay(1);
    
    SSI_write(PN532_SPI_DATAWRITE);                    // tell PN532 the host about to write data
    SSI_write(PN532_PREAMBLE);                         // write PREAMBLE
    SSI_write(PN532_STARTCODE1);                       // write first byte of START CODE
    SSI_write(PN532_STARTCODE2);                       // write second byte of START CODE
    
    cmd_length++;                                      // length of data field: TFI + DATA
    SSI_write(cmd_length);                             // write command length to LEN
    SSI_write(~cmd_length + 1);                        // write the 2's complement of command length to LCS
    SSI_write(PN532_HOSTTOPN532);                      // a frame from the host controller to the PN532
    
    uint8_t DCS = PN532_HOSTTOPN532;                   // data checksum, see datasheet how it is used
    
    for (uint8_t i = 0; i < cmd_length - 1; i++) {
        SSI_write(cmd[i]);                             // write data byte
        DCS += cmd[i];                                 // accumulate data checksum
    }
    
    SSI_write(~DCS + 1);                               // write 2's complement of DCS
    SSI_write(PN532_POSTAMBLE);                        // write POSTAMBLE

    delay(1);                                         // give time for the last write to finish
    SS_HIGH();
}


/**
 * writeCommand
 * ----------
 *
 */
static int writeCommand(uint8_t *cmd, uint8_t cmd_length) {
    command = cmd[0];                                  // record command for response verification
    writeFrame(cmd, cmd_length);                       // write command
    if (!waitToBeReadyForResponse(PN532_ACK_WAIT_TIME)) return 0;
    if (readACK()) return 0;                           // read ACK
    return 1;
}


/**
 * readACK
 * ----------
 * see NXP PN532 data sheet page 30 for ACK frame
 */
static int8_t readACK() {
    uint8_t ACK_buffer[6];                             // buffer for ACK signal
    
    SS_LOW();                                          // start
    delay(1);                                          // wake up PN532
    
    SSI_write(PN532_SPI_DATAREAD);                     // tell PN532 the host about to read data
    for (int i = 0; i < 6; i++)
        ACK_buffer[i] = SSI_read();                    // read ACK frame
    
    delay(1);                                          // give time for the last write to finish
    SS_HIGH();                                         // end
    
    return memcmp(ACK_buffer, ACK_frame, 6);           // check ACK frame and return
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
    while((SSI0_SR_R & SSI_SR_BSY) == SSI_SR_BSY){};   // wait until SSI0 not busy/transmit FIFO empty
    SSI0_DR_R = 0x00;                                  // data out, garbage
    while((SSI0_SR_R & SSI_SR_RNE) == 0){};            // wait until response
    uint8_t byte = SSI0_DR_R;                          // read byte of data
    return reverseBitOrder(byte);                      // reverse for LSB input
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
    while((SSI0_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};  // wait until SSI0 not busy/transmit FIFO empty
    SSI0_DR_R = reverseBitOrder(byte);                 // write data, reverse for LSB output
    while((SSI0_SR_R & SSI_SR_RNE) == 0){};            // wait until response
    uint8_t data = SSI0_DR_R;                          // read byte of data, just for synchronization
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
    for(int n = 0; n < N; n++)                         // N msec
        for(int msec = 72724*2/50; msec > 0; msec--);  // 1 msec
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

