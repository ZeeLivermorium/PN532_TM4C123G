/*
  * SSI.c
  * ----------
  * Inspired by examples in ValvanoWareTM4C123 by Dr. Jonathan Valvano
  * as well as his book Embedded Systems: Real-Time Interfacing to Arm Cortex-M Microcontrollers
  * You can find ValvanoWareTM4C123 at http://edx-org-utaustinx.s3.amazonaws.com/UT601x/ValvanoWareTM4C123.zip?dl=1
  * You can find his book at https://www.amazon.com/gp/product/1463590156/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1
  * You can find more of his work at http://users.ece.utexas.edu/~valvano/
  * ----------
  * @author Zee Livermorium
  * @date Apr 14, 2018
  */

#include <stdint.h>
#include <string.h>
#include "SSI.h"
#include "tm4c123gh6pm.h"


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
 *                 Helper Functions                 *
 *                                                  *
 ****************************************************/

/**
 * reverseBitOrder
 * ----------
 * Discription: to output in the order of LSB first, we need to reverse all bits.
 */
uint8_t reverseBitOrder(uint8_t byte) {
    return (((byte & 0x01) << 7) +
            ((byte & 0x02) << 5) +
            ((byte & 0x04) << 3) +
            ((byte & 0x08) << 1) +
            ((byte & 0x10) >> 1) +
            ((byte & 0x20) >> 3) +
            ((byte & 0x40) >> 5) +
            ((byte & 0x80) >> 7));
}

/****************************************************
 *                                                  *
 *                   SS Functions                   *
 *                                                  *
 ****************************************************/

void SS_HIGH (void) {
#if defined SSI0
    GPIO_PORTA_DATA_R |= 0x08;
#elif defined SSI1D
    GPIO_PORTD_DATA_R |= 0x02;
#elif defined SSI1F
    GPIO_PORTF_DATA_R |= 0x08;
#elif defined SSI2
    GPIO_PORTB_DATA_R |= 0x20;
#elif defined SSI3
    GPIO_PORTD_DATA_R |= 0x02;
#endif
}

void SS_LOW (void) {
#if defined SSI0
    GPIO_PORTA_DATA_R &= ~0x08;
#elif defined SSI1D
    GPIO_PORTD_DATA_R &= ~0x02;
#elif defined SSI1F
    GPIO_PORTF_DATA_R &= ~0x08;
#elif defined SSI2
    GPIO_PORTB_DATA_R &= ~0x20;
#elif defined SSI3
    GPIO_PORTD_DATA_R &= ~0x02;
#endif
    
}


/****************************************************
 *                                                  *
 *                   Initializer                    *
 *                                                  *
 ****************************************************/

/**
 * SSI0_A_Init
 * ----------
 * @brief initialize SSI0 on Port A with corresponding setting parameters.
 */
void SSI_Init() {
#if defined SSI0
    /*-- SSI0 and Port A Activation --*/
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R0;                 // enable SSI Module 0 clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;               // enable GPIO Port A clock
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R0) == 0) {};  // allow time for activating
    
    /*-- Port A Set Up --*/
    GPIO_PORTA_DIR_R |= 0x08;                              // make PA3 output
    GPIO_PORTA_AFSEL_R |= 0x34;                            // enable alt funct on PA2, 4, 5
    GPIO_PORTA_AFSEL_R &= ~0x08;                           // disable alt funct on PA3
    GPIO_PORTA_PCTL_R &= ((~GPIO_PCTL_PA2_M) &             // clear bit fields for PA2
                          (~GPIO_PCTL_PA3_M) &             // clear bit fields for PA3, PA3 will be used as GPIO
                          (~GPIO_PCTL_PA4_M) &             // clear bit fields for PA4
                          (~GPIO_PCTL_PA5_M));             // clear bit fields for PA5
    GPIO_PORTA_PCTL_R |= (GPIO_PCTL_PA2_SSI0CLK |          // configure PA2 as SSI0CLK
                          GPIO_PCTL_PA4_SSI0RX  |          // configure PA4 as SSI0RX
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
    /* SSI1 Activation */
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R1;                 // enable SSI Module 1 clock
    
#if defined SSI1D
    /* Port D Activation */
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;               // enable GPIO Port D clock
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R3) == 0) {};  // allow time for activating
    
    /* Port D Set Up */
    GPIO_PORTD_DIR_R |= 0x02;                              // make PD1 output
    GPIO_PORTD_AFSEL_R |= 0x0D;                            // enable alt funct on PD0, 2, 3
    GPIO_PORTD_AFSEL_R &= ~0x02;                           // disable alt funct on PD1
    GPIO_PORTD_PCTL_R &= ((~GPIO_PCTL_PD0_M) &             // clear bit fields for PD0
                          (~GPIO_PCTL_PD1_M) &             // clear bit fields for PD1, PD1 will be used as GPIO
                          (~GPIO_PCTL_PD2_M) &             // clear bit fields for PD2
                          (~GPIO_PCTL_PD3_M));             // clear bit fields for PD3
    GPIO_PORTD_PCTL_R |= (GPIO_PCTL_PD0_SSI1CLK |          // configure PD0 as SSI1CLK
                          GPIO_PCTL_PD2_SSI1RX  |          // configure PD2 as SSI1RX
                          GPIO_PCTL_PD3_SSI1TX);           // configure PD3 as SSI1TX
    GPIO_PORTD_AMSEL_R &= ~0x0F;                           // disable analog functionality on PD0-3
    GPIO_PORTD_DEN_R |= 0x0F;                              // enable digital I/O on PD0-3
    
#elif defined SSI1F
    /* Port F Activation */
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;               // enable GPIO Port F clock
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0) {};  // allow time for activating
    
    /* Port F Set Up */
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;                     // unlock GPIO Port F --- this step is only for Port F
    GPIO_PORTF_CR_R = 0x0F;                                // allow changes to PF0-3 --- this step is only for Port F
    GPIO_PORTF_DIR_R |= 0x08;                              // make PF3 output
    GPIO_PORTF_AFSEL_R |= 0x07;                            // enable alt funct on PF0-2
    GPIO_PORTF_AFSEL_R &= ~0x08;                           // disable alt funct on PF3
    GPIO_PORTF_PCTL_R &= ((~GPIO_PCTL_PF0_M) &             // clear bit fields for PF0
                          (~GPIO_PCTL_PF1_M) &             // clear bit fields for PF1
                          (~GPIO_PCTL_PF2_M) &             // clear bit fields for PF2
                          (~GPIO_PCTL_PF3_M));             // clear bit fields for PF3, PF3 will be used as GPIO
    GPIO_PORTF_PCTL_R |= (GPIO_PCTL_PF0_SSI1RX  |          // configure PF0 as SSI1RX
                          GPIO_PCTL_PF1_SSI1TX  |          // configure PF1 as SSI1TX
                          GPIO_PCTL_PF2_SSI1CLK);          // configure PF2 as SSI1CLK
    GPIO_PORTF_AMSEL_R &= ~0x0F;                           // disable analog functionality on PF0-3
    GPIO_PORTF_DEN_R |= 0x0F;                              // enable digital I/O on PF0-3
    
#endif
    
    /* SSI1 Set Up */
    SSI1_CR1_R &= ~SSI_CR1_SSE;                            // disable SSI operation
    SSI1_CR1_R &= ~SSI_CR1_MS;                             // configure SSI1 as master mode
    SSI1_CPSR_R &= SSI_CPSR_CPSDVSR_M;                     // clear bit fields for SSI Clock Prescale Divisor
    SSI1_CPSR_R += 40;                                     // /40 clock divisor, 2Mhz
    SSI1_CR0_R &= ~SSI_CR0_SCR_M;                          // clear bit fields for SSI1 Serial Clock Rate, SCR = 0
    SSI1_CR0_R &= ~SSI_CR0_SPO;                            // clear bit fields for SSI1 Serial Clock Polarity, SPO = 0
    SSI1_CR0_R &= ~SSI_CR0_SPH;                            // clear bit fields for SSI1 Serial Clock Phase, SPH = 0
    SSI1_CR0_R &= ~SSI_CR0_FRF_M;                          // clear bit fields for SSI1 Frame Format Select
    SSI1_CR0_R |= SSI_CR0_FRF_MOTO;                        // set frame format to Freescale SPI Frame Format
    SSI1_CR0_R &= ~SSI_CR0_DSS_M;                          // clear bit fields for SSI1 Data Size Select
    SSI1_CR0_R |= SSI_CR0_DSS_8;                           // set SSI data size to 8
    SSI1_CR1_R |= SSI_CR1_SSE;                             // enable SSI operation
    
#elif defined SSI2
    /* SSI2 and Port B Activation */
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;                 // enable SSI Module 2 clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;               // enable GPIO Port B clock
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1) == 0) {};  // allow time for activating
    
    /* Port B Set Up */
    GPIO_PORTB_DIR_R |= 0x20;                              // make PB5 output
    GPIO_PORTB_AFSEL_R |= 0xD0;                            // enable alt funct on PB4, 6, 7
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
    /* SSI3 and Port D Activation */
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R3;                 // enable SSI Module 3 clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;               // enable GPIO Port D clock
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R3) == 0) {};  // allow time for activating
    
    /* Port D Set Up */
    GPIO_PORTD_DIR_R |= 0x02;                              // make PD1 output
    GPIO_PORTD_AFSEL_R |= 0x0D;                            // enable alt funct on PD0, 2, 3
    GPIO_PORTD_AFSEL_R &= ~0x02;                           // disable alt funct on PD1
    GPIO_PORTD_PCTL_R &= ((~GPIO_PCTL_PD0_M) &             // clear bit fields for PD0
                          (~GPIO_PCTL_PD1_M) &             // clear bit fields for PD1, PD1 will be used as GPIO
                          (~GPIO_PCTL_PD2_M) &             // clear bit fields for PD2
                          (~GPIO_PCTL_PD3_M));             // clear bit fields for PD3
    GPIO_PORTD_PCTL_R |= (GPIO_PCTL_PD0_SSI3CLK |          // configure PD0 as SSI3CLK
                          GPIO_PCTL_PD2_SSI3RX  |          // configure PD2 as SSI3RX
                          GPIO_PCTL_PD3_SSI3TX);           // configure PD3 as SSI3TX
    GPIO_PORTD_AMSEL_R &= ~0x0F;                           // disable analog functionality on PD0-3
    GPIO_PORTD_DEN_R |= 0x0F;                              // enable digital I/O on PD0-3
    
    /* SSI3 Set Up */
    SSI3_CR1_R &= ~SSI_CR1_SSE;                            // disable SSI operation
    SSI3_CR1_R &= ~SSI_CR1_MS;                             // configure SSI3 as master mode
    SSI3_CPSR_R &= SSI_CPSR_CPSDVSR_M;                     // clear bit fields for SSI Clock Prescale Divisor
    SSI3_CPSR_R += 40;                                     // /40 clock divisor, 2Mhz
    SSI3_CR0_R &= ~SSI_CR0_SCR_M;                          // clear bit fields for SSI3 Serial Clock Rate, SCR = 0
    SSI3_CR0_R &= ~SSI_CR0_SPO;                            // clear bit fields for SSI3 Serial Clock Polarity, SPO = 0
    SSI3_CR0_R &= ~SSI_CR0_SPH;                            // clear bit fields for SSI3 Serial Clock Phase, SPH = 0
    SSI3_CR0_R &= ~SSI_CR0_FRF_M;                          // clear bit fields for SSI3 Frame Format Select
    SSI3_CR0_R |= SSI_CR0_FRF_MOTO;                        // set frame format to Freescale SPI Frame Format
    SSI3_CR0_R &= ~SSI_CR0_DSS_M;                          // clear bit fields for SSI3 Data Size Select
    SSI3_CR0_R |= SSI_CR0_DSS_8;                           // set SSI data size to 8
    SSI3_CR1_R |= SSI_CR1_SSE;                             // enable SSI operation
    
#endif
}


/****************************************************
 *                                                  *
 *                   I/O Functions                  *
 *                                                  *
 ****************************************************/

/**
 * SSI_read
 * ----------
 * @return date read from slave device.
 */
uint16_t SSI_read (void) {
#if defined SSI0
    while((SSI0_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};     // wait until SSI0 not busy/transmit FIFO empty
    SSI0_DR_R = 0x00;                                     // data out, garbage, just for synchronization
    while((SSI0_SR_R & SSI_SR_RNE) == 0) {};              // wait until response
    return SSI0_DR_R;                                     // read data
#elif defined SSI1
    while((SSI1_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};     // wait until SSI1 not busy/transmit FIFO empty
    SSI1_DR_R = 0x00;                                     // data out, garbage, just for synchronization
    while((SSI1_SR_R & SSI_SR_RNE) == 0) {};              // wait until response
    return SSI1_DR_R;                                     // read data
#elif defined SSI2
    while((SSI2_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};     // wait until SSI2 not busy/transmit FIFO empty
    SSI2_DR_R = 0x00;                                     // data out, garbage, just for synchronization
    while((SSI2_SR_R & SSI_SR_RNE) == 0) {};              // wait until response
    return SSI2_DR_R;                                     // read data
#elif defined SSI3
    while((SSI3_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};     // wait until SSI3 not busy/transmit FIFO empty
    SSI3_DR_R = 0x00;                                     // data out, garbage, just for synchronization
    while((SSI3_SR_R & SSI_SR_RNE) == 0) {};              // wait until response
    return SSI3_DR_R;                                     // read data
#endif
}

/**
 * SSI_write
 * ----------
 * @param  data  data to be written.
 */
void SSI_write(uint16_t data){
#if defined SSI0
    while ((SSI0_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};   // wait until SSI1 not busy/transmit FIFO empty
    SSI0_DR_R = data;                                    // write data
    while ((SSI0_SR_R & SSI_SR_RNE) == 0) {};            // wait until response
    uint16_t sync = SSI0_DR_R;                           // read byte of data, just for synchronization
#elif defined SSI1
    while ((SSI1_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};   // wait until SSI3 not busy/transmit FIFO empty
    SSI1_DR_R = data;                                    // write data
    while ((SSI1_SR_R & SSI_SR_RNE) == 0) {};            // wait until response
    uint16_t sync = SSI1_DR_R;                           // read byte of data, just for synchronization
#elif defined SSI2
    while ((SSI2_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};   // wait until SSI2 not busy/transmit FIFO empty
    SSI2_DR_R = data;                                    // write data
    while ((SSI2_SR_R & SSI_SR_RNE) == 0) {};            // wait until response
    uint16_t sync = SSI2_DR_R;                           // read byte of data, just for synchronization
#elif defined SSI3
    while ((SSI3_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};   // wait until SSI3 not busy/transmit FIFO empty
    SSI3_DR_R = data;                                    // write data
    while ((SSI3_SR_R & SSI_SR_RNE) == 0) {};            // wait until response
    uint16_t sync = SSI3_DR_R;                           // read byte of data, just for synchronization
#endif
}


