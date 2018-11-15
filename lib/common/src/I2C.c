/*!
 * @file  I2C.c
 * @brief TM4C123G I2C API implementation.
 * ----------
 * For future development and updates, please follow this repository:
 * ----------
 * If you find any bug or problem, please create new issue or a pull request with a fix in the repository.
 * Or you can simply email me about the problem or bug at zeelivermorium@gmail.com
 * Much Appreciated!
 * ----------
 * @author Zee Livermorium
 * @date   Aug 4, 2018
 */

#include <stdint.h>
#include "I2C.h"
#include "tm4c123gh6pm.h"

#define MAXRETRIES 5           // number of receive attempts before giving up

/****************************************************
 *                                                  *
 *                   Initializer                    *
 *                                                  *
 ****************************************************/

/**
 * I2C_Init
 * ----------
 * @brief initialize a I2C module with corresponding setting parameters.
 */
void I2C_Init() {
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
    I2C0_MTPR_R = 39;                                      // configure for 100 kbps clock
    // 20 * (TPR + 1) * 12.5ns = 10us, with TPR=24
#elif defined I2C1
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
    I2C1_MTPR_R = 39;                                      // configure for 100 kbps clock
    // 20 * (TPR + 1) * 12.5ns = 10us, with TPR=24
#elif defined I2C2
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
    I2C2_MTPR_R = 39;                                      // configure for 100 kbps clock
    // 20 * (TPR + 1) * 12.5ns = 10us, with TPR=24
#elif defined I2C3
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
    I2C3_MTPR_R = 39 ;                                     // configure for 100 kbps clock
    // 20 * (TPR + 1) * 12.5ns = 10us, with TPR=24
#endif
}


/****************************************************
 *                                                  *
 *                     R/W API                      *
 *                                                  *
 ****************************************************/

/**
 * I2C_read
 * ----------
 * @param  deviceAddress  address of slave device.
 * @param  targetRegister target register of slave device.
 * @param  data           data address to store read data.
 * @param  count          number of bytes to be read.
 * ----------
 * @brief read 1 or more bytes from slave device.
 */
int I2C_read(uint8_t deviceAddress, uint8_t targetRegister, uint8_t* data, uint32_t count) {
    
    int retryCounter = 1;
    
#ifdef I2C0
    while (I2C0_MCS_R & I2C_MCS_BUSY) {};                            // wait for I2C ready
    I2C0_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;                // MSA[7:1] is slave address
    I2C0_MSA_R &= ~I2C_MSA_RS;                                       // MSA[0] is 0 for send
    
    I2C0_MDR_R = targetRegister & I2C_MDR_DATA_M;                    // prepare targetRegister
    I2C0_MCS_R = (I2C_MCS_STOP  |                                    // generate stop
                  I2C_MCS_START |                                    // generate start/restart
                  I2C_MCS_RUN);                                      // master enable
    while (I2C0_MCS_R & I2C_MCS_BUSY) {};                            // wait for transmission done
    // check error bits
    if((I2C0_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
        return (I2C0_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
    }
    
    switch (count) {
        case 1:
            do {
                while (I2C0_MCS_R & I2C_MCS_BUSY) {};                // wait for I2C ready
                I2C0_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;    // MSA[7:1] is slave address
                I2C0_MSA_R |= I2C_MSA_RS;                            // MSA[0] is 1 for receive
                
                I2C0_MCS_R = (I2C_MCS_STOP  |                        // generate stop
                              I2C_MCS_START |                        // generate start/restart
                              I2C_MCS_RUN);                          // master enable
                while (I2C0_MCS_R & I2C_MCS_BUSY) {};                // wait for transmission done
                retryCounter++;                                      // increment retry counter
            }                                                        // repeat if error
            while (((I2C0_MCS_R & (I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0) && (retryCounter <= MAXRETRIES));
            data[0] = (I2C0_MDR_R & I2C_MDR_DATA_M);                 // usually 0xFF on error
            
            break;
        case 2:
            do {
                while (I2C0_MCS_R & I2C_MCS_BUSY) {};                // wait for I2C ready
                I2C0_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;    // MSA[7:1] is slave address
                I2C0_MSA_R |= I2C_MSA_RS;                            // MSA[0] is 1 for receive
                
                I2C0_MCS_R = (I2C_MCS_ACK   |                        // positive data ack
                              I2C_MCS_START |                        // generate start/restart
                              I2C_MCS_RUN);                          // master enable
                while (I2C0_MCS_R & I2C_MCS_BUSY) {};                // wait for transmission done
                data[0] = (I2C0_MDR_R & I2C_MDR_DATA_M);             // most significant byte
                
                I2C0_MCS_R = (I2C_MCS_STOP |                         // generate stop
                              I2C_MCS_RUN);                          // master enable
                while (I2C0_MCS_R & I2C_MCS_BUSY) {};                // wait for transmission done
                data[1] = (I2C0_MDR_R & I2C_MDR_DATA_M);             // least significant byte
                retryCounter++;                                      // increment retry counter
            }                                                        // repeat if error
            while (((I2C0_MCS_R & (I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0) && (retryCounter <= MAXRETRIES));
            
            break;
            
        default:
            do {
                while (I2C0_MCS_R & I2C_MCS_BUSY) {};                // wait for I2C ready
                I2C0_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;    // MSA[7:1] is slave address
                I2C0_MSA_R |= I2C_MSA_RS;                            // MSA[0] is 1 for receive
                
                I2C0_MCS_R = (I2C_MCS_ACK   |                        // positive data ack
                              I2C_MCS_START |                        // generate start/restart
                              I2C_MCS_RUN);                          // master enable
                while (I2C0_MCS_R & I2C_MCS_BUSY) {};                // wait for transmission done
                data[0] = (I2C0_MDR_R & I2C_MDR_DATA_M);             // most significant byte
                
                
                for (int i = 1; i < count - 1; i++) {
                    I2C0_MCS_R = (I2C_MCS_ACK |                      // positive data ack
                                  I2C_MCS_RUN);                      // master enable
                    while (I2C0_MCS_R & I2C_MCS_BUSY) {};            // wait for transmission done
                    data[i] = (I2C0_MDR_R & I2C_MDR_DATA_M);         // read byte
                }
                
                I2C0_MCS_R = (I2C_MCS_STOP |                         // generate stop
                              I2C_MCS_RUN);                          // master enable
                while (I2C0_MCS_R & I2C_MCS_BUSY) {};                // wait for transmission done
                data[count - 1] = (I2C0_MDR_R & I2C_MDR_DATA_M);     // least significant byte
                retryCounter++;                                      // increment retry counter
            }                                                        // repeat if error
            while (((I2C0_MCS_R & (I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0) && (retryCounter <= MAXRETRIES));
            
            break;
    }
    
    return (I2C0_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
#elif defined I2C1
    while (I2C1_MCS_R & I2C_MCS_BUSY) {};                            // wait for I2C ready
    I2C1_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;                // MSA[7:1] is slave address
    I2C1_MSA_R &= ~I2C_MSA_RS;                                       // MSA[0] is 0 for send
    
    I2C1_MDR_R = targetRegister & I2C_MDR_DATA_M;                    // prepare targetRegister
    I2C1_MCS_R = (I2C_MCS_STOP  |                                    // generate stop
                  I2C_MCS_START |                                    // generate start/restart
                  I2C_MCS_RUN);                                      // master enable
    while (I2C1_MCS_R & I2C_MCS_BUSY) {};                            // wait for transmission done
    // check error bits
    if((I2C1_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
        return (I2C1_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
    }
    
    switch (count) {
        case 1:
            do {
                while (I2C1_MCS_R & I2C_MCS_BUSY) {};                // wait for I2C ready
                I2C1_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;    // MSA[7:1] is slave address
                I2C1_MSA_R |= I2C_MSA_RS;                            // MSA[0] is 1 for receive
                
                I2C1_MCS_R = (I2C_MCS_STOP  |                        // generate stop
                              I2C_MCS_START |                        // generate start/restart
                              I2C_MCS_RUN);                          // master enable
                while (I2C1_MCS_R & I2C_MCS_BUSY) {};                // wait for transmission done
                retryCounter++;                                      // increment retry counter
            }                                                        // repeat if error
            while (((I2C1_MCS_R & (I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0) && (retryCounter <= MAXRETRIES));
            data[0] = (I2C1_MDR_R & I2C_MDR_DATA_M);                 // usually 0xFF on error
            
            break;
        case 2:
            do {
                while (I2C1_MCS_R & I2C_MCS_BUSY) {};                // wait for I2C ready
                I2C1_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;    // MSA[7:1] is slave address
                I2C1_MSA_R |= I2C_MSA_RS;                            // MSA[0] is 1 for receive
                
                I2C1_MCS_R = (I2C_MCS_ACK   |                        // positive data ack
                              I2C_MCS_START |                        // generate start/restart
                              I2C_MCS_RUN);                          // master enable
                while (I2C1_MCS_R & I2C_MCS_BUSY) {};                // wait for transmission done
                data[0] = (I2C1_MDR_R & I2C_MDR_DATA_M);             // most significant byte
                
                I2C1_MCS_R = (I2C_MCS_STOP |                         // generate stop
                              I2C_MCS_RUN);                          // master enable
                while (I2C1_MCS_R & I2C_MCS_BUSY) {};                // wait for transmission done
                data[1] = (I2C1_MDR_R & I2C_MDR_DATA_M);             // least significant byte
                retryCounter++;                                      // increment retry counter
            }                                                        // repeat if error
            while (((I2C1_MCS_R & (I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0) && (retryCounter <= MAXRETRIES));
            
            break;
            
        default:
            do {
                while (I2C1_MCS_R & I2C_MCS_BUSY) {};                // wait for I2C ready
                I2C1_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;    // MSA[7:1] is slave address
                I2C1_MSA_R |= I2C_MSA_RS;                            // MSA[0] is 1 for receive
                
                I2C1_MCS_R = (I2C_MCS_ACK   |                        // positive data ack
                              I2C_MCS_START |                        // generate start/restart
                              I2C_MCS_RUN);                          // master enable
                while (I2C1_MCS_R & I2C_MCS_BUSY) {};                // wait for transmission done
                data[0] = (I2C1_MDR_R & I2C_MDR_DATA_M);             // most significant byte
                
                
                for (int i = 1; i < count - 1; i++) {
                    I2C1_MCS_R = (I2C_MCS_ACK |                      // positive data ack
                                  I2C_MCS_RUN);                      // master enable
                    while (I2C1_MCS_R & I2C_MCS_BUSY) {};            // wait for transmission done
                    data[i] = (I2C1_MDR_R & I2C_MDR_DATA_M);         // read byte
                }
                
                I2C1_MCS_R = (I2C_MCS_STOP |                         // generate stop
                              I2C_MCS_RUN);                          // master enable
                while (I2C1_MCS_R & I2C_MCS_BUSY) {};                // wait for transmission done
                data[count - 1] = (I2C1_MDR_R & I2C_MDR_DATA_M);     // least significant byte
                retryCounter++;                                      // increment retry counter
            }                                                        // repeat if error
            while (((I2C1_MCS_R & (I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0) && (retryCounter <= MAXRETRIES));
            
            break;
    }
    
    return (I2C1_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
#elif defined I2C2
    while (I2C2_MCS_R & I2C_MCS_BUSY) {};                            // wait for I2C ready
    I2C2_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;                // MSA[7:1] is slave address
    I2C2_MSA_R &= ~I2C_MSA_RS;                                       // MSA[0] is 0 for send
    
    I2C2_MDR_R = targetRegister & I2C_MDR_DATA_M;                    // prepare targetRegister
    I2C2_MCS_R = (I2C_MCS_STOP  |                                    // generate stop
                  I2C_MCS_START |                                    // generate start/restart
                  I2C_MCS_RUN);                                      // master enable
    while (I2C2_MCS_R & I2C_MCS_BUSY) {};                            // wait for transmission done
    // check error bits
    if((I2C2_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
        return (I2C2_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
    }
    
    switch (count) {
        case 1:
            do {
                while (I2C2_MCS_R & I2C_MCS_BUSY) {};                // wait for I2C ready
                I2C2_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;    // MSA[7:1] is slave address
                I2C2_MSA_R |= I2C_MSA_RS;                            // MSA[0] is 1 for receive
                
                I2C2_MCS_R = (I2C_MCS_STOP  |                        // generate stop
                              I2C_MCS_START |                        // generate start/restart
                              I2C_MCS_RUN);                          // master enable
                while (I2C2_MCS_R & I2C_MCS_BUSY) {};                // wait for transmission done
                retryCounter++;                                      // increment retry counter
            }                                                        // repeat if error
            while (((I2C2_MCS_R & (I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0) && (retryCounter <= MAXRETRIES));
            data[0] = (I2C2_MDR_R & I2C_MDR_DATA_M);                 // usually 0xFF on error
            
            break;
        case 2:
            do {
                while (I2C2_MCS_R & I2C_MCS_BUSY) {};                // wait for I2C ready
                I2C2_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;    // MSA[7:1] is slave address
                I2C2_MSA_R |= I2C_MSA_RS;                            // MSA[0] is 1 for receive
                
                I2C2_MCS_R = (I2C_MCS_ACK   |                        // positive data ack
                              I2C_MCS_START |                        // generate start/restart
                              I2C_MCS_RUN);                          // master enable
                while (I2C2_MCS_R & I2C_MCS_BUSY) {};                // wait for transmission done
                data[0] = (I2C2_MDR_R & I2C_MDR_DATA_M);             // most significant byte
                
                I2C2_MCS_R = (I2C_MCS_STOP |                         // generate stop
                              I2C_MCS_RUN);                          // master enable
                while (I2C2_MCS_R & I2C_MCS_BUSY) {};                // wait for transmission done
                data[1] = (I2C2_MDR_R & I2C_MDR_DATA_M);             // least significant byte
                retryCounter++;                                      // increment retry counter
            }                                                        // repeat if error
            while (((I2C2_MCS_R & (I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0) && (retryCounter <= MAXRETRIES));
            
            break;
            
        default:
            do {
                while (I2C2_MCS_R & I2C_MCS_BUSY) {};                // wait for I2C ready
                I2C2_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;    // MSA[7:1] is slave address
                I2C2_MSA_R |= I2C_MSA_RS;                            // MSA[0] is 1 for receive
                
                I2C2_MCS_R = (I2C_MCS_ACK   |                        // positive data ack
                              I2C_MCS_START |                        // generate start/restart
                              I2C_MCS_RUN);                          // master enable
                while (I2C2_MCS_R & I2C_MCS_BUSY) {};                // wait for transmission done
                data[0] = (I2C2_MDR_R & I2C_MDR_DATA_M);             // most significant byte
                
                
                for (int i = 1; i < count - 1; i++) {
                    I2C2_MCS_R = (I2C_MCS_ACK |                      // positive data ack
                                  I2C_MCS_RUN);                      // master enable
                    while (I2C2_MCS_R & I2C_MCS_BUSY) {};            // wait for transmission done
                    data[i] = (I2C2_MDR_R & I2C_MDR_DATA_M);         // read byte
                }
                
                I2C2_MCS_R = (I2C_MCS_STOP |                         // generate stop
                              I2C_MCS_RUN);                          // master enable
                while (I2C2_MCS_R & I2C_MCS_BUSY) {};                // wait for transmission done
                data[count - 1] = (I2C2_MDR_R & I2C_MDR_DATA_M);     // least significant byte
                retryCounter++;                                      // increment retry counter
            }                                                        // repeat if error
            while (((I2C2_MCS_R & (I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0) && (retryCounter <= MAXRETRIES));
            
            break;
    }
    
    return (I2C2_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
#elif defined I2C3
    while (I2C3_MCS_R & I2C_MCS_BUSY) {};                            // wait for I2C ready
    I2C3_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;                // MSA[7:1] is slave address
    I2C3_MSA_R &= ~I2C_MSA_RS;                                       // MSA[0] is 0 for send
    
    I2C3_MDR_R = targetRegister & I2C_MDR_DATA_M;                    // prepare targetRegister
    I2C3_MCS_R = (I2C_MCS_STOP  |                                    // generate stop
                  I2C_MCS_START |                                    // generate start/restart
                  I2C_MCS_RUN);                                      // master enable
    while (I2C3_MCS_R & I2C_MCS_BUSY) {};                            // wait for transmission done
    // check error bits
    if((I2C3_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
        return (I2C3_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
    }
    
    switch (count) {
        case 1:
            do {
                while (I2C3_MCS_R & I2C_MCS_BUSY) {};                // wait for I2C ready
                I2C3_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;    // MSA[7:1] is slave address
                I2C3_MSA_R |= I2C_MSA_RS;                            // MSA[0] is 1 for receive
                
                I2C3_MCS_R = (I2C_MCS_STOP  |                        // generate stop
                              I2C_MCS_START |                        // generate start/restart
                              I2C_MCS_RUN);                          // master enable
                while (I2C3_MCS_R & I2C_MCS_BUSY) {};                // wait for transmission done
                retryCounter++;                                      // increment retry counter
            }                                                        // repeat if error
            while (((I2C3_MCS_R & (I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0) && (retryCounter <= MAXRETRIES));
            data[0] = (I2C3_MDR_R & I2C_MDR_DATA_M);                 // usually 0xFF on error
            
            break;
        case 2:
            do {
                while (I2C3_MCS_R & I2C_MCS_BUSY) {};                // wait for I2C ready
                I2C3_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;    // MSA[7:1] is slave address
                I2C3_MSA_R |= I2C_MSA_RS;                            // MSA[0] is 1 for receive
                
                I2C3_MCS_R = (I2C_MCS_ACK   |                        // positive data ack
                              I2C_MCS_START |                        // generate start/restart
                              I2C_MCS_RUN);                          // master enable
                while (I2C3_MCS_R & I2C_MCS_BUSY) {};                // wait for transmission done
                data[0] = (I2C3_MDR_R & I2C_MDR_DATA_M);             // most significant byte
                
                I2C3_MCS_R = (I2C_MCS_STOP |                         // generate stop
                              I2C_MCS_RUN);                          // master enable
                while (I2C3_MCS_R & I2C_MCS_BUSY) {};                // wait for transmission done
                data[1] = (I2C3_MDR_R & I2C_MDR_DATA_M);             // least significant byte
                retryCounter++;                                      // increment retry counter
            }                                                        // repeat if error
            while (((I2C3_MCS_R & (I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0) && (retryCounter <= MAXRETRIES));
            
            break;
            
        default:
            do {
                while (I2C3_MCS_R & I2C_MCS_BUSY) {};                // wait for I2C ready
                I2C3_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;    // MSA[7:1] is slave address
                I2C3_MSA_R |= I2C_MSA_RS;                            // MSA[0] is 1 for receive
                
                I2C3_MCS_R = (I2C_MCS_ACK   |                        // positive data ack
                              I2C_MCS_START |                        // generate start/restart
                              I2C_MCS_RUN);                          // master enable
                while (I2C3_MCS_R & I2C_MCS_BUSY) {};                // wait for transmission done
                data[0] = (I2C3_MDR_R & I2C_MDR_DATA_M);             // most significant byte
                
                
                for (int i = 1; i < count - 1; i++) {
                    I2C3_MCS_R = (I2C_MCS_ACK |                      // positive data ack
                                  I2C_MCS_RUN);                      // master enable
                    while (I2C3_MCS_R & I2C_MCS_BUSY) {};            // wait for transmission done
                    data[i] = (I2C3_MDR_R & I2C_MDR_DATA_M);         // read byte
                }
                
                I2C3_MCS_R = (I2C_MCS_STOP |                         // generate stop
                              I2C_MCS_RUN);                          // master enable
                while (I2C3_MCS_R & I2C_MCS_BUSY) {};                // wait for transmission done
                data[count - 1] = (I2C3_MDR_R & I2C_MDR_DATA_M);     // least significant byte
                retryCounter++;                                      // increment retry counter
            }                                                        // repeat if error
            while (((I2C3_MCS_R & (I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0) && (retryCounter <= MAXRETRIES));
            
            break;
    }
    
    return (I2C3_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
#endif
}

/**
 * I2C_write
 * ----------
 * @param  deviceAddress  address of slave device.
 * @param  targetRegister target register of slave device.
 * @param  data          data address of data to be writen.
 * @param  count          number of bytes to be writen.
 * ----------
 * @brief write 1 or more bytes to slave device.
 */
int I2C_write(uint8_t deviceAddress, uint8_t targetRegister, uint8_t* data, uint32_t count) {
#ifdef I2C0
    while (I2C0_MCS_R & I2C_MCS_BUSY) {};                            // wait for I2C ready
    I2C0_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;                // MSA[7:1] is slave address
    I2C0_MSA_R &= ~I2C_MSA_RS;                                       // MSA[0] is 0 for send
    
    I2C0_MDR_R = targetRegister & I2C_MDR_DATA_M;                    // prepare targetRegister
    I2C0_MCS_R = (I2C_MCS_START |                                    // generate start/restart
                  I2C_MCS_RUN);                                      // master enable
    while (I2C0_MCS_R & I2C_MCS_BUSY) {};                            // wait for transmission done
    // check error bits
    if((I2C0_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
        I2C0_MCS_R = I2C_MCS_STOP;                                   // stop transmission
        // return error bits if nonzero
        return (I2C0_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
    }
    
    if (count == 1) {
        I2C0_MDR_R = data[0] & I2C_MDR_DATA_M;                       // prepare data byte
        I2C0_MCS_R = (I2C_MCS_STOP |                                 // generate stop
                      I2C_MCS_RUN);                                  // master enable
        while (I2C0_MCS_R & I2C_MCS_BUSY) {};                        // wait for transmission done
        // return error bits
        return (I2C0_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
        
    } else {
        for (int i = 0; i < count - 1; i++) {
            I2C0_MDR_R = data[i] & I2C_MDR_DATA_M;                   // prepare data byte
            I2C0_MCS_R = I2C_MCS_RUN;                                // master enable
            while (I2C0_MCS_R & I2C_MCS_BUSY) {};                    // wait for transmission done
            // check error bits
            if((I2C0_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
                I2C0_MCS_R = I2C_MCS_STOP;                           // stop transmission
                // return error bits if nonzero
                return (I2C0_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
            }
        }
        
        I2C0_MDR_R = data[count - 1] & I2C_MDR_DATA_M;               // prepare last byte
        I2C0_MCS_R = (I2C_MCS_STOP |                                 // generate stop
                      I2C_MCS_RUN);                                  // master enable
        while (I2C0_MCS_R & I2C_MCS_BUSY) {};                        // wait for transmission done
        // return error bits
        return (I2C0_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
    }
#elif defined I2C1
    while (I2C1_MCS_R & I2C_MCS_BUSY) {};                            // wait for I2C ready
    I2C1_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;                // MSA[7:1] is slave address
    I2C1_MSA_R &= ~I2C_MSA_RS;                                       // MSA[0] is 0 for send
    
    I2C1_MDR_R = targetRegister & I2C_MDR_DATA_M;                    // prepare targetRegister
    I2C1_MCS_R = (I2C_MCS_START |                                    // generate start/restart
                  I2C_MCS_RUN);                                      // master enable
    while (I2C1_MCS_R & I2C_MCS_BUSY) {};                            // wait for transmission done
    // check error bits
    if((I2C1_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
        I2C1_MCS_R = I2C_MCS_STOP;                                   // stop transmission
        // return error bits if nonzero
        return (I2C1_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
    }
    
    if (count == 1) {
        I2C1_MDR_R = data[0] & I2C_MDR_DATA_M;                       // prepare data byte
        I2C1_MCS_R = (I2C_MCS_STOP |                                 // generate stop
                      I2C_MCS_RUN);                                  // master enable
        while (I2C1_MCS_R & I2C_MCS_BUSY) {};                        // wait for transmission done
        // return error bits
        return (I2C1_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
        
    } else {
        for (int i = 0; i < count - 1; i++) {
            I2C1_MDR_R = data[i] & I2C_MDR_DATA_M;                   // prepare data byte
            I2C1_MCS_R = I2C_MCS_RUN;                                // master enable
            while (I2C1_MCS_R & I2C_MCS_BUSY) {};                    // wait for transmission done
            // check error bits
            if((I2C1_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
                I2C1_MCS_R = I2C_MCS_STOP;                           // stop transmission
                // return error bits if nonzero
                return (I2C1_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
            }
        }
        
        I2C1_MDR_R = data[count - 1] & I2C_MDR_DATA_M;               // prepare last byte
        I2C1_MCS_R = (I2C_MCS_STOP |                                 // generate stop
                      I2C_MCS_RUN);                                  // master enable
        while (I2C1_MCS_R & I2C_MCS_BUSY) {};                        // wait for transmission done
        // return error bits
        return (I2C1_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
    }
#elif defined I2C2
    while (I2C2_MCS_R & I2C_MCS_BUSY) {};                            // wait for I2C ready
    I2C2_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;                // MSA[7:1] is slave address
    I2C2_MSA_R &= ~I2C_MSA_RS;                                       // MSA[0] is 0 for send
    
    I2C2_MDR_R = targetRegister & I2C_MDR_DATA_M;                    // prepare targetRegister
    I2C2_MCS_R = (I2C_MCS_START |                                    // generate start/restart
                  I2C_MCS_RUN);                                      // master enable
    while (I2C2_MCS_R & I2C_MCS_BUSY) {};                            // wait for transmission done
    // check error bits
    if((I2C2_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
        I2C2_MCS_R = I2C_MCS_STOP;                                   // stop transmission
        // return error bits if nonzero
        return (I2C2_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
    }
    
    if (count == 1) {
        I2C2_MDR_R = data[0] & I2C_MDR_DATA_M;                       // prepare data byte
        I2C2_MCS_R = (I2C_MCS_STOP |                                 // generate stop
                      I2C_MCS_RUN);                                  // master enable
        while (I2C2_MCS_R & I2C_MCS_BUSY) {};                        // wait for transmission done
        // return error bits
        return (I2C2_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
        
    } else {
        for (int i = 0; i < count - 1; i++) {
            I2C2_MDR_R = data[i] & I2C_MDR_DATA_M;                   // prepare data byte
            I2C2_MCS_R = I2C_MCS_RUN;                                // master enable
            while (I2C2_MCS_R & I2C_MCS_BUSY) {};                    // wait for transmission done
            // check error bits
            if((I2C2_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
                I2C2_MCS_R = I2C_MCS_STOP;                           // stop transmission
                // return error bits if nonzero
                return (I2C2_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
            }
        }
        
        I2C2_MDR_R = data[count - 1] & I2C_MDR_DATA_M;               // prepare last byte
        I2C2_MCS_R = (I2C_MCS_STOP |                                 // generate stop
                      I2C_MCS_RUN);                                  // master enable
        while (I2C2_MCS_R & I2C_MCS_BUSY) {};                        // wait for transmission done
        // return error bits
        return (I2C2_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
    }
#elif defined I2C3
    while (I2C3_MCS_R & I2C_MCS_BUSY) {};                            // wait for I2C ready
    I2C3_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;                // MSA[7:1] is slave address
    I2C3_MSA_R &= ~I2C_MSA_RS;                                       // MSA[0] is 0 for send
    
    I2C3_MDR_R = targetRegister & I2C_MDR_DATA_M;                    // prepare targetRegister
    I2C3_MCS_R = (I2C_MCS_START |                                    // generate start/restart
                  I2C_MCS_RUN);                                      // master enable
    while (I2C3_MCS_R & I2C_MCS_BUSY) {};                            // wait for transmission done
    // check error bits
    if((I2C3_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
        I2C3_MCS_R = I2C_MCS_STOP;                                   // stop transmission
        // return error bits if nonzero
        return (I2C3_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
    }
    
    if (count == 1) {
        I2C3_MDR_R = data[0] & I2C_MDR_DATA_M;                       // prepare data byte
        I2C3_MCS_R = (I2C_MCS_STOP |                                 // generate stop
                      I2C_MCS_RUN);                                  // master enable
        while (I2C3_MCS_R & I2C_MCS_BUSY) {};                        // wait for transmission done
        // return error bits
        return (I2C3_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
        
    } else {
        for (int i = 0; i < count - 1; i++) {
            I2C3_MDR_R = data[i] & I2C_MDR_DATA_M;                   // prepare data byte
            I2C3_MCS_R = I2C_MCS_RUN;                                // master enable
            while (I2C3_MCS_R & I2C_MCS_BUSY) {};                    // wait for transmission done
            // check error bits
            if((I2C3_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
                I2C3_MCS_R = I2C_MCS_STOP;                           // stop transmission
                // return error bits if nonzero
                return (I2C3_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
            }
        }
        
        I2C3_MDR_R = data[count - 1] & I2C_MDR_DATA_M;               // prepare last byte
        I2C3_MCS_R = (I2C_MCS_STOP |                                 // generate stop
                      I2C_MCS_RUN);                                  // master enable
        while (I2C3_MCS_R & I2C_MCS_BUSY) {};                        // wait for transmission done
        // return error bits
        return (I2C3_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
    }
#endif
}

/**
 * I2C_read_byte
 * ----------
 * @param  deviceAddress  address of slave device.
 * @param  targetRegister target register of slave device.
 * @param  data           data address to store read data.
 * ----------
 * @brief read 1 byte from slave device.
 */
int I2C_read_byte(uint8_t deviceAddress, uint8_t targetRegister, uint8_t *data) {
    // read 1 byte
    return I2C_read(deviceAddress, targetRegister, data, 1);
}


/**
 * I2C_write_byte
 * ----------
 * @param  deviceAddress  address of slave device.
 * @param  targetRegister target register of slave device.
 * @param  data           data to be writen.
 * ----------
 * @brief write 1 byte to slave device.
 */
int I2C_write_byte(uint8_t deviceAddress, uint8_t targetRegister, uint8_t data) {
    // write 1 byte
    return I2C_write(deviceAddress, targetRegister, &data, 1);
}

/**
 * I2C_read_2_bytes
 * ----------
 * @param  deviceAddress  address of slave device.
 * @param  targetRegister target register of slave device.
 * @param  data           data address to store read data.
 * ----------
 * @brief read 2 bytes from slave device.
 */
int I2C_read_2_bytes(uint8_t deviceAddress, uint8_t targetRegister, uint8_t* data) {
    // read 2 bytes
    return I2C_read(deviceAddress, targetRegister, data, 2);
}

/**
 * I2C_write_2_bytes
 * ----------
 * @param  deviceAddress  address of slave device.
 * @param  targetRegister target register of slave device.
 * @param  data           data to be writen.
 * ----------
 * @brief write 2 bytes to slave device.
 */
int I2C_write_2_bytes(uint8_t deviceAddress, uint8_t targetRegister, uint8_t* data) {
    // write 2 bytes
    return I2C_write(deviceAddress, targetRegister, data, 2);
}

/**
 * I2C_read_4_bytes
 * ----------
 * @param  deviceAddress  address of slave device.
 * @param  targetRegister target register of slave device.
 * @param  data           data address to store read data.
 * ----------
 * @brief read 4 bytes from slave device.
 */
int I2C_read_4_bytes(uint8_t deviceAddress, uint8_t targetRegister, uint8_t* data) {
    // read 4 bytes
    return I2C_read(deviceAddress, targetRegister, data, 4);
}

/**
 * I2C_write_4_bytes
 * ----------
 * @param  deviceAddress  address of slave device.
 * @param  targetRegister target register of slave device.
 * @param  data           data to be writen.
 * ----------
 * @brief write 4 bytes to slave device.
 */
int I2C_write_4_bytes(uint8_t deviceAddress, uint8_t targetRegister, uint8_t* data) {
    // write 4 bytes
    return I2C_write(deviceAddress, targetRegister, data, 4);
}

