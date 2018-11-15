/*!
 * SSI.h
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


#ifndef __SSI_H__
#define __SSI_H__

#include <stdint.h>

/*
 *  SSI0 A Conncection | SSI1 D Conncection | SSI1 F Conncection | SSI2 B Conncection | SSI3 D Conncection
 *  ------------------ | ------------------ | ------------------ | ------------------ | ------------------
 *  SCK  --------- PA2 | SCK  --------- PD0 | SCK  --------- PF2 | SCK  --------- PB4 | SCK  --------- PD0
 *  SS   --------- PA3 | SS   --------- PD1 | SS   --------- PF3 | SS   --------- PB5 | SS   --------- PD1
 *  MISO --------- PA4 | MISO --------- PD2 | MISO --------- PF0 | MISO --------- PB6 | MISO --------- PD2
 *  MOSI --------- PA5 | MOSI --------- PD3 | MOSI --------- PF1 | MOSI --------- PB7 | MOSI --------- PD3
 */
#if 0           // set this to 1 to use SSI0
    #define SSI0
#elif 0         // set this to 1 to use SSI1
    #define SSI1
    #if 1       // set to 1 for SSI1 on Port D, 0 on Port F
        #define SSI1D
    #else       // **WARNING** do not use on board LEDs if you want to use Port F SSI, they overlap
        #define SSI1F
    #endif
#elif 1         // set this to 1 to use SSI2
    #define SSI2
#else           // SSI3 if all 0 above
    #define SSI3
#endif

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
uint8_t reverseBitOrder(uint8_t byte);

/****************************************************
 *                                                  *
 *                   SS Functions                   *
 *                                                  *
 ****************************************************/

void SS_HIGH (void);

void SS_LOW (void);


/****************************************************
 *                                                  *
 *                   Initializer                    *
 *                                                  *
 ****************************************************/

/**
 * SSI_Init
 * ----------
 * @brief initialize SSI with corresponding setting parameters.
 */
void SSI_Init(void);


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
uint16_t SSI_read (void);

/**
 * SSI_write
 * ----------
 * @param  data  data to be written.
 */
void SSI_write(uint16_t data);

#endif

