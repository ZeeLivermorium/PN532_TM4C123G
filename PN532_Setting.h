/*!
 * @file PN532_Setting.h
 * @brief setting for communications between TM4C123 and PN532.
 * ----------
 * @author Zee Livermorium
 * @date Apr 14, 2018
 */

#ifndef __PN532_SETTING_H__
#define __PN532_SETTING_H__


/*-- Protocol Selection --*/
#if 1           // set this to 1 to use SSI
    #define SSI
#elif 0         // set this to 1 to use I2C, not supported yet
    #define I2C
#elif 0         // set this to 1 to use HSU, not supported yet
    #define HSU
#endif


/*-- SSI Setting --*/
/*
 *  SSI0 A Conncection | SSI1 D Conncection | SSI1 F Conncection | SSI2 B Conncection | SSI3 D Conncection
 *  ------------------ | ------------------ | ------------------ | ------------------ | ------------------
 *  SCK  --------- PA2 | SCK  --------- PD0 | SCK  --------- PF2 | SCK  --------- PB4 | SCK  --------- PD0
 *  SS   --------- PA3 | SS   --------- PD1 | SS   --------- PF3 | SS   --------- PB5 | SS   --------- PD1
 *  MISO --------- PA4 | MISO --------- PD2 | MISO --------- PF0 | MISO --------- PB6 | MISO --------- PD2
 *  MOSI --------- PA5 | MOSI --------- PD3 | MOSI --------- PF1 | MOSI --------- PB7 | MOSI --------- PD3
 */
#ifdef SSI
    #if 0           // set this to 1 to use SSI0
        #define SSI0
    #elif 0         // set this to 1 to use SSI1
        #define SSI1
        #if 1       // set to 1 for SSI1 on Port D, 0 on Port F
            #define SSI1D
        #else       // **WARNING** do not use LEDs if you want to use Port F SSI, they overlap
            #define SSI1F
        #endif
    #elif 1         // set this to 1 to use SSI2
        #define SSI2
    #elif 0         // set this to 1 to use SSI3
        #define SSI3
    #endif
#endif


/*-- I2C Setting --*/
#ifdef I2C

#endif


/*-- HSU Setting --*/
#ifdef HSU

#endif


#endif
