/**
 * PN532_Setting.h
 * ----------
 * Zee Livermorium
 * Apr 14, 2018
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
#ifdef SSI
    #if 0           // set this to 1 to use SSI0
        #define SSI0
    #elif 0         // set this to 1 to use SSI1
        #define SSI1
        #if 1       // set to 1 for SSI1 on Port D, 0 on Port F
            #define SSI1D
        #else
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
