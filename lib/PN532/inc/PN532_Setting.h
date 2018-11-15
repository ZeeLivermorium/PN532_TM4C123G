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
    #define PN532_SSI
#elif 0         // set this to 1 to use I2C, not supported yet
    #define PN532_I2C
#elif 0         // set this to 1 to use UART, not supported yet
    #define PN532_UART
#endif



/*-- I2C Setting --*/
/*
 *  I2C0 Conncection | I2C1 Conncection | I2C2 Conncection | I2C3 Conncection
 *  ---------------- | ---------------- | ---------------- | ----------------
 *  SCL -------- PB2 | SCL -------- PA6 | SCL -------- PE4 | SCL -------- PD0
 *  SDA -------- PB3 | SDA -------- PA7 | SDA -------- PE5 | SDA -------- PD1
 */
#ifdef PN532_I2C
    #if 1           // set this to 1 to use I2C0
        #define I2C0
    #elif 0         // set this to 1 to use I2C1
        #define I2C1
    #elif 0         // set this to 1 to use I2C2
        #define I2C2
    #else           // I2C3 if all 0 above
        #define I2C3
    #endif
#endif


/*-- UART Setting --*/
/*
 *  UART0 Conncection | UART1 Conncection | UART2 Conncection | UART3 Conncection
 *  ----------------- | ----------------- | ----------------- | -----------------
 *  RX ---------- PA0 | RX ---------- PB0 | RX ---------- PD6 | RX ---------- PC6
 *  TX ---------- PA1 | TX ---------- PB1 | TX ---------- PD7 | TX ---------- PC7
 *  -----------------------------------------------------------------------------
 *  UART4 Conncection | UART5 Conncection | UART6 Conncection | UART7 Conncection
 *  ----------------- | ----------------- | ----------------- | -----------------
 *  RX ---------- PC4 | RX ---------- PE4 | RX ---------- PD4 | RX ---------- PE0
 *  TX ---------- PC5 | TX ---------- PE5 | TX ---------- PD5 | TX ---------- PE1
 */
#ifdef PN532_UART
    #if 1           // set this to 1 to use UART0
        #define UART0  // **WARNING** do not use UART0 if you want to use Serial I/O
    #elif 0         // set this to 1 to use UART1
        #define UART1
    #elif 0         // set this to 1 to use UART2
        #define UART2
    #elif 0         // set this to 1 to use UART3
        #define UART3
    #elif 0         // set this to 1 to use UART4
        #define UART4
    #elif 0         // set this to 1 to use UART5
        #define UART5
    #elif 0         // set this to 1 to use UART6
        #define UART6
    #else           // UART7 if all 0 above
        #define UART7
    #endif
#endif


#endif
