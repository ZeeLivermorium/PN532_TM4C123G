// UART.h
// Runs on LM3S811, LM3S1968, LM3S8962, LM4F120, TM4C123
// Simple device driver for the UART.
// Daniel Valvano
// May 30, 2014
// Modified by EE345L students Charlie Gough && Matt Hawk
// Modified by EE345M students Agustinus Darmawan && Mingjie Qiu

/* This example accompanies the book
 "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
 ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
 Program 4.12, Section 4.9.4, Figures 4.26 and 4.40
 
 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
 You may use, edit, run or distribute this file
 as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// U0Rx (VCP receive) connected to PA0
// U0Tx (VCP transmit) connected to PA1

// standard ASCII symbols
#define CR   0x0D
#define LF   0x0A
#define BS   0x08
#define ESC  0x1B
#define SP   0x20
#define DEL  0x7F

/**
 * Serial_Init
 * ----------
 * @brief Initialize the UART for 115,200 baud rate (assuming 80 MHz UART clock),
 *        8 bit word length, no parity bits, one stop bit, FIFOs enabled.
 */
void Serial_Init(void);

/**
 * Serial_print
 * ----------
 * @brief a mini version of c print for serial.
 */
void Serial_print(char* format, ...);

/**
 * Serial_getChar
 * ----------
 * @return ASCII code for key typed.
 * ----------
 * @brief Wait for new serial port input.
 */
char Serial_getChar(void);

/**
 * Serial_putChar
 * ----------
 * @param  data  an 8-bit ASCII character to be transferred.
 * ----------
 * @brief Output 8-bit to serial port.
 */
void Serial_putChar(char data);

/**
 * Serial_getUDec
 * ----------
 * @return 32-bit unsigned number.
 * ----------
 * @brief InUDec accepts ASCII input in unsigned decimal format
 *        and converts to a 32-bit unsigned number
 *        valid range is 0 to 4294967295 (2^32-1).
 * ----------
 * @warning If you enter a number above 4294967295, it will return an incorrect value.
 *          Backspace will remove last digit typed.
 */
uint32_t Serial_getUDec(void);

/**
 * Serial_putUDec
 * ----------
 * @param  number  32-bit number to be transferred.
 * ----------
 * @brief Output a 32-bit number in unsigned decimal format.
 */
void Serial_putUDec(uint32_t number);

/**
 * Serial_getUHex
 * ----------
 * @return 32-bit unsigned number.
 * ----------
 * @brief Accepts ASCII input in unsigned hexadecimal( format.
 * ----------
 * @warning No '$' or '0x' need be entered, just the 1 to 8 hex digits.
 *          It will convert lower case a-f to uppercase A-F and converts to
 *          a 16 bit unsigned number value range is 0 to FFFFFFFF.
 *          If you enter a number above FFFFFFFF, it will return an incorrect value.
 *          Backspace will remove last digit typed.
 */
uint32_t Serial_getUHex(void);

/**
 * Serial_putUhex
 * ----------
 * @param  number  32-bit number to be transferred.
 * ----------
 * @brief Output a 32-bit number in unsigned hexadecimal format
 */
void Serial_putUhex(uint32_t number);

/**
 * Serial_putString
 * ----------
 * @param  str  pointer to a NULL-terminated string to be transferred.
 * ----------
 * @brief Output String (NULL termination).
 */
void Serial_putString(char *str);

/**
 * Serial_getString
 * ----------
 * @param  bufPt  pointer to store output.
 * @param  max    size of buffer
 * ----------
 * @brief Accepts ASCII characters from the serial port
 *        and adds them to a string until <enter> is typed
 *        or until max length of the string is reached.
 *        It echoes each character as it is inputted.
 *        If a backspace is inputted, the string is modified
 *        and the backspace is echoed. Terminates the string
 *        with a null character uses busy-waiting
 *        synchronization on RDRF
 */
void Serial_getString(char *bufPt, uint16_t max);

/**
 * Serial_putNewLine
 * ----------
 * @brief output new line.
 */
void Serial_putNewLine(void);


