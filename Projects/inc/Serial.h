/*!
 * @file Serial.c
 * @brief Serial I/O for TM4C123G using UART0.
 * ----------
 * Adapted code from UART.c from ValvanoWareTM4C123 by Dr. Jonathan Valvano.
 * You can find ValvanoWareTM4C123 at http://edx-org-utaustinx.s3.amazonaws.com/UT601x/ValvanoWareTM4C123.zip?dl=1
 * You can find more of his work at http://users.ece.utexas.edu/~valvano/
 * ----------
 * @author Zee Livermorium
 * @date Apr 21, 2018
 */

#ifndef __Serial_H__
#define __Serial_H__

#include <stdint.h>

// U0Rx (VCP receive) connected to PA0
// U0Tx (VCP transmit) connected to PA1

/* standard ASCII symbols */
#define CR   0x0D
#define LF   0x0A
#define BS   0x08
#define ESC  0x1B
#define SP   0x20
#define DEL  0x7F

/****************************************************
 *                                                  *
 *                   Initializer                    *
 *                                                  *
 ****************************************************/

/**
 * Serial_Init
 * ----------
 * @brief Initialize the UART for 115,200 baud rate (assuming 80 MHz UART clock),
 *        8 bit word length, no parity bits, one stop bit, FIFOs enabled.
 */
void Serial_Init(void);

/****************************************************
 *                                                  *
 *                  Read Functions                  *
 *                                                  *
 ****************************************************/

/**
 * Serial_getChar
 * ----------
 * @return ASCII code for key typed.
 * ----------
 * @brief Wait for new serial port input.
 */
char Serial_getChar(void);

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
uint32_t Serial_getUHex (void);

/**
 * Serial_getString
 * ----------
 * @param  bufPt  pointer to store output.
 * @param  max    size of buffer.
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


/****************************************************
 *                                                  *
 *                 Write Functions                  *
 *                                                  *
 ****************************************************/

/**
 * Serial_putChar
 * ----------
 * @param  data  an 8-bit ASCII character to be transferred.
 * ----------
 * @brief Output 8-bit to serial port.
 */
void Serial_putChar(char data);

/**
 * Serial_putUDec
 * ----------
 * @param  number  32-bit number to be transferred.
 * ----------
 * @brief Output a 32-bit number in unsigned decimal format.
 */
void Serial_putUDec(uint32_t number);

/**
 * Serial_putUHex
 * ----------
 * @param  number  32-bit number to be transferred.
 * ----------
 * @brief Output a 32-bit number in unsigned hexadecimal format
 */
void Serial_putUHex(uint32_t number);

/**
 * Serial_putString
 * ----------
 * @param  str  pointer to a NULL-terminated string to be transferred.
 * ----------
 * @brief Output String (NULL termination).
 */
void Serial_putString(char *str);

/**
 * Serial_putNewLine
 * ----------
 * @brief output new line.
 */
void Serial_putNewLine(void);

/**
 * Serial_print
 * ----------
 * @brief a mini version of c print for serial.
 */
void Serial_print(char* format, ...);

/**
 * Serial_println
 * ----------
 * @brief a mini version of c println for serial.
 */
void Serial_println(char* format, ...);

/**
 * Serial_PutHexAndASCII
 * ----------
 * @param  data      Pointer to the data
 * @param  numBytes  Data length in bytes
 * ----------
 * @brief  Prints a hexadecimal value in plain characters, along with
 *         the char equivalents in the following format
 *
 *         00 00 00 00 00 00  ......
 */
void Serial_PutHexAndASCII (const uint8_t *data, const uint32_t length);
#endif

