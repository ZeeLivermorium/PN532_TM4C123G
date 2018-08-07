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

#include <stdint.h>
#include <stdarg.h>
#include "Serial.h"
#include "tm4c123gh6pm.h"

// U0Rx (VCP receive) connected to PA0
// U0Tx (VCP transmit) connected to PA1

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
void Serial_Init(void){
    /*-- UART0 and Port A Activation --*/
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;               // activate UART Module 0 clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;               // activate GPIO Port A clock
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R0) == 0) {};  // allow time for activating
    
    /*-- Port A Set Up --*/
    GPIO_PORTA_AFSEL_R |= 0x03;                            // enable alt funct on PA0, 1
    GPIO_PORTA_DEN_R |= 0x03;                              // enable digital I/O on PA0, 1
    GPIO_PORTA_PCTL_R &= ((~GPIO_PCTL_PA0_M) &             // clear bit fields for PA0
                          (~GPIO_PCTL_PA1_M));             // clear bit fields for PA1
    GPIO_PORTA_PCTL_R |= (GPIO_PCTL_PA0_U0RX |             // configure PA0 as SSI0CLK
                          GPIO_PCTL_PA1_U0TX);             // configure PA1 as SSI0TX
    GPIO_PORTA_AMSEL_R &= ~0x03;                           // disable analog funct on PA0, 1
    
    /*-- SSI0 Set Up --*/
    UART0_CTL_R &= ~UART_CTL_UARTEN;                       // disable UART0
    UART0_IBRD_R = 43;
    UART0_FBRD_R = 26;
    UART0_LCRH_R = (UART_LCRH_WLEN_8 | UART_LCRH_FEN);     // 8 bit word length (no parity bits, one stop bit, FIFOs)
    UART0_CTL_R |= (UART_CTL_UARTEN |                      // enable UART0
                    UART_CTL_RXE    |                      // enable UART0 RX
                    UART_CTL_TXE);                         // enable UART0 TX
}


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
char Serial_getChar(void){
    while((UART0_FR_R & UART_FR_RXFE) != 0);
    return((char)(UART0_DR_R & 0xFF));
}

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
uint32_t Serial_getUDec(void){
    uint32_t number=0, length=0;
    char character;
    character = Serial_getChar();
    while(character != CR){ // accepts until <enter> is typed
        // The next line checks that the input is a digit, 0-9.
        // If the character is not 0-9, it is ignored and not echoed
        if((character>='0') && (character<='9')) {
            number = 10*number+(character-'0');   // this line overflows if above 4294967295
            length++;
            Serial_putChar(character);
        }
        // If the input is a backspace, then the return number is
        // changed and a backspace is outputted to the screen
        else if((character==BS) && length){
            number /= 10;
            length--;
            Serial_putChar(character);
        }
        character = Serial_getChar();
    }
    return number;
}

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
uint32_t Serial_getUHex (void) {
    uint32_t number=0, digit, length=0;
    char character;
    character = Serial_getChar();
    while(character != CR){
        digit = 0x10; // assume bad
        if((character>='0') && (character<='9')){
            digit = character-'0';
        }
        else if((character>='A') && (character<='F')){
            digit = (character-'A')+0xA;
        }
        else if((character>='a') && (character<='f')){
            digit = (character-'a')+0xA;
        }
        // If the character is not 0-9 or A-F, it is ignored and not echoed
        if(digit <= 0xF){
            number = number*0x10+digit;
            length++;
            Serial_putChar(character);
        }
        // Backspace outputted and return value changed if a backspace is inputted
        else if((character==BS) && length){
            number /= 0x10;
            length--;
            Serial_putChar(character);
        }
        character = Serial_getChar();
    }
    return number;
}

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
void Serial_getString(char *bufPt, uint16_t max) {
    int length=0;
    char character;
    character = Serial_getChar();
    while(character != CR){
        if(character == BS){
            if(length){
                bufPt--;
                length--;
                Serial_putChar(BS);
            }
        }
        else if(length < max){
            *bufPt = character;
            bufPt++;
            length++;
            Serial_putChar(character);
        }
        character = Serial_getChar();
    }
    *bufPt = 0;
}


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
void Serial_putChar(char data){
    while((UART0_FR_R & UART_FR_TXFF) != 0);
    UART0_DR_R = data;
}

/**
 * Serial_putUDec
 * ----------
 * @param  number  32-bit number to be transferred.
 * ----------
 * @brief Output a 32-bit number in unsigned decimal format.
 */
void Serial_putUDec(uint32_t number) {
    // This function uses recursion to convert decimal number
    //   of unspecified length as an ASCII string
    if(number >= 10){
        Serial_putUDec(number/10);
        number = number % 10;
    }
    Serial_putChar(number + '0'); /* n is between 0 and 9 */
}

/**
 * Serial_putUHex
 * ----------
 * @param  number  32-bit number to be transferred.
 * ----------
 * @brief Output a 32-bit number in unsigned hexadecimal format
 */
void Serial_putUHex(uint32_t number){
    /*
     * This function uses recursion to convert the number of
     * unspecified length as an ASCII string.
     */
    if(number >= 0x10){
        Serial_putUHex(number/0x10);
        Serial_putUHex(number%0x10);
    } else{
        if(number < 0xA)Serial_putChar(number + '0');
        else Serial_putChar((number-0x0A) + 'A');
    }
}

/**
 * Serial_putString
 * ----------
 * @param  str  pointer to a NULL-terminated string to be transferred.
 * ----------
 * @brief Output String (NULL termination).
 */
void Serial_putString(char *str){
    while(*str){
        Serial_putChar(*str);
        str++;
    }
}

/**
 * Serial_putNewLine
 * ----------
 * @brief output new line.
 */
void Serial_putNewLine(void){
    Serial_putChar(CR);
    Serial_putChar(LF);
}

/**
 * Serial_print
 * ----------
 * @brief a mini version of c print for serial.
 */
void Serial_print(char* format, ...) {
    /* initializing arguments */
    va_list arg_list;
    va_start(arg_list, format);
    int32_t number;
    
    for (char* str_pt = format; *str_pt != '\0'; str_pt++) {
        if (*str_pt != '%') Serial_putChar(*str_pt);
        else {
            str_pt++;
            /* fetch and execute arguments */
            switch(*str_pt) {
                /* char */
                case 'c':
                    Serial_putChar(va_arg(arg_list, int));
                    break;
                case 'C':
                    Serial_putChar(va_arg(arg_list, int));
                    break;
                /* signed decimal */
                case 'd':
                    number = va_arg(arg_list, int32_t);
                    if (number < 0) {
                        Serial_putChar('-');
                        number = -number;
                    }
                    Serial_putUDec(number);
                    break;
                case 'D':
                    number = va_arg(arg_list, int32_t);
                    if (number < 0) {
                        Serial_putChar('-');
                        number = -number;
                    }
                    Serial_putUDec(number);
                    break;
                /* unsigned decimal */
                case 'u':
                    Serial_putUDec(va_arg(arg_list, uint32_t));
                    break;
                case 'U':
                    Serial_putUDec(va_arg(arg_list, uint32_t));
                    break;
                /* hexadecimal */
                case 'x':
                    Serial_putUHex(va_arg(arg_list, uint32_t));
                    break;
                case 'X':
                    Serial_putUHex(va_arg(arg_list, uint32_t));
                    break;
                /* string */
                case 's':
                    Serial_putString(va_arg(arg_list, char*));
                    break;
                case 'S':
                    Serial_putString(va_arg(arg_list, char*));
                    break;
            }
        }
    }
    /* closing argument list to necessary clean-up */
    va_end(arg_list);
}

/**
 * Serial_println
 * ----------
 * @brief a mini version of c println for serial.
 */
void Serial_println(char* format, ...) {
    /* initializing arguments */
    va_list arg_list;
    va_start(arg_list, format);
    int32_t number;
    
    for (char* str_pt = format; *str_pt != '\0'; str_pt++) {
        if (*str_pt != '%') Serial_putChar(*str_pt);
        else {
            str_pt++;
            /* fetch and execute arguments */
            switch(*str_pt) {
                /* char */
                case 'c':
                    Serial_putChar(va_arg(arg_list, int));
                    break;
                case 'C':
                    Serial_putChar(va_arg(arg_list, int));
                    break;
                /* signed integer */
                case 'd':
                    number = va_arg(arg_list, int32_t);
                    if (number < 0) {
                        Serial_putChar('-');
                        number = -number;
                    }
                    Serial_putUDec(number);
                    break;
                case 'D':
                    number = va_arg(arg_list, int32_t);
                    if (number < 0) {
                        Serial_putChar('-');
                        number = -number;
                    }
                    Serial_putUDec(number);
                    break;
                /* unsigned integer */
                case 'u':
                    Serial_putUDec(va_arg(arg_list, uint32_t));
                    break;
                case 'U':
                    Serial_putUDec(va_arg(arg_list, uint32_t));
                    break;
                /* hexadecimal */
                case 'x':
                    Serial_putUHex(va_arg(arg_list, uint32_t));
                    break;
                case 'X':
                    Serial_putUHex(va_arg(arg_list, uint32_t));
                    break;
                /* string */
                case 's':
                    Serial_putString(va_arg(arg_list, char*));
                    break;
                case 'S':
                    Serial_putString(va_arg(arg_list, char*));
                    break;
            }
        }
    }
    /* closing argument list to necessary clean-up */
    va_end(arg_list);
    Serial_putNewLine(); // new line
}

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
void Serial_PutHexAndASCII (const uint8_t *data, const uint32_t length) {
    for (uint8_t i = 0; i < length; i++) {
        if (data[i] < 0x10) Serial_print(" 0");
        else Serial_print(" ");
        Serial_print("%x", data[i]);
    }
    Serial_print("    ");
    for (uint8_t i = 0; i < length; i++) {
        char c = data[i];
        if (c <= 0x1f || c > 0x7f) Serial_print(".");
        else Serial_print("%c", c);
    }
    Serial_println("");
}

