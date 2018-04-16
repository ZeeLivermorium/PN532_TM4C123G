# PN532 NFC Library for Texas Instruments TM4C123G MCU

The API calling flow is as following:

    <Application>.c →---[Calls APIs]---→ PN532.c →---(#ifdef SSI)----[Calls R/W APIs]---→ PN532_SSI.c
                    |                            ↳---(#ifdef I2C)----[Calls R/W APIs]---→ PN532_I2C.c
                    |                            ↳---(#ifdef HSU)----[Calls R/W APIs]---→ PN532_HSU.c
                    ↳---[Calls APIs]---→ Additional Files, ie UART.c, LED.c, PLL.c and etc.                     
