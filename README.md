# PN532 NFC Library for Texas Instruments TM4C123G MCU

## Introduction
This repository contains the driver software for [NXP PN532 NFC controller](https://www.nxp.com/docs/en/user-guide/141520.pdf) on [TI TM4C123G MCU](http://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf) and some example projects based on the driver software, as well as other necessary [addtional files](Projects/inc). I also include Keil uVision project file (the uvproj file in each project folder) so that if you are using ARM Keil uVision with your TM4C123G, you can just double click on the uVision project file and load to your MCU. If you are using some other IDEs, you need to organize all the source files by yourself. Hope this repository can help you integrate NFC functionalities into your projects with TM4C123G. 

## PN532 Module
Right now, any boards similar to the picture below are supported. This board design should be the most popular one on the market right now. Here is a really good [guide](https://dangerousthings.com/wp-content/uploads/PN532_Manual_V3-1.pdf) for this module from [**elechouse**](http://www.elechouse.com). Other variations of PN532 boards are not tested for now (since I dont have them) but they should work, if wired correctly. 

![PN532 Board](Images/PN532.png)

## Protocols
- [x] SSI/SPI: All SSI ports supported.
- [ ] I2C: Not supported.
- [ ] HSU: Not supported.

## API
Driver APIs please refer to [PN532.h](PN532/PN532.h). There are only 2 low level R/W APIs, *writeCommand* and *readResponse*. Regardless what protocol you are using, the APIs called by [PN532.c](PN532/PN532.c) are the same. This is achived by the preprocessor setting in [PN532_Setting.h](PN532_Setting.h). The setting not only enables users to turn on only the certain comunication protocol (SSI, I2C and HSU) they want to use, but also prevents from including the code for unused protocols (when SSI is used, only SSI code is included). This approach significantly reduces code size loaded into the precious ROM space on TM4C123G and allows reusable R/W APIs across all protocols (since it does not include function declaration from unused protocol, different protocol can have the same API function signatures without causing error). The setting file used in the projects locates in the [inc](Projects/inc) folder in [Projects](Projects) directory, instead of the one in the root folder. 

### API calling graph

    <ProjectName>.c →--[Calls APIs]--→ PN532.c →--(#if   defined SSI)---[Calls R/W APIs]--→ PN532_SSI.c
                  |                          ↳----(#elif defined I2C)---[Calls R/W APIs]--→ PN532_I2C.c
                  |                          ↳----(#elif defined HSU)---[Calls R/W APIs]--→ PN532_HSU.c
                  ↳----[Calls APIs]--→ Additional Files, ie UART.c, LED.c, PLL.c and etc.    

#### In the graph: 
- **\<ProjectName\>**: project file name placeholder.  

- **(condition)**: the calling path exists only if the condition inside the ( ) is true.

- **\[action\]**: action for current calling path.

## Projects

### ISO14443A(Mifare) Card Detection
#### [Mifare_Detect](Projects/PN532_Mifare_Detect_4C123)
> Detect an ISO14443A card and send its UID to serial output (UART via USB).

#### [Mifare_Detect_ST7735](Projects/PN532_Mifare_Detect_ST7735_4C123)
> Detect an ISO14443A card and output its UID to an [ST7735](https://www.adafruit.com/product/358) LCD.

### Mifare Classic

![Mifare_Classic_Format_FSM](Images/Mifare_Classic_Format_FSM.png)

#### [MifareClassic_Format_NDEF](Projects/PN532_MifareClassic_Format_NDEF_4C123)
> Format a Mifare Classic card from default format to NDEF format with customized content.

#### [MifareClassic_Update_NDEF](Projects/PN532_MifareClassic_Update_NDEF_4C123)
> Update the content in a NDEF formatted Mifare Classic card.

#### [MifareClassic_Reset_Default](Projects/PN532_MifareClassic_Reset_Default_4C123)
> Format a NDEF formatted Mifare Classic card to its default format.

#### [MifareClassic_Memory_Dump](Projects/PN532_MifareClassic_Memory_Dump_4C123)
> Dump all memory content of a Mifare Classic card.

## Copyright Note
This repository adpated and modified source code from [**elechouse**](http://www.elechouse.com) [PN532 driver for Arduino]( https://github.com/elechouse/PN532.git) which is based on [Seeed-Studio's PN532 Arduino driver](https://github.com/Seeed-Studio/PN532). I also include some addtional files from [ValvanoWareTM4C123](http://edx-org-utaustinx.s3.amazonaws.com/UT601x/ValvanoWareTM4C123.zip?dl=1), by [Dr. Jonathan Valvano](http://users.ece.utexas.edu/~valvano/), to support the [projects](Projects) in this repository. Some low level interface functions are inspired by examples in [ValvanoWareTM4C123](http://edx-org-utaustinx.s3.amazonaws.com/UT601x/ValvanoWareTM4C123.zip?dl=1) as well.
