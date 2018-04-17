# PN532 NFC Library for Texas Instruments TM4C123G MCU

## Introduction
This repository contains the driver software for [NXP PN532 NFC controller](https://www.nxp.com/docs/en/user-guide/141520.pdf) on [TI TM4C123G MCU](http://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf) and some example projects based on the driver software, as well as other necessary [addtional files](Projects/inc). I also include Keil uVision project file (the uvproj file in each project folder) so that if you are using ARM Keil uVision with your TM4C123G, you can just double click on the uVision project file and load to your MCU. If you are using some other IDEs, you need to organize all the source files by yourself. Hope this repository can help you integrate NFC functionalities into your projects with TM4C123G. 

## Copyright Note 
This repository adpated and modified source code from [**elechouse**](http://www.elechouse.com) [PN532 driver for Arduino]( https://github.com/elechouse/PN532.git). I also include some addtional files from [ValvanoWareTM4C123](http://edx-org-utaustinx.s3.amazonaws.com/UT601x/ValvanoWareTM4C123.zip?dl=1), by [Dr. Jonathan Valvano](http://users.ece.utexas.edu/~valvano/), to support the [projects](Projects) in this repository. Some low level interface functions are inspired by examples in [ValvanoWareTM4C123](http://edx-org-utaustinx.s3.amazonaws.com/UT601x/ValvanoWareTM4C123.zip?dl=1) as well.

## API calling flow

    <ProjectName>.c →--[Calls APIs]--→ PN532.c →--(#if   defined SSI)---[Calls R/W APIs]--→ PN532_SSI.c
                  |                          ↳----(#elif defined I2C)---[Calls R/W APIs]--→ PN532_I2C.c
                  |                          ↳----(#elif defined HSU)---[Calls R/W APIs]--→ PN532_HSU.c
                  ↳----[Calls APIs]--→ Additional Files, ie UART.c, LED.c, PLL.c and etc.    
                    
**\<ProjectName\>**: project file name placeholder.   

**(condition)**: the calling path exists only if the condition inside the ( ) is true. This preprocessor [setting](PN532_Setting.h) enables users to turn on only the certain comunication protocol (SSI, I2C and HSU) they want to use. The program also only includes code corresponding to your [setting](PN532_Setting.h), which significantly reduced code size for the precious ROM space on TM4C123G, and enables [PN532.c](PN532/PN532.c) to use same low level R/W APIs across all protocols (regardless you are using SSI, I2C or HSU, [PN532.c](PN532/PN532.c) are calling the same APIs). The setting file used in projects is in the [inc](Projects/inc) folder in [Projects](Projects), instead of the one in the root folder.

**\[action\]**: action for current calling path.

