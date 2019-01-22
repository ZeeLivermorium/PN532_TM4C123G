#******************************************************************************
#
# Makefile - Rules for building this project.
#
# Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
# Software License Agreement
# 
# Texas Instruments (TI) is supplying this software for use solely and
# exclusively on TI's microcontroller products. The software is owned by
# TI and/or its suppliers, and is protected under applicable copyright
# laws. You may not combine this software with "viral" open-source
# software in order to form a larger program.
# 
# THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
# NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
# NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
# CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES, FOR ANY REASON WHATSOEVER.
# 
# This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
#
#******************************************************************************

#******************************************************************************
#
# Modified by Zee Lv
# Nov 6, 2018
#
#******************************************************************************

#
# Defines project name
#
PROJ_NAME=PN532_PCD_ISO14443A_Detect_ST7735

#
# Defines the part type that this project uses.
#
PART=TM4C123GH6PM

#
# The root directory for zEEware.
#
ROOT=../..

#
# Include the common make definitions.
#
include ${ROOT}/common.mk

#
# Where to find header files that do not live in the source directory.
#
IPATH += \
	${ROOT}/lib/common/inc          \
	${ROOT}/lib/_tm4c               \
	${ROOT}/lib/NFC/PN532/PN532/inc \
	${ROOT}/lib/LCD/ST7735/inc      \

#
# The default rule
#
all: ${BUILDPATH}
all: ${BUILDPATH}/$(PROJ_NAME).axf

#
# The rule to create the target directory.
#
${BUILDPATH}:
	@mkdir -p ${BUILDPATH}

#
# Rules for building. 
#
${BUILDPATH}/$(PROJ_NAME).axf: main.o
${BUILDPATH}/$(PROJ_NAME).axf: ${ROOT}/lib/_tm4c/startup.o
${BUILDPATH}/$(PROJ_NAME).axf: ${ROOT}/lib/_tm4c/libdriver.a
${BUILDPATH}/$(PROJ_NAME).axf: ${ROOT}/lib/_tm4c/tm4c123gxl.ld
${BUILDPATH}/$(PROJ_NAME).axf: ${ROOT}/lib/common/src/PLL.o
SCATTERgcc_$(PROJ_NAME)=${ROOT}/lib/_tm4c/tm4c123gxl.ld
ENTRY_$(PROJ_NAME)=ResetISR
CFLAGSgcc=-DTARGET_IS_TM4C123_RB1

# Add lib object files you needed below
${BUILDPATH}/$(PROJ_NAME).axf: ${ROOT}/lib/common/src/SSI.o
${BUILDPATH}/$(PROJ_NAME).axf: ${ROOT}/lib/common/src/Serial.o
${BUILDPATH}/$(PROJ_NAME).axf: ${ROOT}/lib/LCD/ST7735/src/ST7735.o
${BUILDPATH}/$(PROJ_NAME).axf: ${ROOT}/lib/NFC/PN532/PN532/src/PN532.o
${BUILDPATH}/$(PROJ_NAME).axf: ${ROOT}/lib/NFC/PN532/PN532/src/PN532_SSI.o

#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${BUILDPATH}/*.d} __dummy__
endif

