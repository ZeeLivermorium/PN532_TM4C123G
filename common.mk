#******************************************************************************
#
# common.mk - Definitions common to all makefiles.
#
# Copyright (c) 2005-2017 Texas Instruments Incorporated.  All rights reserved.
# Software License Agreement
# 
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
# 
#   Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# 
#   Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the  
#   distribution.
# 
#   Neither the name of Texas Instruments Incorporated nor the names of
#   its contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# This is part of revision 2.1.4.178 of the Tiva Firmware Development Package.
#
#******************************************************************************

#******************************************************************************
#
# Modified by Zee Lv
# Nov 6, 2018
#
#******************************************************************************

#******************************************************************************
#
# Get the operating system name.  If this is Cygwin, the .d files will be
# munged to convert c: into /cygdrive/c so that "make" will be happy with the
# auto-generated dependencies.
#
#******************************************************************************
os:=${shell uname -s}

#******************************************************************************
#
# The compiler to be used.
#
#******************************************************************************
ifndef COMPILER
COMPILER=gcc
endif

#******************************************************************************
#
# definitions for using gcc.
#
#******************************************************************************
ifeq (${COMPILER}, gcc)

#
# Get the prefix for the tools to use.  Use arm-stellaris-eabi if it exists,
# otherwise fall back to arm-none-eabi.
#
PREFIX:=${shell type arm-stellaris-eabi-gcc > /dev/null 2>&1 && \
         echo arm-stellaris-eabi || echo arm-none-eabi}


#******************************************************************************
#
# definitions for tools to be used.
#
#******************************************************************************

#
# The command for calling the compiler.
#
CC=${PREFIX}-gcc

#
# The command for calling gdb.
#
GDB=${PREFIX}-gdb

#
# The command for calling flash tool.
#
LM4FLASH = lm4flash

#
# The command for calling openocd.
#
OPENOCD = openocd

#******************************************************************************
#
# definitions for common paths to be used.
#
#******************************************************************************

BUILDPATH = build

#
# The location of the C compiler
# ARMGCC_ROOT is used by some makefiles that need to know where the compiler
# is installed.
#
ARMGCC_ROOT:=${shell dirname '${shell sh -c "which ${CC}"}'}/..

#******************************************************************************
#
# FLAGS
#
#******************************************************************************

#
# Set the compiler CPU/FPU options.
#
CPU=-mcpu=cortex-m4
FPU=-mfpu=fpv4-sp-d16 -mfloat-abi=hard

#
# The flags passed to the assembler.
#
AFLAGS=-mthumb \
       ${CPU}  \
       ${FPU}  \
       -MD

#
# The flags passed to the compiler.
#
CFLAGS=-mthumb             \
       ${CPU}              \
       ${FPU}              \
       -ffunction-sections \
       -fdata-sections     \
       -MD                 \
       -std=c99            \
       -Wall               \
       -pedantic           \
       -DPART_${PART}      \
       -c

#
# The command for calling the library archiver.
#
AR=${PREFIX}-ar

#
# The command for calling the linker.
#
LD=${PREFIX}-ld

#
# The flags passed to the linker.
#
LDFLAGS=--gc-sections

#
# Get the location of libgcc.a from the GCC front-end.
#
LIBGCC:=${shell ${CC} ${CFLAGS} -print-libgcc-file-name}

#
# Get the location of libc.a from the GCC front-end.
#
LIBC:=${shell ${CC} ${CFLAGS} -print-file-name=libc.a}

#
# Get the location of libm.a from the GCC front-end.
#
LIBM:=${shell ${CC} ${CFLAGS} -print-file-name=libm.a}

#
# The command for extracting images from the linked executables.
#
OBJCOPY=${PREFIX}-objcopy

#
# Tell the compiler to include debugging information if the DEBUG environment
# variable is set.
#
CFLAGS+=-g -D DEBUG -O0

#
# Add the tool specific CFLAGS.
#
CFLAGS+=${CFLAGSgcc}

#
# Add the include file paths to AFLAGS and CFLAGS.
#
AFLAGS+=${patsubst %,-I%,${subst :, ,${IPATH}}}
CFLAGS+=${patsubst %,-I%,${subst :, ,${IPATH}}}


#******************************************************************************
#
# Rule definitions
#
#******************************************************************************


#
# The rule for building the object file from each C source file.
#
${BUILDPATH}${SUFFIX}/%.o: %.c
	@if [ 'x${VERBOSE}' = x ];                                  \
	 then                                                       \
	     echo "  CC    ${<}";                                   \
	 else                                                       \
	     echo ${CC} ${CFLAGS} -D${BUILDPATH} -o ${@} ${<};      \
	 fi
	@${CC} ${CFLAGS} -D${BUILDPATH} -o ${@} ${<}
ifneq ($(findstring CYGWIN, ${os}), )
	@if [ -e ${@:.o=.d} ];                                      \
	then                                                        \
		sed -i -r 's/ ([A-Za-z]):/ \/cygdrive\/\1/g' ${@:.o=.d} ; \
	fi
endif

#
# The rule for building the object file from each assembly source file.
#
${BUILDPATH}${SUFFIX}/%.o: %.S
	@if [ 'x${VERBOSE}' = x ];                                  \
	 then                                                       \
	     echo "  AS    ${<}";                                   \
	 else                                                       \
	     echo ${CC} ${AFLAGS} -D${BUILDPATH} -o ${@} -c ${<};   \
	 fi
	@${CC} ${AFLAGS} -D${BUILDPATH} -o ${@} -c ${<}
ifneq ($(findstring CYGWIN, ${os}), )
	@if [ -e ${@:.o=.d} ];                                      \
	then                                                        \
		sed -i -r 's/ ([A-Za-z]):/ \/cygdrive\/\1/g' ${@:.o=.d} ; \
	fi
endif

#
# The rule for creating an object library.
#
${BUILDPATH}${SUFFIX}/%.a:
	@if [ 'x${VERBOSE}' = x ];     \
	 then                          \
	     echo "  AR    ${@}";      \
	 else                          \
	     echo ${AR} -cr ${@} ${^}; \
	 fi
	@${AR} -cr ${@} ${^}

#
# The rule for linking the application.
#
${BUILDPATH}${SUFFIX}/%.axf:
	@if [ 'x${SCATTERgcc_${notdir ${@:.axf=}}}' = x ];                    \
	 then                                                                 \
	     ldname="${ROOT}/gcc/standalone.ld";                              \
	 else                                                                 \
	     ldname="${SCATTERgcc_${notdir ${@:.axf=}}}";                     \
	 fi;                                                                  \
	 if [ 'x${VERBOSE}' = x ];                                            \
	 then                                                                 \
	     echo "  LD    ${@} ${LNK_SCP}";                                  \
	 else                                                                 \
	     echo ${LD} -T $${ldname}                                         \
	          --entry ${ENTRY_${notdir ${@:.axf=}}}                       \
	          ${LDFLAGSgcc_${notdir ${@:.axf=}}}                          \
	          ${LDFLAGS} -o ${@} $(filter %.o %.a, ${^})                  \
	          '${LIBM}' '${LIBC}' '${LIBGCC}';                            \
	 fi;                                                                  \
	${LD} -T $${ldname}                                                   \
	      --entry ${ENTRY_${notdir ${@:.axf=}}}                           \
	      ${LDFLAGSgcc_${notdir ${@:.axf=}}}                              \
	      ${LDFLAGS} -o ${@} $(filter %.o %.a, ${^})                      \
	      '${LIBM}' '${LIBC}' '${LIBGCC}'
	@${OBJCOPY} -O binary ${@} ${@:.axf=.bin}
endif

#
# To create the bin file, we need to make all, which creates both
# bin and axf files at once.
#
${BUILDPATH}/${PROJ_NAME}.bin:
	@if [ ! -e ${BUILDPATH}/${PROJ_NAME}.bin ]; then make all; fi;

#
# Flash depends on the bin file
#
flash: ${BUILDPATH}/$(PROJ_NAME).bin
	$(LM4FLASH) $<

#
# Debug depends on the axf, but if we put the dependency as axf here, it
# runs into an infinite loop for checking the existence of axf file from
# make all. Since bin file and axf file are created at the same time, we
# we can just move the dependency to the bin file.
#
debug: ${BUILDPATH}/$(PROJ_NAME).bin
	$(OPENOCD) --file board/ek-tm4c123gxl.cfg &
	$(GDB) ${BUILDPATH}/$(PROJ_NAME).axf

#
# The rule to clean out all the build products.
#
clean:
	@rm -rf ${BUILDPATH} ${wildcard *~}
	@rm *.o *.d

