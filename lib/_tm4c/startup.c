//*****************************************************************************
//
// startup_gcc.c - Startup code for use with GNU tools.
//
// Copyright (c) 2012-2016 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.3.156 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include "hw_nvic.h"
#include "hw_types.h"

//*****************************************************************************
//
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
void ResetISR(void);
static void NmiSR(void);
static void FaultISR(void);
static void IntDefaultHandler(void);
static void NMI_Handler(void);
static void HardFault_Handler(void);
static void MemManage_Handler(void);
static void BusFault_Handler(void);
static void UsageFault_Handler(void);


extern void SVC_Handler(void) __attribute__((weak));                // SVCall Handler
extern void DebugMon_Handler(void) __attribute__((weak));           // Debug Monitor Handler
extern void PendSV_Handler(void) __attribute__((weak));             // PendSV Handler
extern void SysTick_Handler(void) __attribute__((weak));            // SysTick Handler
extern void GPIOPortA_Handler(void) __attribute__((weak));          // GPIO Port A
extern void GPIOPortB_Handler(void) __attribute__((weak));          // GPIO Port B
extern void GPIOPortC_Handler(void) __attribute__((weak));          // GPIO Port C
extern void GPIOPortD_Handler(void) __attribute__((weak));          // GPIO Port D
extern void GPIOPortE_Handler(void) __attribute__((weak));          // GPIO Port E
extern void UART0_Handler(void) __attribute__((weak));              // UART0 Rx and Tx
extern void UART1_Handler(void) __attribute__((weak));              // UART1 Rx and Tx
extern void SSI0_Handler(void) __attribute__((weak));               // SSI0 Rx and Tx
extern void I2C0_Handler(void) __attribute__((weak));               // I2C0 Master and Slave
extern void PWM0Fault_Handler(void) __attribute__((weak));          // PWM 0 Fault
extern void PWM0Generator0_Handler(void) __attribute__((weak));     // PWM 0 Generator 0
extern void PWM0Generator1_Handler(void) __attribute__((weak));     // PWM 0 Generator 1
extern void PWM0Generator2_Handler(void) __attribute__((weak));     // PWM 0 Generator 2
extern void Quadrature0_Handler(void) __attribute__((weak));        // Quadrature Encoder 0
extern void ADC0Seq0_Handler(void) __attribute__((weak));           // ADC0 Sequence 0
extern void ADC0Seq1_Handler(void) __attribute__((weak));           // ADC0 Sequence 1
extern void ADC0Seq2_Handler(void) __attribute__((weak));           // ADC0 Sequence 2
extern void ADC0Seq3_Handler(void) __attribute__((weak));           // ADC0 Sequence 3
extern void WDT_Handler(void) __attribute__((weak));                // Watchdog
extern void Timer0A_Handler(void) __attribute__((weak));            // Timer 0 subtimer A
extern void Timer0B_Handler(void) __attribute__((weak));            // Timer 0 subtimer B
extern void Timer1A_Handler(void) __attribute__((weak));            // Timer 1 subtimer A
extern void Timer1B_Handler(void) __attribute__((weak));            // Timer 1 subtimer B
extern void Timer2A_Handler(void) __attribute__((weak));            // Timer 2 subtimer A
extern void Timer2B_Handler(void) __attribute__((weak));            // Timer 2 subtimer B
extern void Comp0_Handler(void) __attribute__((weak));              // Analog Comp 0
extern void Comp1_Handler(void) __attribute__((weak));              // Analog Comp 1
extern void Comp2_Handler(void) __attribute__((weak));              // Analog Comp 2
extern void SysCtl_Handler(void) __attribute__((weak));             // System Control
extern void FlashCtl_Handler(void) __attribute__((weak));           // Flash Control
extern void GPIOPortF_Handler(void) __attribute__((weak));          // GPIO Port F
extern void GPIOPortG_Handler(void) __attribute__((weak));          // GPIO Port G
extern void GPIOPortH_Handler(void) __attribute__((weak));          // GPIO Port H
extern void UART2_Handler(void) __attribute__((weak));              // UART2 Rx and Tx
extern void SSI1_Handler(void) __attribute__((weak));               // SSI1 Rx and Tx
extern void Timer3A_Handler(void) __attribute__((weak));            // Timer 3 subtimer A
extern void Timer3B_Handler(void) __attribute__((weak));            // Timer 3 subtimer B
extern void I2C1_Handler(void) __attribute__((weak));               // I2C1 Master and Slave
extern void Quadrature1_Handler(void) __attribute__((weak));        // Quadrature Encoder 1
extern void CAN0_Handler(void) __attribute__((weak));               // CAN0
extern void CAN1_Handler(void) __attribute__((weak));               // CAN1
extern void CAN2_Handler(void) __attribute__((weak));               // CAN2
extern void Ethernet_Handler(void) __attribute__((weak));           // Ethernet
extern void Hibernate_Handler(void) __attribute__((weak));          // Hibernate
extern void USB0_Handler(void) __attribute__((weak));               // USB0
extern void PWM0Generator3_Handler(void) __attribute__((weak));     // PWM 0 Generator 3
extern void uDMA_Handler(void) __attribute__((weak));               // uDMA Software Transfer
extern void uDMA_Error(void) __attribute__((weak));                 // uDMA Error
extern void ADC1Seq0_Handler(void) __attribute__((weak));           // ADC1 Sequence 0
extern void ADC1Seq1_Handler(void) __attribute__((weak));           // ADC1 Sequence 1
extern void ADC1Seq2_Handler(void) __attribute__((weak));           // ADC1 Sequence 2
extern void ADC1Seq3_Handler(void) __attribute__((weak));           // ADC1 Sequence 3
extern void I2S0_Handler(void) __attribute__((weak));               // I2S0
extern void ExtBus_Handler(void) __attribute__((weak));             // External Bus Interface 0
extern void GPIOPortJ_Handler(void) __attribute__((weak));          // GPIO Port J
extern void GPIOPortK_Handler(void) __attribute__((weak));          // GPIO Port K
extern void GPIOPortL_Handler(void) __attribute__((weak));          // GPIO Port L
extern void SSI2_Handler(void) __attribute__((weak));               // SSI2 Rx and Tx
extern void SSI3_Handler(void) __attribute__((weak));               // SSI3 Rx and Tx
extern void UART3_Handler(void) __attribute__((weak));              // UART3 Rx and Tx
extern void UART4_Handler(void) __attribute__((weak));              // UART4 Rx and Tx
extern void UART5_Handler(void) __attribute__((weak));              // UART5 Rx and Tx
extern void UART6_Handler(void) __attribute__((weak));              // UART6 Rx and Tx
extern void UART7_Handler(void) __attribute__((weak));              // UART7 Rx and Tx
extern void I2C2_Handler(void) __attribute__((weak));               // I2C2 Master and Slave
extern void I2C3_Handler(void) __attribute__((weak));               // I2C3 Master and Slave
extern void Timer4A_Handler(void) __attribute__((weak));            // Timer 4 subtimer A
extern void Timer4B_Handler(void) __attribute__((weak));            // Timer 4 subtimer B
extern void Timer5A_Handler(void) __attribute__((weak));            // Timer 5 subtimer A
extern void Timer5B_Handler(void) __attribute__((weak));            // Timer 5 subtimer B
extern void WideTimer0A_Handler(void) __attribute__((weak));        // Wide Timer 0 subtimer A
extern void WideTimer0B_Handler(void) __attribute__((weak));        // Wide Timer 0 subtimer B
extern void WideTimer1A_Handler(void) __attribute__((weak));        // Wide Timer 1 subtimer A
extern void WideTimer1B_Handler(void) __attribute__((weak));        // Wide Timer 1 subtimer B
extern void WideTimer2A_Handler(void) __attribute__((weak));        // Wide Timer 2 subtimer A
extern void WideTimer2B_Handler(void) __attribute__((weak));        // Wide Timer 2 subtimer B
extern void WideTimer3A_Handler(void) __attribute__((weak));        // Wide Timer 3 subtimer A
extern void WideTimer3B_Handler(void) __attribute__((weak));        // Wide Timer 3 subtimer B
extern void WideTimer4A_Handler(void) __attribute__((weak));        // Wide Timer 4 subtimer A
extern void WideTimer4B_Handler(void) __attribute__((weak));        // Wide Timer 4 subtimer B
extern void WideTimer5A_Handler(void) __attribute__((weak));        // Wide Timer 5 subtimer A
extern void WideTimer5B_Handler(void) __attribute__((weak));        // Wide Timer 5 subtimer B
extern void FPU_Handler(void) __attribute__((weak));                // FPU
extern void PECI0_Handler(void) __attribute__((weak));              // PECI 0
extern void LPC0_Handler(void) __attribute__((weak));               // LPC 0
extern void I2C4_Handler(void) __attribute__((weak));               // I2C4 Master and Slave
extern void I2C5_Handler(void) __attribute__((weak));               // I2C5 Master and Slave
extern void GPIOPortM_Handler(void) __attribute__((weak));          // GPIO Port M
extern void GPIOPortN_Handler(void) __attribute__((weak));          // GPIO Port N
extern void Quadrature2_Handler(void) __attribute__((weak));        // Quadrature Encoder 2
extern void Fan0_Handler(void) __attribute__((weak));               // Fan 0
extern void GPIOPortP_Handler(void) __attribute__((weak));          // GPIO Port P (Summary or P0)
extern void GPIOPortP1_Handler(void) __attribute__((weak));         // GPIO Port P1
extern void GPIOPortP2_Handler(void) __attribute__((weak));         // GPIO Port P2
extern void GPIOPortP3_Handler(void) __attribute__((weak));         // GPIO Port P3
extern void GPIOPortP4_Handler(void) __attribute__((weak));         // GPIO Port P4
extern void GPIOPortP5_Handler(void) __attribute__((weak));         // GPIO Port P5
extern void GPIOPortP6_Handler(void) __attribute__((weak));         // GPIO Port P6
extern void GPIOPortP7_Handler(void) __attribute__((weak));         // GPIO Port P7
extern void GPIOPortQ_Handler(void) __attribute__((weak));          // GPIO Port Q (Summary or Q0)
extern void GPIOPortQ1_Handler(void) __attribute__((weak));         // GPIO Port Q1
extern void GPIOPortQ2_Handler(void) __attribute__((weak));         // GPIO Port Q2
extern void GPIOPortQ3_Handler(void) __attribute__((weak));         // GPIO Port Q3
extern void GPIOPortQ4_Handler(void) __attribute__((weak));         // GPIO Port Q4
extern void GPIOPortQ5_Handler(void) __attribute__((weak));         // GPIO Port Q5
extern void GPIOPortQ6_Handler(void) __attribute__((weak));         // GPIO Port Q6
extern void GPIOPortQ7_Handler(void) __attribute__((weak));         // GPIO Port Q7
extern void GPIOPortR_Handler(void) __attribute__((weak));          // GPIO Port R
extern void GPIOPortS_Handler(void) __attribute__((weak));          // GPIO Port S
extern void PWM1Generator0_Handler(void) __attribute__((weak));     // PWM 1 Generator 0
extern void PWM1Generator1_Handler(void) __attribute__((weak));     // PWM 1 Generator 1
extern void PWM1Generator2_Handler(void) __attribute__((weak));     // PWM 1 Generator 2
extern void PWM1Generator3_Handler(void) __attribute__((weak));     // PWM 1 Generator 3
extern void PWM1Fault_Handler(void) __attribute__((weak));          // PWM 1 Fault
//*****************************************************************************
//
// The entry point for the application.
//
//*****************************************************************************
extern int main(void);

//*****************************************************************************
//
// Reserve space for the system stack.
//
//*****************************************************************************
static uint32_t pui32Stack[128];

//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000.
//
//*****************************************************************************
//
void (* __attribute__ ((section (".isr_vector"))) const  g_pfnVectors[])(void)  =
{
    (void (*)(void))((uint32_t)pui32Stack + sizeof(pui32Stack)),
                                            // The initial stack pointer
    ResetISR,                               // The reset handler
    NMI_Handler,                                  // The NMI handler
    HardFault_Handler,                               // The hard fault handler
    MemManage_Handler,                      // The MPU fault handler
    BusFault_Handler,                      // The bus fault handler
    UsageFault_Handler,                      // The usage fault handler
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    SVC_Handler,                      // SVCall handler
    DebugMon_Handler,                      // Debug monitor handler
    0,                                      // Reserved
    PendSV_Handler,                      // The PendSV handler
    SysTick_Handler,                      // The SysTick handler
    GPIOPortA_Handler,                      // GPIO Port A
    GPIOPortB_Handler,                      // GPIO Port B
    GPIOPortC_Handler,                      // GPIO Port C
    GPIOPortD_Handler,                      // GPIO Port D
    GPIOPortE_Handler,                      // GPIO Port E
    UART0_Handler,                      // UART0 Rx and Tx
    UART1_Handler,                      // UART1 Rx and Tx
    SSI0_Handler,                      // SSI0 Rx and Tx
    I2C0_Handler,                      // I2C0 Master and Slave
    PWM0Fault_Handler,                      // PWM Fault
    PWM0Generator0_Handler,                      // PWM Generator 0
    PWM0Generator1_Handler,                      // PWM Generator 1
    PWM0Generator2_Handler,                      // PWM Generator 2
    Quadrature0_Handler,                      // Quadrature Encoder 0
    ADC0Seq0_Handler,                      // ADC Sequence 0
    ADC0Seq1_Handler,                      // ADC Sequence 1
    ADC0Seq2_Handler,                      // ADC Sequence 2
    ADC0Seq3_Handler,                      // ADC Sequence 3
    WDT_Handler,                      // Watchdog timer
    Timer0A_Handler,                      // Timer 0 subtimer A
    Timer0B_Handler,                      // Timer 0 subtimer B
    Timer1A_Handler,                      // Timer 1 subtimer A
    Timer1B_Handler,                      // Timer 1 subtimer B
    Timer2A_Handler,                      // Timer 2 subtimer A
    Timer2B_Handler,                      // Timer 2 subtimer B
    Comp0_Handler,                      // Analog Comparator 0
    Comp1_Handler,                      // Analog Comparator 1
    Comp2_Handler,                      // Analog Comparator 2
    SysCtl_Handler,                      // System Control (PLL, OSC, BO)
    FlashCtl_Handler,                      // FLASH Control
    GPIOPortF_Handler,                      // GPIO Port F
    GPIOPortG_Handler,                      // GPIO Port G
    GPIOPortH_Handler,                      // GPIO Port H
    UART2_Handler,                      // UART2 Rx and Tx
    SSI1_Handler,                      // SSI1 Rx and Tx
    Timer3A_Handler,                      // Timer 3 subtimer A
    Timer3B_Handler,                      // Timer 3 subtimer B
    I2C1_Handler,                      // I2C1 Master and Slave
    Quadrature1_Handler,                      // Quadrature Encoder 1
    CAN0_Handler,                      // CAN0
    CAN1_Handler,                      // CAN1
    0,                                      // Reserved
    0,                                      // Reserved
    Hibernate_Handler,                      // Hibernate
    USB0_Handler,                      // USB0
    PWM0Generator3_Handler,                      // PWM Generator 3
    uDMA_Handler,                      // uDMA Software Transfer
    uDMA_Error,                      // uDMA Error
    ADC1Seq0_Handler,                      // ADC1 Sequence 0
    ADC1Seq1_Handler,                      // ADC1 Sequence 1
    ADC1Seq2_Handler,                      // ADC1 Sequence 2
    ADC1Seq3_Handler,                      // ADC1 Sequence 3
    0,                                      // Reserved
    0,                                      // Reserved
    GPIOPortJ_Handler,                      // GPIO Port J
    GPIOPortK_Handler,                      // GPIO Port K
    GPIOPortL_Handler,                      // GPIO Port L
    SSI2_Handler,                      // SSI2 Rx and Tx
    SSI3_Handler,                      // SSI3 Rx and Tx
    UART3_Handler,                      // UART3 Rx and Tx
    UART4_Handler,                      // UART4 Rx and Tx
    UART5_Handler,                      // UART5 Rx and Tx
    UART6_Handler,                      // UART6 Rx and Tx
    UART7_Handler,                      // UART7 Rx and Tx
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    I2C2_Handler,                      // I2C2 Master and Slave
    I2C3_Handler,                      // I2C3 Master and Slave
    Timer4A_Handler,                      // Timer 4 subtimer A
    Timer4B_Handler,                      // Timer 4 subtimer B
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    Timer5A_Handler,                      // Timer 5 subtimer A
    Timer5B_Handler,                      // Timer 5 subtimer B
    WideTimer0A_Handler,                      // Wide Timer 0 subtimer A
    WideTimer0B_Handler,                      // Wide Timer 0 subtimer B
    WideTimer1A_Handler,                      // Wide Timer 1 subtimer A
    WideTimer1B_Handler,                      // Wide Timer 1 subtimer B
    WideTimer2A_Handler,                      // Wide Timer 2 subtimer A
    WideTimer2B_Handler,                      // Wide Timer 2 subtimer B
    WideTimer3A_Handler,                      // Wide Timer 3 subtimer A
    WideTimer3B_Handler,                      // Wide Timer 3 subtimer B
    WideTimer4A_Handler,                      // Wide Timer 4 subtimer A
    WideTimer4B_Handler,                      // Wide Timer 4 subtimer B
    WideTimer5A_Handler,                      // Wide Timer 5 subtimer A
    WideTimer5B_Handler,                      // Wide Timer 5 subtimer B
    FPU_Handler,                      // FPU
    0,                                      // Reserved
    0,                                      // Reserved
    I2C4_Handler,                      // I2C4 Master and Slave
    I2C5_Handler,                      // I2C5 Master and Slave
    GPIOPortM_Handler,                      // GPIO Port M
    GPIOPortN_Handler,                      // GPIO Port N
    Quadrature2_Handler,                      // Quadrature Encoder 2
    0,                                      // Reserved
    0,                                      // Reserved
    GPIOPortP_Handler,                      // GPIO Port P (Summary or P0)
    GPIOPortP1_Handler,                      // GPIO Port P1
    GPIOPortP2_Handler,                      // GPIO Port P2
    GPIOPortP3_Handler,                      // GPIO Port P3
    GPIOPortP4_Handler,                      // GPIO Port P4
    GPIOPortP5_Handler,                      // GPIO Port P5
    GPIOPortP6_Handler,                      // GPIO Port P6
    GPIOPortP7_Handler,                      // GPIO Port P7
    GPIOPortQ_Handler,                      // GPIO Port Q (Summary or Q0)
    GPIOPortQ1_Handler,                      // GPIO Port Q1
    GPIOPortQ2_Handler,                      // GPIO Port Q2
    GPIOPortQ3_Handler,                      // GPIO Port Q3
    GPIOPortQ4_Handler,                      // GPIO Port Q4
    GPIOPortQ5_Handler,                      // GPIO Port Q5
    GPIOPortQ6_Handler,                      // GPIO Port Q6
    GPIOPortQ7_Handler,                      // GPIO Port Q7
    GPIOPortR_Handler,                      // GPIO Port R
    GPIOPortS_Handler,                      // GPIO Port S
    PWM1Generator0_Handler,                      // PWM 1 Generator 0
    PWM1Generator1_Handler,                      // PWM 1 Generator 1
    PWM1Generator2_Handler,                      // PWM 1 Generator 2
    PWM1Generator3_Handler,                      // PWM 1 Generator 3
    PWM1Fault_Handler                       // PWM 1 Fault
};

//*****************************************************************************
//
// The following are constructs created by the linker, indicating where the
// the "data" and "bss" segments reside in memory.  The initializers for the
// for the "data" segment resides immediately following the "text" segment.
//
//*****************************************************************************
extern uint32_t _ldata;
extern uint32_t _data;
extern uint32_t _edata;
extern uint32_t _bss;
extern uint32_t _ebss;

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied entry() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
void
ResetISR(void)
{
    uint32_t *pui32Src, *pui32Dest;

    //
    // Copy the data segment initializers from flash to SRAM.
    //
    pui32Src = &_ldata;
    for(pui32Dest = &_data; pui32Dest < &_edata; )
    {
        *pui32Dest++ = *pui32Src++;
    }

    //
    // Zero fill the bss segment.
    //
    __asm("    ldr     r0, =_bss\n"
          "    ldr     r1, =_ebss\n"
          "    mov     r2, #0\n"
          "    .thumb_func\n"
          "zero_loop:\n"
          "        cmp     r0, r1\n"
          "        it      lt\n"
          "        strlt   r2, [r0], #4\n"
          "        blt     zero_loop");

    //
    // Enable the floating-point unit.  This must be done here to handle the
    // case where main() uses floating-point and the function prologue saves
    // floating-point registers (which will fault if floating-point is not
    // enabled).  Any configuration of the floating-point unit using DriverLib
    // APIs must be done here prior to the floating-point unit being enabled.
    //
    // Note that this does not use DriverLib since it might not be included in
    // this project.
    //
    HWREG(NVIC_CPAC) = ((HWREG(NVIC_CPAC) &
                         ~(NVIC_CPAC_CP10_M | NVIC_CPAC_CP11_M)) |
                        NVIC_CPAC_CP10_FULL | NVIC_CPAC_CP11_FULL);

    //
    // Call the application's entry point.
    //
    main();
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a NMI.  This
// simply enters an infinite loop, preserving the system state for examination
// by a debugger.
//
//*****************************************************************************
 __attribute__((unused)) static void
NmiSR(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a fault
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
 __attribute__((unused)) static void
FaultISR(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************

__attribute__((unused)) static void
IntDefaultHandler(void) 
{
    //
    // Go into an infinite loop.
    //
    while(1)
    {
    }
}


static void
MemManage_Handler(void)
{

    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}
static void
BusFault_Handler(void)
{

    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}
static void
UsageFault_Handler(void)
{

    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

static void
NMI_Handler(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a fault
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
HardFault_Handler(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}



//*********** DisableInterrupts ***************
// disable interrupts
// inputs:  none
// outputs: none
void DisableInterrupts(void){
	__asm ("    CPSID  I\n");
}

//*********** EnableInterrupts ***************
// emable interrupts
// inputs:  none
// outputs: none
void EnableInterrupts(void){
	__asm  ("    CPSIE  I\n");
}
//*********** StartCritical ************************
// make a copy of previous I bit, disable interrupts
// inputs:  none
// outputs: previous I bit
void StartCritical(void){
 __asm  ("    MRS    R0, PRIMASK\n"
         "    CPSID  I         \n");
}

//*********** EndCritical ************************
// using the copy of previous I bit, restore I bit to previous value
// inputs:  previous I bit
// outputs: none
void EndCritical(void){
	__asm  ("    MSR    PRIMASK, R0\n");
}

//*********** WaitForInterrupt ************************
// go to low power mode while waiting for the next interrupt
// inputs:  none
// outputs: none
void WaitForInterrupt(void){
	__asm  ("    WFI\n");
}
