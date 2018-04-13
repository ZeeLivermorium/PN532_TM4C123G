/*
 * File: LED.c
 * LED for potential debugging use.
 * ----------
 * Adapted code from ValvanoWareTM4C123 by Dr. Jonathan Valvano
 * You can find ValvanoWareTM4C123 at http://edx-org-utaustinx.s3.amazonaws.com/UT601x/ValvanoWareTM4C123.zip?dl=1
 * You can find more of his work at http://users.ece.utexas.edu/~valvano/
 * ----------
 * Zee Livermorium
 * Apr 12, 2018
 */
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"      // put this in the right path accordingly

void LED_Init(void){
    volatile uint32_t delay;
    SYSCTL_RCGCGPIO_R |= 0x00000020;  // 1) activate clock for Port F
    delay = SYSCTL_RCGCGPIO_R;        // allow time for clock to start
    GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
    GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
    // only PF0 needs to be unlocked, other bits can't be locked
    GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog on PF
    GPIO_PORTF_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
    GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 in, PF3-1 out
    GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alt funct on PF7-0
    GPIO_PORTF_PUR_R = 0x11;          // enable pull-up on PF0 and PF4
    GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital I/O on PF4-0
}

void LED_RED_ON(void) { GPIO_PORTF_DATA_R |= 0x02; }

void LED_RED_OFF(void) { GPIO_PORTF_DATA_R &= ~0x02; }

void LED_BLUE_ON(void) { GPIO_PORTF_DATA_R |= 0x04; }

void LED_BLUE_OFF(void) { GPIO_PORTF_DATA_R &= ~0x02; }

void LED_GREEN_ON(void) { GPIO_PORTF_DATA_R |= 0x08; }

void LED_GREEN_OFF(void) { GPIO_PORTF_DATA_R &= ~0x02; }


