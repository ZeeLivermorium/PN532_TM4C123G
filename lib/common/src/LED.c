/*!
 * @file LED.c
 * @brief LaunchPad onboard LEDs for potential debugging use.
 * ----------
 * Adapted code from ValvanoWareTM4C123 by Dr. Jonathan Valvano.
 * You can find ValvanoWareTM4C123 at http://edx-org-utaustinx.s3.amazonaws.com/UT601x/ValvanoWareTM4C123.zip?dl=1
 * You can find more of his work at http://users.ece.utexas.edu/~valvano/
 * ----------
 * @author Zee Livermorium
 * @date Apr 12, 2018
 */
#include <stdint.h>
#include "tm4c123gh6pm.h"            // put this in the right path accordingly

void LED_Init(void){
    /* Port A Activation */
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;               // enable GPIO Port F clock
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0){};    // allow time for activating
    
    /* Port F Set Up */
    GPIO_PORTF_CR_R = 0x0E;                                // allow changes to PF1-3
    GPIO_PORTF_DIR_R = 0x0E;                               // make PF1-3 output
    GPIO_PORTF_AMSEL_R &= ~0x0E;                           // disable analog on PF1-3
    GPIO_PORTF_PCTL_R &= ((~GPIO_PCTL_PF1_M) &             // configure PF1 as GPIO
                          (~GPIO_PCTL_PF2_M) &             // configure PF2 as GPIO
                          (~GPIO_PCTL_PF3_M));             // configure PF3 as GPIO
    GPIO_PORTF_AFSEL_R  &= ~0x0E;                          // disable alt functtion on PF1-3
    GPIO_PORTF_DEN_R = 0x0E;                               // enable digital I/O on PF1-3
}

void LED_RED_ON(void) { GPIO_PORTF_DATA_R |= 0x02; }

void LED_RED_OFF(void) { GPIO_PORTF_DATA_R &= ~0x02; }

void LED_BLUE_ON(void) { GPIO_PORTF_DATA_R |= 0x04; }

void LED_BLUE_OFF(void) { GPIO_PORTF_DATA_R &= ~0x04; }

void LED_GREEN_ON(void) { GPIO_PORTF_DATA_R |= 0x08; }

void LED_GREEN_OFF(void) { GPIO_PORTF_DATA_R &= ~0x08; }


