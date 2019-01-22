#include <stdint.h>
#include "tm4c123gh6pm.h"

void button_init(void){
    /* Port A Activation */
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;               // enable GPIO Port F clock
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0){};    // allow time for activating
    GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
    /* Port F Set Up */
    GPIO_PORTF_CR_R |= 0x11;                               // allow changes to PF0 and PF4
    GPIO_PORTF_DIR_R &= ~0x11;                             // make PF0 and PF4 intput
    GPIO_PORTF_AMSEL_R &= ~0x11;                           // disable analog on PF0 and PF4
    GPIO_PORTF_PCTL_R &= ((~GPIO_PCTL_PF0_M) &             // configure PF0 as GPIO
                          (~GPIO_PCTL_PF4_M));             // configure PF4 as GPIO
    GPIO_PORTF_PUR_R |= 0x11;                               // enable pull-up on PF0 and PF4
    GPIO_PORTF_AFSEL_R &= ~0x11;                          // disable alt functtion on PF0 and PF4
    GPIO_PORTF_DEN_R |= 0x11;                              // enable digital I/O on PF0 and PF4
}

uint32_t get_right_button_status (void) {
    return GPIO_PORTF_DATA_R & 0x01;
}

uint32_t get_left_button_status (void) {
    return GPIO_PORTF_DATA_R & 0x10;
}
