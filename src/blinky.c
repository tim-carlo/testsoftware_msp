#include <msp430.h>
#include <msp430fr5994.h>

#define RED_LED    BIT0
#define GREEN_LED  BIT1

void init_clock(void) {
    CSCTL0_H = CSKEY_H;
    CSCTL4 = LFXTOFF | HFXTOFF;
    CSCTL1 = DCOFSEL_3;  // ~8 MHz DCO
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    
    // Clear Fault-Flags
    __disable_interrupt();
    unsigned int timeout = 5000;
    do {
        CSCTL5 &= ~(LFXTOFFG | HFXTOFFG);
        SFRIFG1 &= ~OFIFG;
        if (--timeout == 0) break;
    } while (SFRIFG1 & OFIFG);
    __enable_interrupt();
    
    CSCTL0_H = 0;
}

void init_timer(void) {
    TA0CTL = TASSEL__SMCLK | MC__UP | TACLR;  // SMCLK, Up mode, clear timer
    TA0CCR0 = 32768;  // ~1s bei 32768 Hz (VLOCLK/1)
    TA0CCTL0 = CCIE;  // CCR0 Interrupt enable
}

// ISR mit Attribut statt Pragma
__attribute__((interrupt(TIMER0_A0_VECTOR)))
void Timer_A0_ISR(void) {
    P1OUT ^= RED_LED;
    __bic_SR_register_on_exit(LPM3_bits);
}


int main(void) {
    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;
    P1DIR |= RED_LED | GREEN_LED;
    P1OUT &= ~(RED_LED | GREEN_LED);

    init_clock();
    init_timer();
    __enable_interrupt();

    while (1) {
        __bis_SR_register(LPM3_bits | GIE);
    }
}