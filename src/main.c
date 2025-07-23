#include <msp430.h>

volatile unsigned int timer_ticks = 0;

void clock_init(void)
{
    CSCTL0_H = CSKEY >> 8;
    CSCTL1 = DCOFSEL_0;
    CSCTL2 = SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVS__1 | DIVM__1;
    CSCTL0_H = 0;
}

void gpio_init(void)
{
    P1DIR |= BIT0 | BIT1;
    P1OUT &= ~(BIT0 | BIT1);
}

void timer_init(void)
{
    TA0CCR0 = 62500 - 1;
    TA0CCTL0 &= ~CCIFG;  // Clear flag
    TA0CCTL0 = CCIE;     // Enable interrupt
    TA0CTL = TASSEL__SMCLK | ID__8 | MC__UP | TACLR;
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;
    clock_init();
    gpio_init();
    PM5CTL0 &= ~LOCKLPM5;
    timer_init();

    __bis_SR_register(GIE); // Enable global interrupts

    while (1) {}
}

__attribute__((interrupt(TIMER0_A0_VECTOR)))
void TIMER0_A0_ISR(void)
{
    timer_ticks++;
    P1OUT ^= BIT0;
}