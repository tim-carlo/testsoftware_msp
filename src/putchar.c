
#include <msp430.h>
#include <msp430fr5994.h>
#include "putchar.h"


#define TXLED   BIT0
#define TXD     BIT0  // P2.0 (UCA0TXD)
#define RXD     BIT1  // P2.1 (UCA0RXD)


void configureClock16MHZ() {
    FRCTL0 = FRCTLPW | NWAITS_1;  // Set FRAM wait state to 1
    CSCTL0_H = CSKEY_H;           // Unlock clock system registers
    CSCTL1 = DCOFSEL_4 | DCORSEL; // Configure DCO for 16 MHz
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1; // Set ACLK, SMCLK, MCLK dividers to 1
    CSCTL0_H = 0;                 // Lock clock system registers
}

void configureUART() {
    UCA0CTLW0 = UCSWRST;          // Enable UART reset
    UCA0CTLW0 |= UCSSEL__SMCLK;   // Select SMCLK = 16 MHz
    UCA0BR0 = 104;                // Set baud rate divider low byte to 104
    UCA0BR1 = 0;                  // Set baud rate divider high byte to 0 → 9600 baud
    UCA0MCTLW = UCOS16 | 0x4900;  // Enable oversampling, set modulation stage to 0x49
    UCA0CTLW0 &= ~UCSWRST;        // Release UART from reset (enable UART)

    PM5CTL0 &= ~LOCKLPM5;         // Disable GPIO high-impedance mode
    P2SEL0 &= ~(TXD | RXD);       // Clear P2 SEL0 bits for UART pins
    P2SEL1 |= (TXD | RXD);        // Set P2 SEL1 bits for UART function
    configureClock16MHZ();      // Configure clock to 16 MHz
}

void _putchar(char character) {
    while (!(UCA0IFG & UCTXIFG)); // Wait until TX buffer is empty
    UCA0TXBUF = character;       // Send character
}