#include "msp430fr5994_helper.h"
bool delay_done = false;
uint32_t timer_overflows = 0;
uint64_t gpio_blacklist_intern_mask = 0;

/**
 * @brief Get the port of absolute pin object
 *
 * @param abs_pin
 * @return uint32_t
 */
uint16_t get_port_base_of_absolute_pin(uint32_t abs_pin)
{
    if (abs_pin >= 0 && abs_pin < 8)
    { // P1.0 to P1.7
        return P1_BASE;
    }
    else if (abs_pin >= 8 && abs_pin < 16)
    { // P2.0 to P2.7
        return P2_BASE;
    }
    else if (abs_pin >= 16 && abs_pin < 24)
    { // P3.0 to P3.7
        return P3_BASE;
    }
    else if (abs_pin >= 24 && abs_pin < 32)
    { // P4.0 to P4.7
        return P4_BASE;
    }
    else if (abs_pin >= 32 && abs_pin < 40)
    { // P5.0 to P5.7
        return P5_BASE;
    }
    else if (abs_pin >= 40 && abs_pin < 48)
    { // P6.0 to P6.7
        return P6_BASE;
    }
    else if (abs_pin >= 48 && abs_pin < 56)
    { // P7.0 to P7.7
        return P7_BASE;
    }
    else if (abs_pin >= 56 && abs_pin < 64)
    { // P8.0 to P8.7
        return P8_BASE;
    }
    else if (abs_pin >= 64 && abs_pin < 72)
    { // P9.0 to P9.7
        return P9_BASE;
    }
    return 0; // Invalid pin
}

/**
 * @brief Get the relative pin object
 *
 * @param abs_pin
 * @return uint8_t
 */
uint8_t get_relative_pin(uint8_t abs_pin)
{
    return abs_pin % 8; // Return the pin number within the port
}

/**
 * @brief Initialize the I/O subsystem
 *
 * This function initializes the GPIO, UART, and Timer modules.
 */
void io_init()
{
    // Disable watchdog
    WDTCTL = WDTPW | WDTHOLD;

    // Unlock GPIO
    PM5CTL0 &= ~LOCKLPM5;

    // Configure clock to 16MHz
    FRCTL0 = FRCTLPW | NWAITS_1;          // FRAM wait states
    CSCTL0_H = CSKEY >> 8;                // Unlock CS registers
    CSCTL1 = DCOFSEL_4 | DCORSEL;         // DCO = 16MHz
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1; // All dividers = 1
    CSCTL0_H = 0;                         // Lock CS registers

    // Configure UART
    UCA0CTLW0 = UCSWRST;         // Reset UART
    UCA0CTLW0 |= UCSSEL__SMCLK;  // SMCLK source (16MHz)
    UCA0BR0 = 104;               // 16MHz/9600 = 1666.67
    UCA0BR1 = 0;                 // High byte
    UCA0MCTLW = UCOS16 | 0x4900; // Oversampling + fractional tuning
    UCA0CTLW0 &= ~UCSWRST;       // Enable UART

    // Configure UART pins
    P2SEL0 &= ~(BIT0 | BIT1); // Clear P2.0/P2.1 SEL0
    P2SEL1 |= BIT0 | BIT1;    // Set UART function

    // Configure Timer A1
    TA1CTL = TASSEL__SMCLK | ID__8 | MC__STOP | TACLR | TAIE;
    timer_overflows = 0;

    __enable_interrupt(); // Enable global interrupts
}

/**
 * @brief Initialize GPIO pin with pull-up resistor
 *
 * @param abs_pin Absolute pin number
 */
void gpio_pullup_init(uint8_t abs_pin)
{
    uint16_t base = get_port_base_of_absolute_pin(abs_pin);
    uint8_t mask = 1 << get_relative_pin(abs_pin);
    // Enable pull-up resistor (REN = 1, OUT = 1)
    *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) |= mask;
    *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) |= mask;
}

/**
 * @brief Initialize GPIO pin with pull-down resistor
 *
 * @param abs_pin Absolute pin number
 */
void gpio_pulldown_init(uint8_t abs_pin)
{
    uint16_t base = get_port_base_of_absolute_pin(abs_pin);
    uint8_t mask = 1 << get_relative_pin(abs_pin);
    // Enable pull-down resistor (REN = 1, OUT = 0)
    *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) |= mask;
    *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) &= ~mask;
}

/**
 * @brief Pull-up resistor clear
 * @param abs_pin Absolute pin number
 */
void gpio_pullup_clear(uint8_t abs_pin)
{
    uint16_t base = get_port_base_of_absolute_pin(abs_pin);
    uint8_t mask = 1 << get_relative_pin(abs_pin);
    // Disable pull-up resistor (REN = 0)
    *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) &= ~mask;
}
/**
 * @brief Pull-down resistor clear
 * @param abs_pin Absolute pin number
 *
 */
void gpio_pulldown_clear(uint8_t abs_pin)
{
    uint16_t base = get_port_base_of_absolute_pin(abs_pin);
    uint8_t mask = 1 << get_relative_pin(abs_pin);
    // Disable pull-down resistor (REN = 0)
    *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) &= ~mask;
}
/**
 * @brief Drive GPIO pin low
 *
 * @param abs_pin Absolute pin number
 */
void gpio_drive_low(uint8_t abs_pin)
{
    uint16_t base = get_port_base_of_absolute_pin(abs_pin);
    uint8_t mask = 1 << get_relative_pin(abs_pin);
    // Set pin low (OUT = 0)
    *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) &= ~mask;
}
/**
 * @brief Drive GPIO pin high
 * @param abs_pin Absolute pin number
 */
void gpio_drive_high(uint8_t abs_pin)
{
    uint16_t base = get_port_base_of_absolute_pin(abs_pin);
    uint8_t mask = 1 << get_relative_pin(abs_pin);
    // Set pin high (OUT = 1)
    *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) |= mask;
}
/**
 * @brief Initialize GPIO pin as input
 * @param abs_pin Absolute pin number
 */
void gpio_input_init(uint8_t abs_pin)
{
    uint16_t base = get_port_base_of_absolute_pin(abs_pin);
    uint8_t mask = 1 << get_relative_pin(abs_pin);
    // Set pin as input (DIR = 0)
    *(volatile uint8_t *)((uintptr_t)(base + PORT_DIR_OFFSET)) &= ~mask;
}
/**
 * @brief Initialize GPIO pin as output
 * @param abs_pin Absolute pin number
 */
void gpio_output_init(uint8_t abs_pin)
{
    uint16_t base = get_port_base_of_absolute_pin(abs_pin);
    uint8_t mask = 1 << get_relative_pin(abs_pin);
    // Set pin as output (DIR = 1)
    *(volatile uint8_t *)((uintptr_t)(base + PORT_DIR_OFFSET)) |= mask;
}
/**
 * @brief Read GPIO pin state
 *
 * @param abs_pin Absolute pin number
 * @return true if pin is high, false if low
 */
bool gpio_read(uint8_t abs_pin)
{
    uint16_t base = get_port_base_of_absolute_pin(abs_pin);
    uint8_t mask = 1 << get_relative_pin(abs_pin);
    // Read pin state (IN)
    return (*(volatile uint8_t *)((uintptr_t)(base + PORT_IN_OFFSET)) & mask) != 0;
}

/**
 * @brief Execute one 62.5ms timer cycle
 */
void delay_62_5ms(void)
{
    TA0CTL |= MC__UP | TACLR; // Start timer in up mode
    while (!(TA0CCTL0 & CCIFG))
        ;               // Wait for CCR0
    TA0CTL &= ~MC_3;    // Stop timer
    TA0CCTL0 &= ~CCIFG; // Clear flag
}

/**
 * @brief Delay for a specified number of microseconds
 *
 * @param us Number of microseconds to delay
 */
void delay_us(uint32_t us)
{
    if (us == 0)
        return;

    // Convert delay time to timer ticks
    uint32_t ticks = us * TICKS_PER_US;

    while (ticks > 0)
    {
        // Chunk max size (16-bit timer)
        uint16_t chunk = (ticks > 65535) ? 65535 : ticks;

        delay_done = false;

        // Stop and reset Timer A0 before configuration
        TA0CTL = TASSEL__SMCLK | ID__1 | MC__STOP | TACLR;

        TA0CCR0 = chunk - 1; // Set compare value
        TA0CCTL0 = CCIE;     // Enable interrupt

        TA0CTL |= MC__UP; // Start timer in up mode

        __bis_SR_register(LPM0_bits | GIE); // Enter low-power mode with interrupts

        // After waking up
        TA0CTL = MC__STOP; // Stop the timer
        TA0CCTL0 &= ~CCIE; // Disable interrupt

        ticks -= chunk; // Decrement remaining ticks
    }
}

/**
 * @brief Interrupt Service Routine for Timer A0
 */
void __attribute__((interrupt(TIMER0_A0_VECTOR))) Timer0_A0_ISR(void)
{
    TA0CCTL0 &= ~CCIFG; // Clear interrupt flag
    delay_done = true;
    __bic_SR_register_on_exit(LPM0_bits); // Wake up from LPM
}

/**
 * @brief Busy-wait delay in milliseconds using Timer A0
 * @param ms Milliseconds to delay
 */
void delay_ms(uint32_t ms)
{
    while (ms--)
        delay_us(1000);
}


/**
 * @brief Push all active GPIO pins except the specified one to a stack using absolute pin numbers
 *
 * @param stack Pointer to the stack where active pins will be pushed
 * @param expected_level Expected level of the pins (true for high, false for low)
 * @param blacklist_mask Bitmask of pins to exclude (1 for excluded, 0 for included)
 */
void push_active_pins_except_blacklist_to_stack(Stack *stack, bool expected_level, uint64_t blacklist_mask)
{
    for (uint8_t abs_pin = 0; abs_pin < NUMBER_OF_GPIO_PINS; abs_pin++)
    {
        if ((blacklist_mask >> abs_pin) & 1)
            continue;
        if (gpio_read(abs_pin) == expected_level)
        {
            printf("Pushing active pin %lu to stack\n", abs_pin);
            push(stack, &abs_pin);
        }
    }
}

bool is_interupt_blacklisted(uint8_t abs_pin)
{
    return (gpio_blacklist_intern_mask >> abs_pin) & 1;
}

void gpio_listen_interrupt_on_all_pins(uint64_t blacklist,
                                       gpio_interrupt_handler_t falling_handler,
                                       gpio_interrupt_handler_t rising_handler)
{
    gpio_blacklist_intern_mask = blacklist;

}