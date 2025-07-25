#include "msp430fr5994_helper.h"

bool delay_done = false;
uint32_t overflow_count = 0;
uint64_t gpio_blacklist_intern_mask = 0;
gpio_interrupt_handler_t global_falling_handler = NULL;
gpio_interrupt_handler_t global_rising_handler = NULL;
uint64_t previous_state = 0;

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
    // Ensure pin is driven low (OUT = 0)
    *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) &= ~mask;
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
    // Ensure pin is driven high (OUT = 1)
    *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) |= mask;
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
 * @brief Delay for a specified number of microseconds
 *
 * @param us Number of microseconds to delay
 */
void delay_us(uint32_t us)
{
    uint32_t ticks = us * (SMCLK_HZ / 1000000);
    ; // 1 tick = 1 µs @ 1 MHz SMCLK
    uint32_t full_overflows = ticks / 65536;
    uint16_t remaining_ticks = ticks % 65536;

    overflow_count = full_overflows;
    delay_done = false;

    TA0CTL = TASSEL__SMCLK | ID__1 | MC__STOP | TACLR; // SMCLK, no divider, stop mode, clear
    TA0R = 0;

    // Initial CCR0 setup
    if (remaining_ticks == 0 && full_overflows == 0)
        TA0CCR0 = 1; // minimal delay
    else
        TA0CCR0 = remaining_ticks;

    TA0CCTL0 |= CCIE; // Enable CCR0 interrupt

    if (overflow_count > 0)
    {
        TA0CCR0 = 0xFFFF; // run full overflows first
        TA0CTL |= TAIE;   // Enable overflow interrupt
    }

    TA0CTL |= MC__UP; // Start timer in up mode

    // Wait until delay completes
    while (!delay_done)
        ;

    // Cleanup
    TA0CCTL0 &= ~CCIE;
    TA0CTL &= ~(MC__UP | TAIE);
    TA0CTL |= TACLR;
}

/**
 * @brief Interrupt Service Routine for Timer A0
 */
void __attribute__((interrupt(TIMER0_A0_VECTOR))) Timer0_A0_ISR(void)
{
    TA0CCTL0 &= ~CCIFG; // Clear CCR0 interrupt flag
    delay_done = true;
}

void __attribute__((interrupt(TIMER0_A1_VECTOR))) Timer0_A1_ISR(void)
{
    switch (__even_in_range(TA0IV, TA0IV_TAIFG))
    {
    case TA0IV_TAIFG: // Overflow
        if (overflow_count > 0)
            overflow_count--;

        if (overflow_count == 0)
        {
            TA0CTL &= ~TAIE;    // Disable further overflow interrupts
            TA0CCR0 = TA0R + 1; // Trigger CCR0 one tick later
        }
        break;

    default:
        break;
    }
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

uint64_t read_all_gpio_states()
{
    uint64_t state = 0;
    state |= (uint64_t)(*(volatile uint8_t *)&P1IN) << 0;
    state |= (uint64_t)(*(volatile uint8_t *)&P2IN) << 8;
    state |= (uint64_t)(*(volatile uint8_t *)&P3IN) << 16;
    state |= (uint64_t)(*(volatile uint8_t *)&P4IN) << 24;
    state |= (uint64_t)(*(volatile uint8_t *)&P5IN) << 32;
    state |= (uint64_t)(*(volatile uint8_t *)&P6IN) << 40;
    state |= (uint64_t)(*(volatile uint8_t *)&P7IN) << 48;
    state |= (uint64_t)(*(volatile uint8_t *)&P8IN) << 56;
    return state;
}

void gpio_listen_on_all_pins_polling(uint64_t blacklist_mask,
                                     gpio_interrupt_handler_t falling_handler,
                                     gpio_interrupt_handler_t rising_handler)
{
    gpio_blacklist_intern_mask = blacklist_mask;
    global_falling_handler = falling_handler;
    global_rising_handler = rising_handler;

    // Configure all pins in open-drain mode
    for (uint8_t abs_pin = 0; abs_pin < NUMBER_OF_GPIO_PINS; abs_pin++)
    {
        if (!((blacklist_mask >> abs_pin) & 1))
        {
            gpio_pullup_init(abs_pin);
            gpio_input_init(abs_pin);
        }
    }

    previous_state = read_all_gpio_states();
    TA4CCR0 = 40000 - 1;                             // Zählt bis 39999 → alle 20 ms Interrupt
    TA4CTL = TASSEL__SMCLK | ID__8 | MC__UP | TACLR; // SMCLK, ÷8, Up-Mode
    TA4CCTL0 = CCIE;                                 // Interrupt für CCR0 aktivieren | MC__UP | ID__8 | TACLR;
}
static void gpio_timer_poll()
{
    uint64_t curr_state = read_all_gpio_states();
    uint64_t changed = (previous_state ^ curr_state) & ~gpio_blacklist_intern_mask;

    for (uint32_t i = 0; i < NUMBER_OF_GPIO_PINS; i++)
    {
        if (changed & ((uint64_t)1 << i))
        {
            bool prev = (previous_state >> i) & 1;
            bool curr = (curr_state >> i) & 1;

            if (prev == 1 && curr == 0 && global_falling_handler)
            {
                printf("Falling edge detected on P%lu.%lu\n", i >> 3, i & 0x07);
                global_falling_handler(i);
            }
            else if (prev == 0 && curr == 1 && global_rising_handler)
            {
                printf("Rising edge detected on P%lu.%lu\n", i >> 3, i & 0x07);
                global_rising_handler(i);
            }
        }
    }

    previous_state = curr_state;
}

static volatile uint8_t *const PxIN[] = {&P1IN, &P2IN, &P3IN, &P4IN, &P5IN, &P6IN, &P7IN, &P8IN};
static volatile uint8_t *const PxIE[] = {&P1IE, &P2IE, &P3IE, &P4IE, &P5IE, &P6IE, &P7IE, &P8IE};
static volatile uint8_t *const PxIFG[] = {&P1IFG, &P2IFG, &P3IFG, &P4IFG, &P5IFG, &P6IFG, &P7IFG, &P8IFG};
static volatile uint8_t *const PxIES[] = {&P1IES, &P2IES, &P3IES, &P4IES, &P5IES, &P6IES, &P7IES, &P8IES};


/**
 * @brief Listen on all GPIO pins for rising and falling edges
 *
 * @param blacklist_mask Bitmask of pins to ignore (1 for ignored, 0 for active)
 * @param falling_handler Handler for falling edges
 * @param rising_handler Handler for rising edges
 */
void gpio_listen_on_all_pins_interrupt(uint64_t blacklist_mask,
                                       gpio_interrupt_handler_t falling_handler,
                                       gpio_interrupt_handler_t rising_handler)
{
    gpio_blacklist_intern_mask = blacklist_mask;
    global_falling_handler = falling_handler;
    global_rising_handler = rising_handler;

    for (uint8_t abs_pin = 0; abs_pin < NUMBER_OF_GPIO_PINS; abs_pin++)
    {
        if ((blacklist_mask >> abs_pin) & 1)
            continue; // Pin ignorieren

        gpio_pullup_init(abs_pin); // Pull-up aktivieren
        gpio_input_init(abs_pin);  // Pin als Eingang konfigurieren

        uint8_t port = abs_pin >> 3;
        uint8_t pin = abs_pin & 0x07;

        *PxIFG[port] &= ~(1 << pin); // Clear Interrupt Flag
        *PxIES[port] |= (1 << pin); // Falling Edge (default)
        *PxIE[port] |= (1 << pin);   // Enable Interrupt
    }

    __enable_interrupt(); // Global Interrupt Enable
}

// Macro to define an Interrupt Service Routine (ISR) for a given GPIO port
// Handles both falling and rising edges by toggling the edge detection after each interrupt
#define DEFINE_PORT_ISR(port)                                                    \
    void __attribute__((interrupt(PORT##port##_VECTOR))) PORT##port##_ISR(void) \
    {                                                                            \
        /* Get only active and enabled interrupt flags for this port */         \
        uint8_t flags = P##port##IFG & P##port##IE;                              \
                                                                                 \
        for (uint8_t pin = 0; pin < 8; pin++)                                    \
        {                                                                        \
            if (flags & (1 << pin))                                              \
            {                                                                    \
                /* Compute absolute pin index, e.g., P3.4 → 20 */               \
                uint8_t abs_pin = ((port - 1) << 3) | pin;                       \
                                                                                 \
                /* Skip if this pin is blacklisted */                            \
                if ((gpio_blacklist_intern_mask >> abs_pin) & 1)                 \
                    continue;                                                    \
                /* Check if the current edge setting is falling */              \
                bool is_falling = (P##port##IES >> pin) & 1;                     \
                                                                                 \
                if (is_falling)                                                  \
                {                                                                \
                    /* Falling edge detected → call falling edge handler */     \
                    if (global_falling_handler)                                 \
                        global_falling_handler(abs_pin);                         \
                                                                                 \
                    /* Switch to rising edge detection for next time */         \
                    P##port##IES &= ~(1 << pin);                                 \
                }                                                                \
                else                                                             \
                {                                                                \
                    /* Rising edge detected → call rising edge handler */       \
                    if (global_rising_handler)                                  \
                        global_rising_handler(abs_pin);                          \
                                                                                 \
                    /* Switch to falling edge detection for next time */        \
                    P##port##IES |= (1 << pin);                                  \
                }                                                                \
                                                                                 \
                /* Clear the interrupt flag (ACK) */                             \
                P##port##IFG &= ~(1 << pin);                                     \
            }                                                                    \
        }                                                                        \
    }         

// Port ISR Definitions
DEFINE_PORT_ISR(1)
DEFINE_PORT_ISR(2)
DEFINE_PORT_ISR(3)
DEFINE_PORT_ISR(4)
DEFINE_PORT_ISR(5)
DEFINE_PORT_ISR(6)
DEFINE_PORT_ISR(7)
DEFINE_PORT_ISR(8)