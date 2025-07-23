#ifndef MSP430FR5994_HELPER_H
#define MSP430FR5994_HELPER_H

#include <msp430.h>
#include <msp430fr5994.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h> 

#include "printf.h"  // Custom printf implementation
#include "putchar.h" // Custom putchar implementation
#include "stack.h"   // Stack implementation for managing GPIO states
#include "pindata.h" // PinData structure and helper functions

// Register offsets from port base
#define PORT_IN_OFFSET 0x00
#define PORT_OUT_OFFSET 0x02
#define PORT_DIR_OFFSET 0x04
#define PORT_REN_OFFSET 0x06
#define PORT_SEL0_OFFSET 0x0A

// UART configuration - Fixed to UCA0 at 9600 baud
#define UART_ID 0 // Identifier for UCA0
#define BAUD_RATE 9600
#define UART_TX_PIN 8 // P2.0 (gpio 8)
#define UART_RX_PIN 9 // P2.1 (gpio 9)

// Base address of the device descriptor table
#define DEVICE_DESCRIPTOR_ADDR 0x1A00

// Timer configuration
#define TIMER_A0_CCR0_VALUE 15624 // For 62.5 ms at SMCLK/64 (16MHz/64 = 250kHz)
#define TICKS_PER_US 16           // 16 ticks per µs at 16MHz SMCLK
#define TIMER_A1_CLK_HZ 500000

#define NUMBER_OF_GPIO_PINS 68


typedef void (*gpio_interrupt_handler_t)(uint32_t gpio);


#define NUM

#ifdef __cplusplus
extern "C"
{
#endif

    void io_init(void);
    
    void gpio_pullup_init(uint8_t abs_pin);
    void gpio_pulldown_init(uint8_t abs_pin);
    void gpio_pullup_clear(uint8_t abs_pin);
    void gpio_drive_low(uint8_t abs_pin);
    void gpio_drive_high(uint8_t abs_pin);
    void gpio_input_init(uint8_t abs_pin);
    void gpio_output_init(uint8_t abs_pin);
    bool gpio_read(uint8_t abs_pin);
    void push_active_pins_to_stack(Stack *stack, uint8_t level);
    void push_active_pins_except_blacklist_to_stack(Stack *stack, bool expected_level, uint64_t blacklist_mask);

    void gpio_reset(uint8_t abs_pin);
    void gpio_open_drain(uint8_t abs_pin);
    void release_gpio_open_drain(uint8_t abs_pin);

    bool is_interupt_blacklisted(uint8_t abs_pin);
    void configure_pin_sense(uint8_t abs_pin, bool sense_low);
    void gpio_listen_interrupt_on_all_pins(uint64_t blacklist_mask, gpio_interrupt_handler_t rising_handler, gpio_interrupt_handler_t falling_handler);

    uint32_t get_elapsed_time(uint32_t start, uint32_t current);
    void delay_us(uint32_t us);
    void delay_ms(uint32_t ms);

    bool is_signal_active(uint8_t pin, bool assert_high);
    uint32_t get_timer_ticks(void);
    void start_timer(void);
    void stop_timer(void);
    void reset_timer(void);

    uint32_t ticks_to_us(uint64_t ticks);
    uint32_t ticks_to_ms(uint64_t ticks);
    uint32_t timer_diff_us(uint64_t start, uint64_t end);
    uint32_t timer_diff_ms(uint64_t start, uint64_t end);

    uint32_t random32_lfsr(void);
    uint32_t random32(void);
    uint32_t select_random_non_blacklisted_and_not_successful_pin(PinData *pindata, uint64_t blacklist_mask);

    void init_software_serial(uint32_t abs_pin, uint32_t baudrate);
    void software_serial_tx(uint8_t byte);

    uint64_t get_unique_id(void);
    const char *get_unique_id_str(void);
    const char *get_chip_family_name(void);

#ifdef __cplusplus
}
#endif

#endif // MSP430FR5994_HELPER_H