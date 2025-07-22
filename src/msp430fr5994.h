#ifndef MSP430FR5994_HELPER_H
#define MSP430FR5994_HELPER_H

#include <msp430.h>
#include <msp430fr5994.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>  // For uintptr_t
#include "printf.h"  // Custom printf implementation
#include "putchar.h" // Custom putchar implementation

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

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Get the base address for a GPIO port
     * @param port Port number (1-10)
     * @return Base address of the port registers
     */
    static inline uint16_t get_port_base(uint8_t port)
    {
        switch (port)
        {
        case 1:
            return P1_BASE;
        case 2:
            return P2_BASE;
        case 3:
            return P3_BASE;
        case 4:
            return P4_BASE;
        case 5:
            return P5_BASE;
        case 6:
            return P6_BASE;
        case 7:
            return P7_BASE;
        case 8:
            return P8_BASE;
        case 9:
            return P9_BASE;
        default:
            return 0; // Invalid port
        }
    }

    /**
     * @brief Initialize UART and clock system
     */
    static inline void io_init(void)
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

        // Initialize Timer A0 for precise timing
        TA0CTL = TASSEL__SMCLK | ID__8 | MC__STOP | TACLR; // SMCLK/8, stopped
        TA0EX0 = TAIDEX_7;                                 // Additional /8 divider (total /64)
        TA0CCR0 = TIMER_A0_CCR0_VALUE;                     // 15624 ticks = 62.5ms

        // Timer A1 Konfiguration
        TA1CTL = TASSEL__SMCLK | ID__1 | MC__STOP | TACLR;
        TA1EX0 = 0x000F; // Gesamtteiler /32 (ID__1 = /2 + EX0=15 = /16)
        TA1CCR0 = 0xFFFF;
        TA1CCTL0 = CCIE; // Interrupt aktivieren

        __enable_interrupt(); // Enable global interrupts
    }

    /**
     * @brief Set Pull-up resistor on GPIO pin
     * @param port Port number (1-10)
     * @param pin Pin number (0-7)

    */
    static inline void gpio_pullup_init(uint8_t port, uint8_t pin)
    {
        uint16_t base = get_port_base(port);
        uint8_t mask = 1 << pin;
        // Enable pull-up resistor (REN = 1, OUT = 1)
        *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) |= mask;
        *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) |= mask;
    }

    /**
     * @brief Set Pull-down resistor on GPIO pin
     * @param port Port number (1-10)
     * @param pin Pin number (0-7)
     */
    static inline void gpio_pulldown_init(uint8_t port, uint8_t pin)
    {
        uint16_t base = get_port_base(port);
        uint8_t mask = 1 << pin;
        // Enable pull-down resistor (REN = 1, OUT = 0)
        *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) |= mask;
        *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) &= ~mask;
    }

    /**
     * @brief Clear Pull-down resistors on GPIO pin
     * @param port Port number (1-10)
     * @param pin Pin number (0-7)
     */
    static inline void gpio_pulldown_clear(uint8_t port, uint8_t pin)
    {
        uint16_t base = get_port_base(port);
        uint8_t mask = 1 << pin;
        // Disable pull-down resistor (REN = 0)
        *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) &= ~mask;
        *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) &= ~mask; // Clear output
    }
    /**
     * @brief Clear Pull-up resistors on GPIO pin
     * @param port Port number (1-10)
     * @param pin Pin number (0-7)
     */
    static inline void gpio_pullup_clear(uint8_t port, uint8_t pin)
    {
        uint16_t base = get_port_base(port);
        uint8_t mask = 1 << pin;
        // Disable pull-up resistor (REN = 0)
        *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) &= ~mask;
        *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) &= ~mask; // Clear output
    }

    /**
     * @brief Drive GPIO pin low (output mode)
     * @param port Port number (1-10)
     * @param pin Pin number (0-7)
     */
    static inline void gpio_drive_low(uint8_t port, uint8_t pin)
    {
        uint16_t base = get_port_base(port);
        uint8_t mask = 1 << pin;

        // Set as output (DIR = 1)
        *(volatile uint8_t *)((uintptr_t)(base + PORT_DIR_OFFSET)) |= mask;

        // Drive low (OUT = 0)
        *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) &= ~mask;
    }

    /**
     * @brief Drive GPIO pin high (output mode)
     * @param port Port number (1-10)
     * @param pin Pin number (0-7)
     */
    static inline void gpio_drive_high(uint8_t port, uint8_t pin)
    {
        uint16_t base = get_port_base(port);
        uint8_t mask = 1 << pin;

        // Set as output (DIR = 1)
        *(volatile uint8_t *)((uintptr_t)(base + PORT_DIR_OFFSET)) |= mask;

        // Drive high (OUT = 1)
        *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) |= mask;
    }

    /**
     * @brief Set GPIO pin as input (no pull-up/pull-down)
     * @param port Port number (1-10)
     * @param pin Pin number (0-7)
     */
    static inline void gpio_input_init(uint8_t port, uint8_t pin)
    {
        uint16_t base = get_port_base(port);
        uint8_t mask = 1 << pin;

        // Set as input (DIR = 0)
        *(volatile uint8_t *)((uintptr_t)(base + PORT_DIR_OFFSET)) &= ~mask;
    }

    /**
     * @brief Set GPIO pin as output
     * @param port Port number (1-10)
     * @param pin Pin number (0-7)
     */
    static inline void gpio_output_init(uint8_t port, uint8_t pin)
    {
        uint16_t base = get_port_base(port);
        uint8_t mask = 1 << pin;

        // Set as output (DIR = 1)
        *(volatile uint8_t *)((uintptr_t)(base + PORT_DIR_OFFSET)) |= mask;

        // Disable pull-up/pull-down resistors
        *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) &= ~mask;
        *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) &= ~mask; // Clear output
    }

    /**
     * @brief Read GPIO pin state
     * @param port Port number (1-10)
     * @param pin Pin number (0-7)
     * @return true if high, false if low
     */
    static inline bool gpio_read(uint8_t port, uint8_t pin)
    {
        uint16_t base = get_port_base(port);
        uint8_t mask = 1 << pin;
        return (*(volatile uint8_t *)((uintptr_t)(base + PORT_IN_OFFSET)) & mask) != 0;
    }

    /**
     * @brief Reset GPIO pin to default state
     * @param port Port number (1-10)
     * @param pin Pin number (0-7)
     */
    static inline void gpio_reset(uint8_t port, uint8_t pin)
    {
        uint16_t base = get_port_base(port);
        uint8_t mask = 1 << pin;

        // Set as input (DIR = 0)
        *(volatile uint8_t *)((uintptr_t)(base + PORT_DIR_OFFSET)) &= ~mask;

        // Disable pull-up/pull-down resistors
        *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) &= ~mask;
        *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) &= ~mask; // Clear output
    }

    /**
     * @brief Open-drain GPIO pin release
     * @param port Port number (1-10)
     * @param pin Pin number (0-7)
     */
    static inline void gpio_open_drain_release(uint8_t port, uint8_t pin)
    {
        uint16_t base = get_port_base(port);
        uint8_t mask = 1 << pin;

        // Set as input (DIR = 0)
        *(volatile uint8_t *)((uintptr_t)(base + PORT_DIR_OFFSET)) &= ~mask;

        // Enable pull-up resistor (REN = 1, OUT = 1)
        *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) |= mask;
        *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) |= mask;
    }

    /**
     * @brief Execute one 62.5ms timer cycle
     */
    static inline void delay_62_5ms(void)
    {
        TA0CTL |= MC__UP | TACLR; // Start timer in up mode
        while (!(TA0CCTL0 & CCIFG))
            ;               // Wait for CCR0
        TA0CTL &= ~MC_3;    // Stop timer
        TA0CCTL0 &= ~CCIFG; // Clear flag
    }

    /**
     * @brief Busy-wait delay in microseconds
     * @param us Microseconds to delay
     */
    static inline void delay_us(uint32_t us)
    {
        // For small delays, use cycle-accurate method
        if (us < 100)
        {
            while (us--)
            {
                __delay_cycles(16); // 16 cycles = 1us @ 16MHz
            }
        }
        // For larger delays, use Timer A0
        else
        {
            // Calculate required timer ticks (16 ticks per us)
            uint32_t ticks = us * 16;
            uint32_t blocks = ticks / 250000; // 62.5ms in ticks (15625 * 16 = 250000)
            uint32_t remainder_ticks = ticks % 250000;

            // Execute full 62.5ms blocks
            for (uint32_t i = 0; i < blocks; i++)
            {
                delay_62_5ms();
            }

            // Handle remaining ticks
            if (remainder_ticks > 0)
            {
                // Set custom CCR0 value
                TA0CCR0 = (remainder_ticks < 65535) ? (remainder_ticks - 1) : 65534;
                TA0CTL |= MC__UP | TACLR; // Start timer
                while (!(TA0CCTL0 & CCIFG))
                    ;               // Wait
                TA0CTL &= ~MC_3;    // Stop timer
                TA0CCTL0 &= ~CCIFG; // Clear flag
            }

            // Restore original CCR0 value
            TA0CCR0 = TIMER_A0_CCR0_VALUE;
        }
    }

    /**
     * @brief Busy-wait delay in milliseconds using Timer A0
     * @param ms Milliseconds to delay
     */
    static inline void delay_ms(uint32_t ms)
    {
        // Calculate number of 62.5ms blocks (16 blocks = 1000ms)
        uint32_t blocks = (ms << 4) / 1000;                  // (ms * 16) / 1000
        uint32_t remainder_ms = ms - ((blocks * 1000) >> 4); // ms - (blocks * 62.5)

        // Execute full 62.5ms blocks
        for (uint32_t i = 0; i < blocks; i++)
        {
            delay_62_5ms();
        }

        // Handle remaining milliseconds
        if (remainder_ms > 0)
        {
            // Calculate delay in microseconds
            uint32_t us = remainder_ms * 1000;
            delay_us(us);
        }
    }

    volatile uint32_t timerA1_overflows = 0;

    void __attribute__((interrupt(TIMER1_A0_VECTOR))) Timer1_A0_ISR(void)
    {
        if (TA1CCTL0 & CCIFG)
        {
            TA1CCTL0 &= ~CCIFG; // Interrupt-Flag löschen
            timerA1_overflows++;

            // LED auf P3.7 toggeln zur Bestätigung
            P3OUT ^= BIT7;
        }
    }

    /**
     * @brief Start the timing counter (Timer A1)
     */
    static inline void start_timer(void)
    {
        timerA1_overflows = 0;
        TA1CTL &= ~MC_3;
        TA1CTL |= TACLR;
        TA1CTL |= MC__CONTINUOUS | TAIE;
    }

    /**
     * @brief Stop the timing counter (Timer A1)
     */
    static inline void stop_timer(void)
    {
        TA1CTL &= ~MC_3; // Stop timer
    }

    /**
     * @brief Reset the timing counter (Timer A1)
     */
    static inline void reset_timer(void)
    {
        TA1CTL |= TACLR; // Clear timer
        timerA1_overflows = 0;
    }

    /**
     * @brief Get current timer value in ticks
     * @return Timer ticks (16-bit value)
     */
    static inline uint32_t get_timer_ticks(void)
    {
        uint32_t overflows;
        uint16_t count;

        // Atomic read without disabling interrupts
        do
        {
            overflows = timerA1_overflows;
            count = TA1R;
            // Repeat if overflow occurred during read
        } while (overflows != timerA1_overflows || (TA1CCTL0 & CCIFG));

        return (overflows << 16) | count;
    }

    /**
     * @brief Convert timer ticks to microseconds
     * @param ticks Timer tick count
     * @return Time in microseconds
     */
    static inline uint32_t ticks_to_us(uint32_t ticks)
    {
        return ticks * 2; // 500 kHz Takt → 2 µs/Tick
    }

    /**
     * @brief Convert timer ticks to milliseconds
     * @param ticks Timer tick count
     * @return Time in milliseconds
     */
    static inline uint32_t ticks_to_ms(uint32_t ticks)
    {
        return ticks / 500; // 500 kHz Takt → 2 µs/Tick → 1000 Ticks = 1 ms
    }

    /**
     * @brief Timer difference in microseconds
     * @param start Start timer value
     * @param end End timer value
     * @return Difference in microseconds
     */
    static inline uint32_t timer_diff_us(uint32_t start, uint32_t end)
    {
        return (end - start) * 2;
    }
    /**
     * @brief Timer difference in milliseconds
     * @param start Start timer value
     * @param end End timer value
     * @return Difference in milliseconds
     */
    static inline uint32_t timer_diff_ms(uint32_t start, uint32_t end)
    {
        return (end - start) / 500;
    }

    /**
     * @brief Generate 32-bit pseudo-random number (LFSR with maximal period)
     * @return 32-bit random value
     */
    static inline uint32_t random32(void)
    {
        static uint32_t lfsr = 0xACE1BACEu; // 32-bit seed (non-zero)

        // Prevent lockup at zero state
        if (lfsr == 0)
            lfsr = 0xACE1BACEu;

        // LFSR step: taps at 32,30,26,25 (maximal period: 2^32 - 1)
        lfsr = (lfsr >> 1) ^ (-(lfsr & 1u) & 0xA0400001u);
        return lfsr;
    }

    /**
     * Device Descriptor Structure
     * Mirrors the memory layout of the device descriptor table
     */
    typedef struct
    {
        uint8_t infoLength;      // 0x01A00
        uint8_t crcLength;       // 0x01A01
        uint16_t crcValue;       // 0x01A02-0x01A03
        uint16_t deviceID;       // 0x01A04-0x01A05
        uint8_t hwRevision;      // 0x01A06
        uint8_t fwRevision;      // 0x01A07
        uint8_t dieRecordTag;    // 0x01A08
        uint8_t dieRecordLength; // 0x01A09
        uint32_t lotWaferID;     // 0x01A0A-0x01A0D
        uint8_t dieRecord;       // 0x01A0E
        uint8_t dieX;            // 0x01A0F
        uint8_t dieY;            // 0x01A10
        uint8_t testResults[3];  // 0x01A11-0x01A13
    } DeviceDescriptor;

    /**
     * @brief Read the entire device descriptor table
     * @return DeviceDescriptor structure with all fields populated
     */
    static inline DeviceDescriptor read_device_descriptor()
    {
        // Cast the memory address to our struct type
        volatile DeviceDescriptor *dd = (volatile DeviceDescriptor *)((uintptr_t)DEVICE_DESCRIPTOR_ADDR);

        // Copy to local struct to ensure safe access
        DeviceDescriptor result;
        result.infoLength = dd->infoLength;
        result.crcLength = dd->crcLength;
        result.crcValue = dd->crcValue;
        result.deviceID = dd->deviceID;
        result.hwRevision = dd->hwRevision;
        result.fwRevision = dd->fwRevision;
        result.dieRecordTag = dd->dieRecordTag;
        result.dieRecordLength = dd->dieRecordLength;
        result.lotWaferID = dd->lotWaferID;
        result.dieRecord = dd->dieRecord;
        result.dieX = dd->dieX;
        result.dieY = dd->dieY;
        result.testResults[0] = dd->testResults[0];
        result.testResults[1] = dd->testResults[1];
        result.testResults[2] = dd->testResults[2];

        return result;
    }

    /**
     * @brief Read specific field from device descriptor
     * @param field_offset Offset from base address (0x1A00)
     * @param size Size of data to read (1, 2, or 4 bytes)
     * @return Value read from memory
     */
    static inline uint32_t read_descriptor_field(uint16_t field_offset, uint8_t size)
    {
        volatile uint8_t *addr = (volatile uint8_t *)((uintptr_t)(DEVICE_DESCRIPTOR_ADDR + field_offset));

        switch (size)
        {
        case 1:
            return *addr;
        case 2:
            return *((volatile uint16_t *)addr);
        case 4:
            return *((volatile uint32_t *)addr);
        default:
            return 0;
        }
    }

    // Convenience functions for specific fields
    static inline uint8_t get_info_length()
    {
        return read_descriptor_field(0x00, 1);
    }

    static inline uint8_t get_crc_length()
    {
        return read_descriptor_field(0x01, 1);
    }

    static inline uint16_t get_crc_value()
    {
        return read_descriptor_field(0x02, 2);
    }

    static inline uint16_t get_device_id()
    {
        return read_descriptor_field(0x04, 2);
    }

    static inline uint8_t get_hw_revision()
    {
        return read_descriptor_field(0x06, 1);
    }

    static inline uint8_t get_fw_revision()
    {
        return read_descriptor_field(0x07, 1);
    }

    static inline uint32_t get_lot_wafer_id()
    {
        return read_descriptor_field(0x0A, 4);
    }

    static inline uint8_t get_die_x_position()
    {
        return read_descriptor_field(0x0F, 1);
    }

    static inline uint8_t get_die_y_position()
    {
        return read_descriptor_field(0x10, 1);
    }

#ifdef __cplusplus
}
#endif

#endif // MSP430FR5994_HELPER_H