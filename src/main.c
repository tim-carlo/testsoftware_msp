#include "msp430fr5994.h"
#include <stdint.h>
#include <stdbool.h>

// Pin definitions using helper library constants
#define LED_RED_PORT 1
#define LED_RED_PIN 0 // P1.0
#define LED_GREEN_PORT 1
#define LED_GREEN_PIN 1 // P1.1
#define TEST_PORT 3
#define TEST_PIN 7 // P3.7


// Protocol configuration
#define INITIAL_DELAY_MAX_MS 1000 // Max random initialization delay
#define SIGNAL_DURATION_MS 50     // Signal duration for reliable detection
#define RESPONSE_TIMEOUT_MS 150   // Response timeout window
#define DEBOUNCE_SAMPLES 5        // Number of samples for debouncing
#define DEBOUNCE_DELAY_US 2000    // 2ms between debounce samples

// Device states
typedef enum
{
    INIT_MODE,      // Initialization state
    INITIATOR_MODE, // Initiator role (sends SYN first)
    RESPONDER_MODE, // Responder role (answers SYN)
    ERROR_MODE,     // Error state
    SUCCESS_MODE    // Successful handshake state
} State;

// Global state variable
volatile State state = INIT_MODE;

bool is_signal_active(void)
{
    uint8_t low_count = 0;
    for (uint8_t i = 0; i < DEBOUNCE_SAMPLES; i++)
    {
        if (!gpio_read(TEST_PORT, TEST_PIN))
            low_count++;
        delay_us(DEBOUNCE_DELAY_US); // Space between samples
    }
    return (low_count >= (DEBOUNCE_SAMPLES - 1)); // >80% low samples = active
}

bool wait_for_signal(uint32_t timeout_ms)
{
    uint16_t start_ticks = get_timer_ticks();

    while (1)
    {
        // Calculate elapsed time
        uint32_t elapsed_ms = timer_diff_ms(start_ticks, get_timer_ticks());

        // Timeout check
        if (elapsed_ms >= timeout_ms)
            break;

        // Signal detection
        if (is_signal_active())
        {
            // Wait for signal release
            while (is_signal_active())
            {
                elapsed_ms = timer_diff_ms(start_ticks, get_timer_ticks());
                if (elapsed_ms >= timeout_ms)
                    return false;
            }
            return true;
        }
    }
    return false;
}

int main(void)
{
    // Initialize hardware
    io_init(); // Initialize clocks and UART
    printf("MSP430FR5994 Communication Protocol\n");

    // Configure LED pins as outputs
    gpio_output_init(LED_RED_PORT, LED_RED_PIN);     // Red LED off
    gpio_output_init(LED_GREEN_PORT, LED_GREEN_PIN); // Green LED off

    // Configure TEST_PIN as open-drain with pull-up
    gpio_open_drain_release(TEST_PORT, TEST_PIN);
    gpio_pullup_init(TEST_PORT, TEST_PIN);

    // Initialize timing system
    reset_timer();
    start_timer();

    // Main state machine loop
    while (1)
    {
        switch (state)
        {
        case INIT_MODE:
        {
            printf("INIT_MODE\n");
            gpio_open_drain_release(TEST_PORT, TEST_PIN); // Ensure pin is released

            // Generate random initialization delay (0-1000ms)
            uint32_t initial_delay = random32() % (INITIAL_DELAY_MAX_MS + 1);
            printf("Initial delay: %lu ms\n", initial_delay);

            start_timer(); // Start the timer for delay measurement
            uint16_t start_ticks = get_timer_ticks();
            bool signal_received = false;

            // Wait period with signal monitoring
            while (timer_diff_ms(start_ticks, get_timer_ticks()) < initial_delay)
            {
                if (is_signal_active())
                {
                    printf("SYN detected\n");
                    signal_received = true;
                    break;
                }
            }
            stop_timer(); // Stop the timer

            if (signal_received)
            {
                state = RESPONDER_MODE; // Become responder
            }
            else
            {
                // Send SYN signal
                gpio_drive_low(TEST_PORT, TEST_PIN);
                delay_ms(SIGNAL_DURATION_MS);
                gpio_open_drain_release(TEST_PORT, TEST_PIN);
                state = INITIATOR_MODE; // Become initiator
            }
            break;
        }

        case INITIATOR_MODE:
        {
            printf("INITIATOR_MODE\n");
            reset_timer(); // Reset for accurate timeout measurement

            // Wait for SYN-ACK response
            if (wait_for_signal(RESPONSE_TIMEOUT_MS))
            {
                printf("SYN+ACK received\n");

                // Send ACK confirmation
                gpio_drive_low(TEST_PORT, TEST_PIN);
                delay_ms(SIGNAL_DURATION_MS);
                gpio_open_drain_release(TEST_PORT, TEST_PIN);

                state = SUCCESS_MODE; // Handshake successful
            }
            else
            {
                printf("Timeout waiting for SYN+ACK\n");
                state = ERROR_MODE; // Handshake failed
            }
            break;
        }

        case RESPONDER_MODE:
        {
            printf("RESPONDER_MODE\n");

            // Send SYN-ACK response
            gpio_drive_low(TEST_PORT, TEST_PIN);
            delay_ms(SIGNAL_DURATION_MS);
            gpio_open_drain_release(TEST_PORT, TEST_PIN);

            // Wait for ACK confirmation
            if (wait_for_signal(RESPONSE_TIMEOUT_MS))
            {
                printf("ACK received\n");
                state = SUCCESS_MODE; // Handshake successful
            }
            else
            {
                printf("Timeout waiting for ACK\n");
                state = ERROR_MODE; // Handshake failed
            }
            break;
        }

        case ERROR_MODE:
        {
            printf("ERROR_MODE\n");
            // Blink red LED 3 times
            for (uint8_t i = 0; i < 3; i++)
            {
                gpio_drive_high(LED_RED_PORT, LED_RED_PIN);
                delay_ms(200);
                gpio_drive_low(LED_RED_PORT, LED_RED_PIN);
                delay_ms(200);
            }
            state = INIT_MODE; // Return to initialization
            break;
        }

        case SUCCESS_MODE:
        {
            printf("SUCCESS_MODE\n");
            // Solid green LED indicates success
            gpio_drive_high(LED_GREEN_PORT, LED_GREEN_PIN);

            break;
        }
        }
    }
    return 0;
}