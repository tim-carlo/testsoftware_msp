#include <msp430fr5994_helper.h>
#include <stdint.h>
#include <stdbool.h>

// Pin definitions using helper library constants
#define LED_RED_PORT 1
#define LED_RED_PIN 0 // P1.0
#define LED_GREEN_PORT 1
#define LED_GREEN_PIN 1 // P1.1



// Protocol configuration
#define INITIAL_DELAY_MAX_MS 1000 // Max random initialization delay
#define SIGNAL_DURATION_MS 50     // Signal duration for reliable detection
#define RESPONSE_TIMEOUT_MS 150   // Response timeout window
#define DEBOUNCE_SAMPLES 5        // Number of samples for debouncing
#define DEBOUNCE_DELAY_US 2000    // 2ms between debounce samples


void rising_edge_handler(uint32_t gpio)
{
    // Handle rising edge interrupt
    printf("Rising edge detected on GPIO %lu\n", gpio);
}
void falling_edge_handler(uint32_t gpio)
{
    // Handle falling edge interrupt
    printf("Falling edge detected on GPIO %lu\n", gpio);
}

int main(void)
{

    io_init(); // Initialize GPIOs

    // Initialize LEDs
    gpio_output_init(LED_RED_PORT * 8 + LED_RED_PIN);
    gpio_output_init(LED_GREEN_PORT * 8 + LED_GREEN_PIN);
    printf("LEDs initialized.\n");

    // Initialize test pin
    gpio_input_init(TEST_PORT * 8 + TEST_PIN);

    // Initialize the interrupt handlers
    gpio_listen_interrupt_on_all_pins(0, rising_edge_handler, falling_edge_handler);

    // Main loop
    while (1)
    {
        // Example: Toggle red LED every second
        gpio_drive_high(LED_RED_PORT * 8 + LED_RED_PIN);
       // printf("Red LED ON\n");
        delay_ms(1000);
        gpio_drive_low(LED_RED_PORT * 8 + LED_RED_PIN);
       // printf("Red LED OFF\n");
        delay_ms(1000);
    }

    return 0;
}