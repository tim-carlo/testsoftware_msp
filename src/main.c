#include <msp430fr5994_helper.h>
#include <stdint.h>
#include <stdbool.h>

// Pin definitions using helper library constants
#define LED_RED_PORT 1
#define LED_RED_PIN 0 // P1.0
#define LED_GREEN_PORT 1
#define LED_GREEN_PIN 1 // P1.1
#define ABS_PIN(port, pin) (((port) << 3) | (pin))


uint64_t gpio_blacklist_mask = 0;



// Protocol configuration
#define INITIAL_DELAY_MAX_MS 1000 // Max random initialization delay
#define SIGNAL_DURATION_MS 50     // Signal duration for reliable detection
#define RESPONSE_TIMEOUT_MS 150   // Response timeout window
#define DEBOUNCE_SAMPLES 5        // Number of samples for debouncing
#define DEBOUNCE_DELAY_US 2000    // 2ms between debounce samples

void rising_edge_handler(uint32_t gpio)
{
    printf("rising: P%lu.%lu\n", gpio >> 3, gpio & 0x07);
}
void falling_edge_handler(uint32_t gpio)
{
    printf("falling: P%lu.%lu\n", gpio >> 3, gpio & 0x07);
}

void print_gpio_states_binary(uint64_t gpio_states)
{
    for (int i = 63; i >= 0; i--)
    {
        printf("%c", (gpio_states & ((uint64_t)1 << i)) ? '1' : '0');
        if (i % 8 == 0 && i != 0)
            printf(" "); // Optional: group by bytes
    }
    printf("\n");
}

void print_blacklist_pins(uint64_t blacklist_mask)
{
    printf("Blacklisted pins:\n");
    for (int i = 0; i < 64; i++)
    {
        if (blacklist_mask & (1ULL << i))
        {
            uint8_t port = i >> 3;
            uint8_t pin = i & 0x07;
            printf("P%u.%u\n", port, pin);
        }
    }
}

int main(void)
{

    io_init(); // Initialize GPIOs

    gpio_blacklist_mask = 0xFFFFFFFFFFFFFFFF; // All pins blacklisted

    for (uint8_t pin = 4; pin <= 7; pin++) {
        gpio_blacklist_mask &= ~(1ULL << ABS_PIN(3, pin)); // Allow P3.4 to P3.7
    }
    // For the buttons on P5.5 and P5.6
    gpio_blacklist_mask &= ~(1ULL << ABS_PIN(5, 5)); // Allow P5.5
    gpio_blacklist_mask &= ~(1ULL << ABS_PIN(5, 6)); // Allow P5.6
    gpio_blacklist_mask |= (1ULL << ABS_PIN(3, 4)); // Blacklist P3.4


    print_blacklist_pins(~gpio_blacklist_mask);


    printf("LEDs initialized.\n");
    gpio_listen_on_all_pins_polling(gpio_blacklist_mask, rising_edge_handler, falling_edge_handler);


    // Initialize the interrupt handlers
    //gpio_listen_interrupt_on_all_pins(0, rising_edge_handler, falling_edge_handler);

    // Main loop
    while (1)
    {
        // Read P3.7 in each loop iteration and print its state
        __delay_cycles(1000); // Small delay to avoid flooding output (adjust as needed)
    }

    return 0;
}