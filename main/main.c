/*
 * Christopher Milian: 2025
 *
 * christophermilian16@gmail.com
 * 
 * Custom Fishing Game Controller for the ESP32-S2
 * % idf.py -p /dev/tty.usbserial-210 -b 115200 flash
 */

#include <stdlib.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// Use ESP TinyUSB wrapper instead of raw TinyUSB
#include "tinyusb.h"
#include "tusb.h"
#include "class/hid/hid_device.h"

#define APP_BUTTON (GPIO_NUM_0) // Use BOOT signal by default
static const char *USB_INIT_TAG = "usb_init";

/************* TinyUSB descriptors ****************/
#define TUSB_DESC_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)

/**
 * @brief HID Report Descriptor for a Gamepad (2 buttons)
 * 
 * Using proper TinyUSB HID report descriptor macros
 */
const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_GAMEPAD()
};

/**
 * @brief String descriptor
 */
const char* hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "Espressif",           // 1: Manufacturer
    "TinyUSB Fishing Game Controller",      // 2: Product
    "80:65:99:41:33:5C",      // 3: Serials, should use chip ID or MAC Address
    "Fishing Controller HID",  // 4: HID
};

/**
 * @brief Configuration descriptor
 */
static const uint8_t hid_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

/********* TinyUSB HID callbacks ***************/

// Invoked when received GET HID REPORT DESCRIPTOR request
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void) instance;
    return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;

    return 0;
}

// Invoked when received SET_REPORT control request
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) bufsize;
}

/********* Application ***************/

#define BUTTON_1_PIN GPIO_NUM_8
#define BUTTON_2_PIN GPIO_NUM_2

#define ENCODER_CLK_PIN GPIO_NUM_3
#define ENCODER_DT_PIN GPIO_NUM_6
#define ENCODER_SW_PIN GPIO_NUM_7

static volatile int32_t encoder_position = 0;
static volatile bool last_clk_state = false;
static volatile bool last_dt_state = false;
static uint32_t last_button_press_time = 0;
static bool last_encoder_button_state = true;
static int32_t last_reported_position = 0;
static uint32_t last_position_time = 0;

/**
 * @brief Initialize GPIO pins for push buttons
 *
 * Configures BUTTON_1_PIN and BUTTON_2_PIN as input pins with internal
 * pull-up resistors enabled. Buttons are expected to be active-low
 * (pressed when pin reads 0).
 *
 * @return None
 *
 * @note Pull-up resistors are enabled, so buttons should connect pin to GND
 * @note Interrupts are disabled - buttons are polled in main loop
 */
void init_buttons(void) {
    const gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BUTTON_1_PIN) | (1ULL << BUTTON_2_PIN),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = true,
        .pull_down_en = false,
    };

    ESP_ERROR_CHECK(gpio_config(&button_config));
}

/**
 * @brief Interrupt service routine for rotary encoder rotation detection
 *
 * This ISR is triggered on any edge change of the CLK pin. It uses full
 * quadrature decoding on both edges to handle fast rotation more reliably.
 * Tracks both CLK and DT state changes for better accuracy.
 *
 * @param arg Unused interrupt argument
 *
 * @note This function runs in interrupt context and should be kept minimal
 * @note Uses full quadrature encoding on both rising and falling edges
 * @note More robust for high-speed rotation than single-edge detection
 */
static void IRAM_ATTR encoder_isr_handler(void* arg) {
    bool clk_state = gpio_get_level(ENCODER_CLK_PIN);
    bool dt_state = gpio_get_level(ENCODER_DT_PIN);

    // Full quadrature decoding - check both CLK and DT state changes
    if (clk_state != last_clk_state) {
        if (clk_state == dt_state) {
            encoder_position--;  // Counter-clockwise
        } else {
            encoder_position++;  // Clockwise
        }
        last_clk_state = clk_state;
    }
}

/**
 * @brief Initialize KY-040 rotary encoder GPIO pins and interrupts
 *
 * Configures the rotary encoder pins (CLK, DT, SW) as inputs with pull-up
 * resistors. Sets up interrupt handling on the CLK pin for rotation detection.
 * The SW (switch) pin is configured for polling instead of interrupts.
 *
 * @return None
 *
 * @note CLK pin generates interrupts on any edge change
 * @note SW pin interrupts are disabled - button is polled with debouncing
 * @note Installs global GPIO ISR service if not already installed
 * @note Initializes last_clk_state for proper edge detection
 */
void init_rotary_encoder(void) {
    const gpio_config_t encoder_config = {
        .pin_bit_mask = (1ULL << ENCODER_CLK_PIN) | (1ULL << ENCODER_DT_PIN) | (1ULL << ENCODER_SW_PIN),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE,
        .pull_up_en = true,
        .pull_down_en = false,
    };

    ESP_ERROR_CHECK(gpio_config(&encoder_config));

    gpio_set_intr_type(ENCODER_SW_PIN, GPIO_INTR_DISABLE);

    last_clk_state = gpio_get_level(ENCODER_CLK_PIN);
    last_dt_state = gpio_get_level(ENCODER_DT_PIN);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENCODER_CLK_PIN, encoder_isr_handler, NULL));
}

/**
 * @brief Read all inputs and send HID gamepad report to host
 *
 * Polls all button states and reads the current encoder position, then
 * formats and sends a complete HID gamepad report. Handles debouncing
 * for the encoder button and constrains encoder position to valid range.
 *
 * @return None
 *
 * @note Called continuously in main loop when USB is mounted and ready
 * @note Encoder position maps to X-axis (-127 to +127 range)
 * @note Y-axis always set to 0 since encoder only provides 1D input
 * @note Encoder button uses 50ms debouncing to prevent multiple triggers
 * @note Button mapping: bit 0=Button1, bit 1=Button2, bit 2=Encoder button
 */
void send_gamepad_report(void) {
    // Read button states (active low due to pull-up)
    bool button_1_pressed = (gpio_get_level(BUTTON_1_PIN) == 0);
    bool button_2_pressed = (gpio_get_level(BUTTON_2_PIN) == 0);

    // Read encoder button state with proper debouncing
    bool current_encoder_button_state = (gpio_get_level(ENCODER_SW_PIN) == 0);
    bool encoder_button_pressed = false;
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Only update state if button state changed and enough time has passed
    if (current_encoder_button_state != last_encoder_button_state) {
        if (current_time - last_button_press_time > 50) { // 50ms debounce
            last_encoder_button_state = current_encoder_button_state;
            last_button_press_time = current_time;
        }
    }

    // Button is pressed if current debounced state is pressed
    encoder_button_pressed = last_encoder_button_state;

    // Create button mask
    uint32_t buttons = 0;
    if (button_1_pressed) buttons |= (1 << 0);        // Button 1
    if (button_2_pressed) buttons |= (1 << 1);        // Button 2
    if (encoder_button_pressed) buttons |= (1 << 2);  // Encoder button

    // Convert encoder position to analog stick values (-127 to 127)
    // Since the encoder only moves in one dimension, we dont need Y axis
    int8_t x_axis = 0;

    // Calculate velocity-based response for better fast rotation handling
    int32_t position_delta = encoder_position - last_reported_position;
    uint32_t time_delta = current_time - last_position_time;

    // Apply velocity scaling for fast movements (more responsive)
    int32_t scaled_position = encoder_position;
    if (time_delta > 0 && abs(position_delta) > 2) {
        // Fast movement detected - apply scaling factor
        float velocity = (float)abs(position_delta) / time_delta;
        if (velocity > 0.5f) { // Threshold for "fast" movement
            scaled_position = encoder_position + (position_delta > 0 ? 10 : -10);
        }
    }

    // Update tracking variables
    last_reported_position = encoder_position;
    last_position_time = current_time;

    // Map encoder position to X-axis (constrain to prevent overflow)
    if (scaled_position > 127) scaled_position = 127;
    if (scaled_position < -127) scaled_position = -127;
    x_axis = (int8_t)scaled_position;

    // Send HID gamepad report
    // Format: report_id, x, y, z, rz, rx, ry, hat, buttons
    tud_hid_gamepad_report(0, x_axis, 0, 0, 0, 0, 0, 0, buttons);
}

/**
 * @brief Initialize USB HID gamepad functionality
 *
 * Sets up TinyUSB with custom HID gamepad descriptors and configuration.
 * Uses the ESP-IDF TinyUSB wrapper for easier integration with ESP32-S2.
 * Configures the device as a USB HID gamepad with custom string descriptors.
 *
 * @return None
 *
 * @note Uses internal USB PHY (not external)
 * @note Custom HID report descriptor defines gamepad with 3 buttons and 2 axes
 * @note Device appears as "TinyUSB Fishing Game Controller" to host
 * @note Must be called after GPIO initialization but before main loop
 */
void usb_init(void) {
    ESP_LOGI(USB_INIT_TAG, "USB initialization");

    // Initialize TinyUSB using ESP wrapper
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,  // Use default from menuconfig
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false, // Use internal USB PHY
        .configuration_descriptor = hid_configuration_descriptor,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(USB_INIT_TAG, "USB initialized");
}

/**
 * @brief Main application entry point
 *
 * Initializes all hardware components (buttons, encoder, USB) and enters
 * the main polling loop. Continuously reads input states and sends HID
 * reports to the host at 100Hz when USB is connected and ready.
 *
 * @return None (function never returns)
 *
 * @note Initialization order: GPIO buttons → rotary encoder → USB
 * @note Main loop runs at 100Hz (10ms delay between iterations)
 * @note HID reports only sent when USB device is mounted and HID ready
 * @note This is the FreeRTOS main task - blocking is acceptable here
 */
void app_main(void) {
    // Initialize GPIO buttons
    init_buttons();

    // Initialize rotary encoder
    init_rotary_encoder();

    // Initialize USB
    usb_init();
    
    printf("Starting main loop\n");
    ESP_LOGI(USB_INIT_TAG, "Starting main loop");

    while (1) {
        // Only send reports when USB is mounted and HID is ready
        if (tud_mounted() && tud_hid_ready()) {
            send_gamepad_report();
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz polling rate
    }
}
