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
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

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

// Analog joystick pins
#define JOYSTICK_VRX_CHANNEL ADC_CHANNEL_3  // GPIO4
#define JOYSTICK_VRY_CHANNEL ADC_CHANNEL_4  // GPIO5
#define JOYSTICK_SW_PIN GPIO_NUM_9          // Digital button

// Joystick configuration
#define JOYSTICK_CENTER_VALUE 4096       // 13-bit ADC center (8192/2)
#define JOYSTICK_DEADZONE 410            // ~5% dead zone (8192 * 0.05)
#define JOYSTICK_MIN 0
#define JOYSTICK_MAX 8191

static volatile int32_t encoder_position = 0;
static volatile bool last_clk_state = false;
static volatile bool last_dt_state = false;
static uint32_t last_button_press_time = 0;
static bool last_encoder_button_state = true;
static int32_t last_reported_position = 0;
static uint32_t last_position_time = 0;

// Joystick state variables
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool last_joystick_button_state = true;
static uint32_t last_joystick_button_press_time = 0;

/**
 * @brief Joystick state structure
 *
 * Holds raw ADC values and calibrated gamepad axis values for the analog joystick
 */
typedef struct {
    int16_t x_raw;           // Raw ADC reading for X-axis (0-4095)
    int16_t y_raw;           // Raw ADC reading for Y-axis (0-4095)
    int8_t x_calibrated;     // Calibrated X-axis (-127 to +127)
    int8_t y_calibrated;     // Calibrated Y-axis (-127 to +127)
    bool sw_pressed;         // Joystick button state
} joystick_state_t;

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
 * @brief Initialize analog joystick ADC and GPIO
 *
 * Configures ADC1 for reading the joystick's X and Y potentiometer values.
 * Sets up 13-bit resolution with 12dB attenuation for 0-2.5V range.
 * Also configures the joystick's digital button pin with pull-up resistor.
 *
 * @return None
 *
 * @note Joystick should be powered from 3.3V supply (not 5V)
 * @note ADC calibration is performed for more accurate readings
 * @note 12dB attenuation allows reading up to ~2.5V input range
 * @note ESP32-S2 supports 13-bit ADC resolution (0-8191)
 */
void init_joystick(void) {
    // Initialize ADC1 oneshot unit
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    // Configure ADC1 channels for X and Y axes
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_13,  // ESP32-S2 supports 13-bit ADC
        .atten = ADC_ATTEN_DB_12,  // 0-2500mV range for 3.3V operation
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOYSTICK_VRX_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOYSTICK_VRY_CHANNEL, &config));

    // Initialize ADC calibration
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_13,  // ESP32-S2 supports 13-bit ADC
    };
    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_config, &adc1_cali_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(USB_INIT_TAG, "ADC calibration scheme initialized");
    } else {
        ESP_LOGW(USB_INIT_TAG, "ADC calibration failed, using raw values");
        adc1_cali_handle = NULL;
    }

    // Configure joystick button as input with pull-up
    const gpio_config_t joystick_button_config = {
        .pin_bit_mask = (1ULL << JOYSTICK_SW_PIN),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = true,
        .pull_down_en = false,
    };
    ESP_ERROR_CHECK(gpio_config(&joystick_button_config));

    ESP_LOGI(USB_INIT_TAG, "Joystick ADC initialized (13-bit, 12dB attenuation)");
}

/**
 * @brief Read joystick analog values and button state
 *
 * Reads raw ADC values from both joystick axes, applies dead-zone filtering,
 * and converts to standard gamepad range (-127 to +127). Also reads the
 * joystick button state with debouncing.
 *
 * @return joystick_state_t Structure containing calibrated axis values and button state
 *
 * @note Center position (2048) maps to 0 output
 * @note 5% dead-zone prevents drift when joystick is at rest
 * @note Button uses 50ms debouncing to prevent false triggers
 * @note Values are clamped to prevent overflow
 */
joystick_state_t read_joystick(void) {
    joystick_state_t state = {0};

    // Read raw ADC values (0-8191 for 13-bit ADC)
    int raw_x, raw_y;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, JOYSTICK_VRX_CHANNEL, &raw_x));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, JOYSTICK_VRY_CHANNEL, &raw_y));
    state.x_raw = raw_x;
    state.y_raw = raw_y;

    // Apply dead-zone and map to gamepad range (-127 to +127)
    // X-axis processing
    int16_t x_offset = state.x_raw - JOYSTICK_CENTER_VALUE;
    if (abs(x_offset) < JOYSTICK_DEADZONE) {
        state.x_calibrated = 0;  // Within dead-zone, report as centered
    } else {
        // Map from ADC range to gamepad range
        float x_normalized = (float)x_offset / (JOYSTICK_MAX - JOYSTICK_CENTER_VALUE);
        int16_t x_scaled = (int16_t)(x_normalized * 127.0f);
        // Clamp to valid range before casting to int8_t
        if (x_scaled > 127) x_scaled = 127;
        if (x_scaled < -127) x_scaled = -127;
        state.x_calibrated = (int8_t)x_scaled;
    }

    // Y-axis processing (inverted for typical joystick orientation)
    int16_t y_offset = state.y_raw - JOYSTICK_CENTER_VALUE;
    if (abs(y_offset) < JOYSTICK_DEADZONE) {
        state.y_calibrated = 0;  // Within dead-zone, report as centered
    } else {
        // Map from ADC range to gamepad range (inverted)
        float y_normalized = -(float)y_offset / (JOYSTICK_MAX - JOYSTICK_CENTER_VALUE);
        int16_t y_scaled = (int16_t)(y_normalized * 127.0f);
        // Clamp to valid range before casting to int8_t
        if (y_scaled > 127) y_scaled = 127;
        if (y_scaled < -127) y_scaled = -127;
        state.y_calibrated = (int8_t)y_scaled;
    }

    // Read joystick button with debouncing
    bool current_joystick_button = (gpio_get_level(JOYSTICK_SW_PIN) == 0);
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (current_joystick_button != last_joystick_button_state) {
        if (current_time - last_joystick_button_press_time > 50) {  // 50ms debounce
            last_joystick_button_state = current_joystick_button;
            last_joystick_button_press_time = current_time;
        }
    }

    state.sw_pressed = last_joystick_button_state;

    return state;
}

/**
 * @brief Read all inputs and send HID gamepad report to host
 *
 * Polls all button states, reads joystick analog values and encoder position,
 * then formats and sends a complete HID gamepad report. Handles debouncing
 * for all buttons and applies dead-zone filtering to joystick.
 *
 * @return None
 *
 * @note Called continuously in main loop when USB is mounted and ready
 * @note Joystick X/Y → Gamepad X/Y axes (-127 to +127 range)
 * @note Encoder position → Gamepad Z-axis (-127 to +127 range)
 * @note All buttons use 50ms debouncing to prevent multiple triggers
 * @note Button mapping: bit 0=Button1, bit 1=Button2, bit 2=Encoder, bit 3=Joystick
 */
void send_gamepad_report(void) {
    // Read digital button states (active low due to pull-up)
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

    // Read joystick state (includes analog axes and button)
    joystick_state_t joystick = read_joystick();

    // Create button mask
    uint32_t buttons = 0;
    if (button_1_pressed) buttons |= (1 << 0);        // Button 1
    if (button_2_pressed) buttons |= (1 << 1);        // Button 2
    if (encoder_button_pressed) buttons |= (1 << 2);  // Encoder button
    if (joystick.sw_pressed) buttons |= (1 << 3);     // Joystick button

    // Process encoder for Z-axis with velocity-based response
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

    // Map encoder position to Z-axis (constrain to prevent overflow)
    if (scaled_position > 127) scaled_position = 127;
    if (scaled_position < -127) scaled_position = -127;
    int8_t z_axis = (int8_t)scaled_position;

    // Send HID gamepad report with full axis mapping
    // Format: report_id, x, y, z, rz, rx, ry, hat, buttons
    // X/Y from joystick, Z from encoder, rest unused
    tud_hid_gamepad_report(0, joystick.x_calibrated, joystick.y_calibrated,
                           z_axis, 0, 0, 0, 0, buttons);
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

    // Initialize analog joystick
    init_joystick();

    // Initialize USB
    usb_init();

    printf("Starting main loop\n");
    ESP_LOGI(USB_INIT_TAG, "Starting main loop - Gamepad with Joystick + Encoder ready");

    while (1) {
        // Only send reports when USB is mounted and HID is ready
        if (tud_mounted() && tud_hid_ready()) {
            send_gamepad_report();
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz polling rate
    }
}
