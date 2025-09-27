# ESP32-S2 Fishing Game Controller

A custom USB HID gamepad controller for fishing games, featuring a KY-040 rotary encoder for realistic reel simulation and push buttons for game controls.

## Features

- **USB HID Gamepad**: Appears as a standard gamepad to host systems
- **KY-040 Rotary Encoder**: Provides analog stick X-axis control for realistic fishing reel simulation
- **Push Buttons**: Two additional buttons plus encoder push button (3 total)
- **High-Speed Rotation Support**: Enhanced quadrature decoding with velocity-based scaling
- **Debounced Inputs**: Reliable button press detection with proper debouncing

## Hardware Components

- ESP32-S2 development board
- KY-040 rotary encoder module
- 2x push buttons (optional)
- Breadboard and jumper wires

As a USB stack, TinyUSB component is used for optimal performance and compatibility.

## Hardware Setup

### Pin Assignment

**KY-040 Rotary Encoder:**
```
KY-040  →  ESP32-S2
VCC     →  3.3V
GND     →  GND
CLK     →  GPIO3
DT      →  GPIO6
SW      →  GPIO7
```

**Push Buttons (Optional):**
```
Button 1 → GPIO8 (active low, internal pull-up)
Button 2 → GPIO2 (active low, internal pull-up)
```

### Wiring Notes

- All buttons use internal pull-up resistors and are active-low
- Connect button switches between GPIO pin and GND
- KY-040 module requires 3.3V power supply
- Use short, quality jumper wires for stable encoder operation

## Software Setup

### ESP-IDF Development

1. **Install ESP-IDF** (if not already installed):
   ```bash
   # Follow official ESP-IDF installation guide
   https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/get-started/
   ```

2. **Build and Flash**:
   ```bash
   idf.py build
   idf.py -p /dev/ttyUSB0 flash monitor
   ```

### Testing Setup with Poetry

For running automated tests and linting, we use Poetry for dependency management:

1. **Install Poetry** (if not already installed):
   ```bash
   curl -sSL https://install.python-poetry.org | python3 -
   ```

2. **Install Testing Dependencies**:
   ```bash
   poetry install
   ```

3. **Run Tests**:
   ```bash
   # Run the hardware-in-the-loop test
   poetry run pytest pytest_usb_device_hid.py -v

   # Run linting
   poetry run pylint pytest_usb_device_hid.py

   # Format code
   poetry run black pytest_usb_device_hid.py
   ```

### What Poetry Installs

The `pyproject.toml` file defines all testing dependencies:
- **pytest & pytest-embedded**: For ESP-IDF hardware testing
- **pylint, black, isort**: Code quality tools
- **mypy**: Type checking

This ensures consistent testing environments across different machines.

_Note:_ In case your board doesn't have micro-USB connector connected to USB-OTG peripheral, you may have to DIY a cable and connect **D+** and **D-** to the pins listed below.

See common pin assignments for USB Device examples from [upper level](../../README.md#common-pin-assignments).

Boot signal (GPIO0) is used to send HID reports to USB host.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```bash
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

After the flashing you should see the output at idf monitor:

```
I (21) boot: ESP-IDF v5.5-dev-3951-ga74725a31b 2nd stage bootloader
...
I (254) main_task: Calling app_main()
I (254) usb_init: USB initialization
W (254) tusb_desc: No Device descriptor provided, using default.
I (264) tusb_desc: 
┌─────────────────────────────────┐
│  USB Device Descriptor Summary  │
├───────────────────┬─────────────┤
│bDeviceClass       │ 0           │
├───────────────────┼─────────────┤
│bDeviceSubClass    │ 0           │
├───────────────────┼─────────────┤
│bDeviceProtocol    │ 0           │
├───────────────────┼─────────────┤
│bMaxPacketSize0    │ 64          │
├───────────────────┼─────────────┤
│idVendor           │ 0x303a      │
├───────────────────┼─────────────┤
│idProduct          │ 0x4004      │
├───────────────────┼─────────────┤
│bcdDevice          │ 0x100       │
├───────────────────┼─────────────┤
│iManufacturer      │ 0x1         │
├───────────────────┼─────────────┤
│iProduct           │ 0x2         │
├───────────────────┼─────────────┤
│iSerialNumber      │ 0x3         │
├───────────────────┼─────────────┤
│bNumConfigurations │ 0x1         │
└───────────────────┴─────────────┘
I (434) TinyUSB: TinyUSB Driver installed
I (434) usb_init: USB initialized
I (434) usb_init: Starting main loop
```
