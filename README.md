# ESP32-S2 Fishing Game Controller

A custom USB HID gamepad controller for fishing games, featuring a KY-040 rotary encoder for realistic reel simulation, analog joystick for rod control, and multiple tactile buttons for game actions.

## Features

- **USB HID Gamepad**: Appears as a standard gamepad to host systems (Windows, macOS, Linux)
- **Analog Joystick**: 2-axis analog joystick (X/Y axes) with 13-bit ADC resolution for precise rod control
- **KY-040 Rotary Encoder**: Provides Z-axis control for realistic fishing reel simulation with velocity-based scaling
- **7 Digital Buttons**: Seven tactile buttons
- **High-Speed Rotation Support**: Enhanced quadrature decoding with velocity-based scaling for responsive reel action
- **Debounced Inputs**: Reliable button press detection with 50ms debouncing on all buttons
- **Dead-zone Filtering**: 5% dead-zone on joystick axes prevents drift when at rest
- **Low Power Operation**: Optimized for USB bus power with proper power management

## Hardware Components

- ESP32-S2 development board
- Analog joystick module (2-axis with push button)
- KY-040 rotary encoder module
- 7x Mini Momentary Push Button
- Mini Snap Switch with long Leg, Single Pull Double Throw (SPDT)

As a USB stack, TinyUSB component is used for optimal performance and compatibility.

## Hardware Setup

[Espressif ESP32-S2 Pin Diagram](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s2/_images/esp32-s2-devkitc-1-v1-pinout.png)

### Pin Assignment

**Analog Joystick:**
```
Joystick  →  ESP32-S2
VCC       →  3.3V (IMPORTANT: Use 3.3V, not 5V!)
GND       →  GND
VRX       →  GPIO4 (ADC1 Channel 3)
VRY       →  GPIO5 (ADC1 Channel 4)
SW        →  GPIO9 (active low, internal pull-up)
```

**KY-040 Rotary Encoder:**
```
KY-040  →  ESP32-S2
VCC     →  3.3V
GND     →  GND
CLK     →  GPIO3
DT      →  GPIO6
SW      →  GPIO7
```

**Tactile Push Buttons:**
```
Button 1 → GPIO8  (active low, internal pull-up)
Button 2 → GPIO2  (active low, internal pull-up)
Button 3 → GPIO10 (active low, internal pull-up)
Button 4 → GPIO11 (active low, internal pull-up)
Button 5 → GPIO12 (active low, internal pull-up)
Button 6 → GPIO13 (active low, internal pull-up)
Button 7 → GPIO14 (active low, internal pull-up)
```

**IMPORTANT**: Do NOT connect 5V to buttons. The ESP32-S2 internal pull-ups provide 3.3V.

### Wiring Notes

- **CRITICAL**: Joystick MUST be powered from 3.3V (not 5V) - ESP32-S2 ADC inputs are not 5V tolerant!
- All buttons use internal pull-up resistors and are active-low (pressed = GND)
- Connect button switches between GPIO pin and GND only
- KY-040 module requires 3.3V power supply
- Use short, quality jumper wires for stable encoder operation
- Joystick uses 13-bit ADC resolution (0-8191 range) with 12dB attenuation for 0-2.5V input range
- 5% dead-zone is applied to joystick axes to prevent drift

### GPIO Pin Safety

**Safe GPIOs used in this project:**
- GPIO2, GPIO3, GPIO4, GPIO5, GPIO6, GPIO7, GPIO8, GPIO9, GPIO10, GPIO11, GPIO12, GPIO13, GPIO14

**Avoid these GPIOs:**
- GPIO0 (BOOT mode pin - can cause resets)
- GPIO19, GPIO20 (USB D-, D+)
- GPIO43, GPIO44 (UART0 TX/RX)
- GPIO45, GPIO46 (System pins)

Picture below shows the prototype with only two buttons to test. The code supports seven in total.
![PXL_20251015_020349234](https://github.com/user-attachments/assets/71d8b990-86b1-4c38-b076-bf7b09293d62)

## Software Setup

### ESP-IDF Development

1. **Install ESP-IDF** (if not already installed):
```bash
   # Follow official ESP-IDF installation guide
   https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/get-started/
```

2. **Configure the project** (optional):
```bash
   idf.py menuconfig
   # Navigate to: Component config → TinyUSB Stack
   # Ensure HID Device is enabled
```

3. **Build and Flash**:
```bash
   idf.py build
   idf.py -p /dev/tty.usbserial-2120 -b 115200 flash
```
**Note**: On macOS, use the actual serial port (e.g., `/dev/tty.usbserial-XXXX`). On Linux, typically `/dev/ttyUSB0` or `/dev/ttyACM0`.

4. **Entering Download Mode (ESP32-S2 specific)**:
   - If flashing fails with "Wrong boot mode detected":
     1. Hold down the BOOT button
     2. Press and release the RESET button
     3. Release the BOOT button
     4. Immediately run the flash command

### Optional: Add idf.py Alias

Add to your `~/.zshrc` or `~/.bashrc`:
```bash
# ESP-IDF shortcuts
alias get_idf='. $HOME/esp/esp-idf/export.sh'
alias idf='idf.py'

# Auto-load ESP-IDF environment
get_idf
```

Then reload: `source ~/.zshrc`

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

## Testing on Host System

### Testing

1. **Web-based Gamepad Tester**:
   - Visit https://gamepad-tester.com or https://html5gamepad.com
   - All 7 buttons should register
   - Joystick X/Y axes should respond
   - Rotating encoder should change Z-axis

3. **Steam**:
   - Steam Settings
   - Controller
   - Look for "TinyUSB Fishing Game Controller" at the top
   - Setup the controller via Steam to test all buttons

### Button Mapping

| Physical Input | HID Button | Bit Position |
|---------------|------------|--------------|
| Button 1 (GPIO8) | Button 1 | 0 |
| Button 2 (GPIO2) | Button 2 | 1 |
| Button 3 (GPIO10) | Button 3 | 2 |
| Button 4 (GPIO11) | Button 4 | 3 |
| Button 5 (GPIO12) | Button 5 | 4 |
| Button 6 (GPIO13) | Button 6 | 5 |
| Button 7 (GPIO14) | Button 7 | 6 |
| Encoder Switch (GPIO7) | Button 8 | 7 |
| Joystick Switch (GPIO9) | Button 9 | 8 |

### Axis Mapping

| Physical Input | HID Axis | Range |
|---------------|----------|-------|
| Joystick X | X-axis | -127 to +127 |
| Joystick Y | Y-axis | -127 to +127 |
| Encoder Rotation | Z-axis | -127 to +127 |

## Troubleshooting

### Brownout Detection / Controller Resets When Pressing Buttons

**Symptoms**: Controller disconnects, power LED turns off when pressing buttons

**Causes**:
- Insufficient USB power supply
- Poor quality USB cable
- Using GPIO0 (BOOT pin) for buttons

**Solutions**:
1. Use a powered USB hub
2. Try a different, high-quality USB cable
3. Use a USB 3.0 port (higher current capacity)
4. Avoid GPIO0 for buttons (use GPIO1-18 instead)
5. Check button wiring - ensure no 5V connections

### Buttons Not Responding

1. Check wiring: Button should connect GPIO to GND only
2. Verify no 5V/3.3V is connected to button pins
3. Check serial monitor for debug output
4. Test with multimeter: GPIO should read ~3.3V when not pressed, 0V when pressed

### Joystick Drift

1. Increase JOYSTICK_DEADZONE value in code (currently 410)
2. Ensure joystick is powered from 3.3V (not 5V)
3. Calibrate by noting center values in serial monitor

### Encoder Not Responding

1. Check CLK and DT connections
2. Verify 3.3V power to encoder module
3. Try swapping CLK and DT if rotation direction is wrong
4. Check serial monitor for encoder position changes

## Example Output

After flashing you should see the output at idf monitor:
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
I (434) usb_init: Joystick ADC initialized (13-bit, 12dB attenuation)
I (434) usb_init: Starting main loop - Gamepad with Joystick + Encoder ready
```

## Project Structure
```
esp32-fishing-controller/
├── main/
│   └── main.c                     # Main controller code
├── managed_components/            # TinyUSB components
├── CMakeLists.txt
├── sdkconfig                      # ESP-IDF configuration
├── idf_component.yml              # Component dependencies
├── pyproject.toml                 # Python testing dependencies
└── README.md                      # This file
```

## Future Enhancements

- [ ] Add haptic feedback (vibration motor)
- [ ] Add LED status indicators
- [ ] 3D-printed enclosure
- [ ] Battery + Bluetooth support for wireless operation

## License

MIT License - Feel free to modify and use for your own projects!

## Credits

Created by Christopher Milian (2025)  
Contact: christophermilian16@gmail.com

Built with ESP-IDF and TinyUSB.
