# ESP32 Ride Height Gauge

A DIY RC ride-height gauge using an ESP32, AS5600 magnetic angle sensor, and SSD1306 OLED display.

## Hardware Requirements

- **ESP32 DevKit (WROOM)** - Any ESP32 development board
- **AS5600 I2C Magnetic Angle Sensor** - Breakout board with I2C interface
- **SSD1306 OLED Display** - 0.91" 128x32 I2C OLED (typically address 0x3C)
- **Push Buttons** - 2x tactile switches (one for ZERO, optional one for UNIT toggle)
- **Jumper wires** for connections

## Wiring Diagram

### I2C Bus (Shared between AS5600 and OLED)
- **SDA** → GPIO 21 (ESP32)
- **SCL** → GPIO 22 (ESP32)
- **VCC** → 3.3V (ESP32)
- **GND** → GND (ESP32)

### AS5600 Connections
- VCC → 3.3V
- GND → GND
- SDA → GPIO 21
- SCL → GPIO 22

### OLED Connections
- VCC → 3.3V
- GND → GND
- SDA → GPIO 21
- SCL → GPIO 22

### Button Connections
- **ZERO Button**: One terminal to GPIO 0, other terminal to GND (active-low with internal pullup)
- **UNIT Button** (optional): One terminal to GPIO 4, other terminal to GND (active-low with internal pullup)

**Note**: Buttons are active-low, meaning they connect GPIO to GND when pressed. The ESP32's internal pullup resistors keep the pin HIGH when not pressed.

## Pin Configuration

Default pins (can be changed in `src/main.cpp`):
- **I2C SDA**: GPIO 21
- **I2C SCL**: GPIO 22
- **ZERO Button**: GPIO 0
- **UNIT Button**: GPIO 4 (set to -1 in code to disable)

## Calibration Guide

### 1. Setting MM_PER_COUNT

The `MM_PER_COUNT` constant determines how many millimeters of height change correspond to one count (0-4095) of the AS5600 sensor.

**To calibrate:**

1. Mount the sensor in your mechanism
2. Move the mechanism to a known position (e.g., 0mm height)
3. Press the ZERO button to set the reference
4. Move to another known position (e.g., 10mm height)
5. Note the raw counts displayed on the OLED (or Serial monitor)
6. Calculate: `MM_PER_COUNT = (height_change_mm) / (counts_change)`

**Example:**
- Start at 0mm, zero set → counts = 2000
- Move to 10mm → counts = 3000
- Counts change = 3000 - 2000 = 1000
- `MM_PER_COUNT = 10.0 / 1000 = 0.01`

Update the `MM_PER_COUNT` constant in `src/main.cpp`:
```cpp
#define MM_PER_COUNT 0.01f  // Your calculated value
```

### 2. Inverting Direction

If your height decreases when the angle increases (or vice versa), set:
```cpp
#define INVERT_DIRECTION true
```

If the direction is correct, leave it as:
```cpp
#define INVERT_DIRECTION false
```

### 3. Zero Calibration

1. Position your mechanism at the desired "zero" height
2. Press and release the ZERO button
3. The current angle will be stored as the zero reference
4. The display will now show relative height from this zero point

## Usage

1. **Power on**: The device will scan I2C devices and initialize
2. **Set zero**: Press ZERO button when mechanism is at reference position
3. **Read height**: Display shows current height in mm or inches
4. **Toggle units**: Press UNIT button to switch between mm and inches

## Building and Uploading

### Method 1: Using PlatformIO (Recommended)

**Step 1: Install PlatformIO**
- Install [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) for VS Code, or
- Install PlatformIO Core via command line: `pip install platformio`

**Step 2: Open the Project**
- If using VS Code: File → Open Folder → Select the `RideHeight` folder
- PlatformIO will automatically detect the `platformio.ini` file

**Step 3: Connect ESP32**
- Connect your ESP32 to your computer via USB cable
- Wait for the computer to recognize the device

**Step 4: Upload the Program**
- **In VS Code**: Click the "Upload" button (→) in the PlatformIO toolbar, or
- **Command line**: Run:
  ```bash
  cd /Users/panic/Documents/GitHub/RideHeight
  pio run --target upload
  ```

**Step 5: Monitor Serial Output**
- **In VS Code**: Click the "Serial Monitor" button (plug icon) in PlatformIO toolbar, or
- **Command line**: Run:
  ```bash
  pio device monitor
  ```
- Set baud rate to 115200 if prompted

**Troubleshooting Upload:**
- If upload fails, you may need to put ESP32 in bootloader mode:
  - Hold the BOOT button on ESP32
  - Press and release RESET button
  - Release BOOT button
  - Try upload again
- Check that correct USB port is selected (Tools → Port in VS Code)

### Method 2: Using Arduino IDE

**Step 1: Install ESP32 Board Support**
1. Open Arduino IDE
2. Go to File → Preferences
3. Add this URL to "Additional Board Manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Go to Tools → Board → Boards Manager
5. Search for "ESP32" and install "esp32 by Espressif Systems"

**Step 2: Install Required Libraries**
1. Go to Sketch → Include Library → Manage Libraries
2. Search for and install:
   - **Adafruit SSD1306** (by Adafruit)
   - **Adafruit GFX Library** (by Adafruit)
   - **Adafruit BusIO** (by Adafruit)

**Step 3: Prepare the Sketch**
1. Create a new sketch in Arduino IDE
2. Copy the entire contents of `src/main.cpp`
3. Paste into the new sketch
4. Save the sketch (Arduino IDE will create a `.ino` file)

**Step 4: Configure Board Settings**
1. Go to Tools → Board → ESP32 Arduino → ESP32 Dev Module
2. Select the correct Port: Tools → Port → (your ESP32's COM port)
3. Set Upload Speed: Tools → Upload Speed → 115200

**Step 5: Upload**
1. Click the Upload button (→) in Arduino IDE
2. Wait for compilation and upload to complete
3. Open Serial Monitor (Tools → Serial Monitor)
4. Set baud rate to 115200

**Note:** If upload fails, you may need to hold the BOOT button while clicking Upload, then release BOOT when upload starts.

## Troubleshooting

### OLED Not Displaying
- Check I2C address (try 0x3D if 0x3C doesn't work)
- Verify wiring (SDA/SCL, VCC/GND)
- Check Serial monitor for I2C scan results

### Sensor Error
- Verify AS5600 is connected correctly
- Check I2C address (should be 0x36)
- Ensure magnet is properly positioned over sensor
- Check Serial monitor for I2C errors

### Buttons Not Working
- Verify button wiring (one side to GPIO, other to GND)
- Check if GPIO pins are correct in code
- Buttons are active-low (LOW when pressed)

### Height Reading Incorrect
- Recalibrate `MM_PER_COUNT` value
- Check if `INVERT_DIRECTION` needs to be changed
- Ensure zero is set correctly at reference position

## Serial Output

The device outputs debug information to Serial at 115200 baud:
- I2C device scan on startup
- Current counts and height readings
- Zero calibration confirmations
- Unit change confirmations

## Features

- ✅ Direct AS5600 register reading (no external library dependency)
- ✅ Large, readable height display with 2 decimal places
- ✅ Zero calibration via button
- ✅ Unit toggle (mm/inches)
- ✅ Wrap-around handling for angle transitions
- ✅ Error handling for missing sensors
- ✅ Configurable calibration constants
- ✅ Button debouncing
- ✅ Display update optimization (reduces flicker)

## License

This project is provided as-is for DIY use.
