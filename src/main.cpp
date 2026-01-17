/*
 * ESP32 Ride Height Gauge
 * 
 * Reads AS5600 magnetic angle sensor and displays calculated ride height
 * on SSD1306 OLED display (0.91" 128x32)
 * 
 * Hardware:
 * - ESP32 DevKit (WROOM)
 * - AS5600 I2C angle sensor (0x36)
 * - SSD1306 OLED 0.91" 128x32 (0x3C)
 * - Button 1: ZERO (active-low, GPIO 0)
 * - Button 2: UNIT toggle (active-low, GPIO 4) - optional
 * 
 * Wiring:
 * - AS5600: VCC -> 3.3V, GND -> GND, SDA -> GPIO 21, SCL -> GPIO 22
 * - OLED: VCC -> 3.3V, GND -> GND, SDA -> GPIO 21, SCL -> GPIO 22
 * - Button 1: One side to GPIO 0, other side to GND (internal pullup enabled)
 * - Button 2: One side to GPIO 4, other side to GND (internal pullup enabled)
 */

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

// I2C Configuration
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define AS5600_ADDRESS 0x36
#define OLED_ADDRESS 0x3C  // Change to 0x3D if your OLED uses that address

// Button Pins (active-low with internal pullup)
#define BUTTON_ZERO_PIN 0
#define BUTTON_UNIT_PIN 4  // Set to -1 to disable unit toggle button

// Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1  // Reset pin not used on I2C OLEDs

// Calibration Constants
#define MM_PER_COUNT 0.01f  // Adjust this based on your mechanism
                            // Example: 0.01 means 1 count = 0.01mm
                            // To calibrate: measure actual height change vs counts change
                            // MM_PER_COUNT = (height_change_mm) / (counts_change)

#define INVERT_DIRECTION false  // Set to true if height decreases when angle increases

// AS5600 Register Addresses
#define AS5600_REG_RAW_ANGLE_H 0x0C  // High byte of raw angle (12-bit)
#define AS5600_REG_RAW_ANGLE_L 0x0D  // Low byte of raw angle

// Display Update
#define DISPLAY_UPDATE_RATE_MS 50  // ~20 Hz
#define DISPLAY_THRESHOLD 0.01f    // Only update display if change > threshold

// Button Debouncing
#define DEBOUNCE_MS 50

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ============================================================================
// GLOBAL STATE
// ============================================================================

int16_t zeroOffsetCounts = 0;  // Calibrated zero position
bool useMetric = true;          // true = mm, false = inches
bool oledPresent = false;

// Button state
unsigned long lastButtonZeroTime = 0;
unsigned long lastButtonUnitTime = 0;
bool lastButtonZeroState = HIGH;
bool lastButtonUnitState = HIGH;

// Display update
unsigned long lastDisplayUpdate = 0;
float lastDisplayedHeight = -999.0f;

// ============================================================================
// AS5600 FUNCTIONS
// ============================================================================

/**
 * Read raw angle from AS5600 (0-4095)
 * Returns -1 on error
 */
int16_t readAngleRaw() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(AS5600_REG_RAW_ANGLE_H);
  if (Wire.endTransmission() != 0) {
    return -1;  // I2C error
  }
  
  if (Wire.requestFrom(AS5600_ADDRESS, 2) != 2) {
    return -1;  // Failed to read 2 bytes
  }
  
  uint8_t highByte = Wire.read();
  uint8_t lowByte = Wire.read();
  
  // AS5600 returns 12-bit value in high byte (bits 11-4) and low byte (bits 3-0)
  // High byte contains upper 8 bits, low byte contains lower 4 bits
  uint16_t rawAngle = ((uint16_t)highByte << 8) | lowByte;
  rawAngle = rawAngle & 0x0FFF;  // Mask to 12 bits (0-4095)
  
  return (int16_t)rawAngle;
}

/**
 * Calculate height in mm from angle counts
 * Handles wrap-around at 0/4095 boundary
 */
float calculateHeightMM(int16_t currentCounts) {
  // Calculate delta from zero position
  int16_t delta = currentCounts - zeroOffsetCounts;
  
  // Handle wrap-around (assuming rotation < 180°, so max delta is ~2048)
  // If delta is very large, we likely wrapped around
  if (delta > 2048) {
    delta = delta - 4096;  // Wrap backwards
  } else if (delta < -2048) {
    delta = delta + 4096;  // Wrap forwards
  }
  
  // Apply direction inversion if needed
  if (INVERT_DIRECTION) {
    delta = -delta;
  }
  
  // Convert to height
  float height = (float)delta * MM_PER_COUNT;
  
  return height;
}

// ============================================================================
// DISPLAY FUNCTIONS
// ============================================================================

void initDisplay() {
  oledPresent = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  
  if (oledPresent) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Ride Height");
    display.println("Initializing...");
    display.display();
    delay(500);
  } else {
    Serial.println("OLED not found! Continuing with Serial output only.");
  }
}

void updateDisplay(float heightMM) {
  if (!oledPresent) {
    return;
  }
  
  // Only update if change is significant (reduce flicker)
  if (abs(heightMM - lastDisplayedHeight) < DISPLAY_THRESHOLD && 
      lastDisplayedHeight != -999.0f) {
    return;
  }
  
  lastDisplayedHeight = heightMM;
  
  display.clearDisplay();
  
  // Convert to display unit
  float displayValue = useMetric ? heightMM : (heightMM / 25.4f);
  const char* unit = useMetric ? "mm" : "in";
  
  // Display main height value (large)
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(displayValue, 2);
  display.print(" ");
  display.setTextSize(1);
  display.print(unit);
  
  // Display debug info (small, bottom line)
  int16_t rawCounts = readAngleRaw();
  display.setTextSize(1);
  display.setCursor(0, 24);
  if (rawCounts >= 0) {
    display.print("Raw: ");
    display.print(rawCounts);
    display.print(" | Zero: ");
    display.print(zeroOffsetCounts);
  } else {
    display.print("SENSOR ERR");
  }
  
  display.display();
}

// ============================================================================
// BUTTON HANDLING
// ============================================================================

void handleButtons() {
  unsigned long now = millis();
  
  // ZERO Button
  bool currentZeroState = digitalRead(BUTTON_ZERO_PIN);
  if (currentZeroState == LOW && lastButtonZeroState == HIGH) {
    // Button just pressed
    if (now - lastButtonZeroTime > DEBOUNCE_MS) {
      int16_t currentCounts = readAngleRaw();
      if (currentCounts >= 0) {
        zeroOffsetCounts = currentCounts;
        Serial.print("Zero set to: ");
        Serial.println(zeroOffsetCounts);
        
        // Update display immediately
        float height = calculateHeightMM(currentCounts);
        updateDisplay(height);
      }
      lastButtonZeroTime = now;
    }
  }
  lastButtonZeroState = currentZeroState;
  
  // UNIT Button (if enabled)
  if (BUTTON_UNIT_PIN >= 0) {
    bool currentUnitState = digitalRead(BUTTON_UNIT_PIN);
    if (currentUnitState == LOW && lastButtonUnitState == HIGH) {
      // Button just pressed
      if (now - lastButtonUnitTime > DEBOUNCE_MS) {
        useMetric = !useMetric;
        Serial.print("Unit changed to: ");
        Serial.println(useMetric ? "mm" : "in");
        
        // Force display update
        lastDisplayedHeight = -999.0f;
        lastButtonUnitTime = now;
      }
    }
    lastButtonUnitState = currentUnitState;
  }
}

// ============================================================================
// I2C SCAN
// ============================================================================

void scanI2C() {
  Serial.println("Scanning I2C bus...");
  byte found = 0;
  
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
      
      if (address == AS5600_ADDRESS) {
        Serial.println("  -> AS5600 detected!");
      } else if (address == OLED_ADDRESS || address == 0x3D) {
        Serial.println("  -> OLED detected!");
      }
      
      found++;
    }
  }
  
  if (found == 0) {
    Serial.println("No I2C devices found!");
  } else {
    Serial.print("Found ");
    Serial.print(found);
    Serial.println(" device(s).");
  }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== ESP32 Ride Height Gauge ===");
  
  // Initialize I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);  // 400kHz I2C speed
  
  // Scan I2C bus
  scanI2C();
  
  // Initialize display
  initDisplay();
  
  // Initialize buttons
  pinMode(BUTTON_ZERO_PIN, INPUT_PULLUP);
  if (BUTTON_UNIT_PIN >= 0) {
    pinMode(BUTTON_UNIT_PIN, INPUT_PULLUP);
  }
  
  // Read initial angle to set zero (optional - user can press button instead)
  int16_t initialCounts = readAngleRaw();
  if (initialCounts >= 0) {
    zeroOffsetCounts = initialCounts;
    Serial.print("Initial zero set to: ");
    Serial.println(zeroOffsetCounts);
  } else {
    Serial.println("WARNING: Could not read AS5600 on startup!");
  }
  
  Serial.println("Setup complete. Press ZERO button to calibrate.");
  Serial.println("Press UNIT button to toggle mm/inch.");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long now = millis();
  
  // Handle button presses
  handleButtons();
  
  // Update display at regular interval
  if (now - lastDisplayUpdate >= DISPLAY_UPDATE_RATE_MS) {
    int16_t currentCounts = readAngleRaw();
    
    if (currentCounts >= 0) {
      float heightMM = calculateHeightMM(currentCounts);
      updateDisplay(heightMM);
      
      // Also print to Serial for debugging
      Serial.print("Counts: ");
      Serial.print(currentCounts);
      Serial.print(" | Height: ");
      Serial.print(heightMM, 2);
      Serial.print(" mm (");
      Serial.print(heightMM / 25.4f, 3);
      Serial.println(" in)");
    } else {
      // Sensor error
      if (oledPresent) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 10);
        display.println("SENSOR ERR");
        display.display();
      }
      Serial.println("ERROR: Failed to read AS5600!");
    }
    
    lastDisplayUpdate = now;
  }
  
  delay(10);  // Small delay to prevent watchdog issues
}
