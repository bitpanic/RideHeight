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
#define AS5600_REG_ZMCO 0x00         // ZMCO register (read-only, can test communication)
#define AS5600_REG_STATUS 0x0B       // Status register (check magnet presence)
#define AS5600_REG_RAW_ANGLE_H 0x0C  // High byte of raw angle (12-bit)
#define AS5600_REG_RAW_ANGLE_L 0x0D  // Low byte of raw angle

// Display Update
#define DISPLAY_UPDATE_RATE_MS 50  // ~20 Hz
#define DISPLAY_THRESHOLD 0.01f    // Only update display if change > threshold
#define DISPLAY_FORCE_UPDATE_MS 500  // Force update every 500ms even if no change

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
unsigned long lastForceDisplayUpdate = 0;
float lastDisplayedHeight = -999.0f;

// ============================================================================
// AS5600 FUNCTIONS
// ============================================================================

/**
 * Read raw angle from AS5600 (0-4095)
 * Returns -1 on error
 */
int16_t readAngleRaw() {
  // Add delay before I2C operations to ensure sensor is ready
  delayMicroseconds(1000);  // Increased delay for stability
  
  // Try reading RAW_ANGLE register (0x0C) - standard I2C read sequence
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(AS5600_REG_RAW_ANGLE_H);  // Set register pointer to 0x0C
  uint8_t error = Wire.endTransmission();  // Send stop bit
  
  if (error != 0) {
    static unsigned long lastErrorPrint = 0;
    unsigned long now = millis();
    if (now - lastErrorPrint > 2000) {
      Serial.print("[DEBUG] readAngleRaw() - Register 0x0C write error: ");
      Serial.print(error);
      Serial.print(" at address 0x");
      Serial.println(AS5600_ADDRESS, HEX);
      Serial.println("[DEBUG] AS5600 detected but register write failing. Check magnet distance/position.");
      lastErrorPrint = now;
    }
    return -1;  // I2C error
  }
  
  // Small delay before read
  delayMicroseconds(100);
  
  // Request 2 bytes from sensor
  uint8_t bytesReceived = Wire.requestFrom((uint8_t)AS5600_ADDRESS, (uint8_t)2);
  if (bytesReceived != 2) {
    static unsigned long lastErrorPrint = 0;
    unsigned long now = millis();
    if (now - lastErrorPrint > 2000) {
      Serial.print("[DEBUG] readAngleRaw() - Failed to receive 2 bytes. Received: ");
      Serial.print(bytesReceived);
      Serial.println(" bytes");
      lastErrorPrint = now;
    }
    return -1;  // Failed to read 2 bytes
  }
  
  uint8_t highByte = Wire.read();
  uint8_t lowByte = Wire.read();
  
  // AS5600 RAW_ANGLE register (0x0C/0x0D) contains 12-bit value
  // High byte: bits 11-4 (in lower 8 bits of high byte)
  // Low byte: bits 3-0 (in lower 4 bits of low byte), bits 7-4 are 0
  // Combined: bits [11:0] contain the 12-bit angle value
  uint16_t rawAngle = ((uint16_t)highByte << 8) | lowByte;
  rawAngle = rawAngle & 0x0FFF;  // Mask to 12 bits (0-4095)
  
  // Log successful reads occasionally
  static unsigned long lastSuccessPrint = 0;
  unsigned long now = millis();
  if (now - lastSuccessPrint > 5000) {  // Print success every 5 seconds
    Serial.print("[DEBUG] readAngleRaw() - SUCCESS! Read angle: ");
    Serial.println(rawAngle);
    lastSuccessPrint = now;
  }
  
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
  Serial.println("\n[DEBUG] Starting OLED initialization...");
  Serial.print("[DEBUG] Attempting to initialize OLED at address 0x");
  Serial.println(OLED_ADDRESS, HEX);
  
  oledPresent = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  
  Serial.print("[DEBUG] display.begin() returned: ");
  Serial.println(oledPresent ? "TRUE (success)" : "FALSE (failed)");
  
  if (oledPresent) {
    Serial.println("[DEBUG] OLED initialized, clearing and drawing init screen...");
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Ride Height");
    display.println("Initializing...");
    display.display();
    Serial.println("[DEBUG] Init screen displayed, waiting 500ms...");
    delay(500);
    Serial.println("[DEBUG] OLED initialization complete.");
  } else {
    Serial.println("[DEBUG] ERROR: OLED not found! Continuing with Serial output only.");
  }
}

void updateDisplay(float heightMM, bool forceUpdate = false) {
  if (!oledPresent) {
    return;
  }
  
  // Only update if change is significant (reduce flicker) or if forced
  float heightDiff = abs(heightMM - lastDisplayedHeight);
  if (!forceUpdate && heightDiff < DISPLAY_THRESHOLD && 
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
  Wire.setClock(50000);  // 50kHz I2C speed (very slow for maximum reliability)
  
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
    
    // Update display with initial reading (after a small delay to ensure display is ready)
    if (oledPresent) {
      Serial.println("[DEBUG] OLED is present, preparing initial display update...");
      delay(100);  // Give display time to be ready
      float initialHeight = calculateHeightMM(initialCounts);
      Serial.print("[DEBUG] Calculated initial height: ");
      Serial.println(initialHeight, 3);
      lastDisplayedHeight = -999.0f;  // Force display update
      Serial.println("[DEBUG] Calling updateDisplay() with initial height...");
      updateDisplay(initialHeight, true);  // Force initial update
      lastForceDisplayUpdate = millis();  // Initialize force update timer
      Serial.println("[DEBUG] Initial display update complete.");
    } else {
      Serial.println("[DEBUG] OLED not present, skipping initial display update.");
    }
  } else {
    Serial.println("WARNING: Could not read AS5600 on startup!");
  }
  
  lastDisplayUpdate = millis();  // Initialize display update timer
  
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
  unsigned long timeSinceLastUpdate = now - lastDisplayUpdate;
  if (timeSinceLastUpdate >= DISPLAY_UPDATE_RATE_MS) {
    int16_t currentCounts = readAngleRaw();
    
    if (currentCounts >= 0) {
      float heightMM = calculateHeightMM(currentCounts);
      
      // Force update periodically even if value hasn't changed much
      unsigned long timeSinceForceUpdate = now - lastForceDisplayUpdate;
      bool forceUpdate = (timeSinceForceUpdate >= DISPLAY_FORCE_UPDATE_MS);
      if (forceUpdate) {
        lastForceDisplayUpdate = now;
      }
      
      updateDisplay(heightMM, forceUpdate);
      
      // Also print to Serial for debugging
      Serial.print("Counts: ");
      Serial.print(currentCounts);
      Serial.print(" | Height: ");
      Serial.print(heightMM, 2);
      Serial.print(" mm (");
      Serial.print(heightMM / 25.4f, 3);
      Serial.println(" in)");
    } else {
      // Sensor error - only print occasionally to avoid spam
      static unsigned long lastErrorPrint = 0;
      if (now - lastErrorPrint > 1000) {  // Print error max once per second
        Serial.println("[DEBUG] ERROR: Failed to read AS5600 in main loop!");
        lastErrorPrint = now;
      }
      
      if (oledPresent) {
        static unsigned long lastErrorDisplay = 0;
        // Only update error display occasionally to reduce flicker
        if (now - lastErrorDisplay > 500) {
          display.clearDisplay();
          display.setTextSize(1);
          display.setCursor(0, 10);
          display.println("SENSOR ERR");
          display.print("Magnet needed");
          display.display();
          lastErrorDisplay = now;
        }
      }
    }
    
    lastDisplayUpdate = now;
  }
  
  delay(10);  // Small delay to prevent watchdog issues
}
