#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <driver/ledc.h>

// Uncomment the line below to enable debugging
#define DEBUG

// Debug Macros
#ifdef DEBUG
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

// Pin Definitions
const uint8_t ledSwitch = T7;  // Capacitive touch input for on/off
const uint8_t switchUp = T8;   // Capacitive touch input for increasing brightness
const uint8_t switchDn = T9;   // Capacitive touch input for decreasing brightness
const uint8_t tempUp = T10;    // Capacitive touch input for increasing cool color
const uint8_t tempDn = T11;    // Capacitive touch input for increasing warm color
const uint8_t dimOut1 = 36;    // Cool white channel (PWM pin)
const uint8_t dimOut2 = 35;    // Warm white channel (PWM pin)
const uint8_t ws2812Pin = 40;  // WS2812 data pin

bool ignoreRepeats = true;
uint32_t startTime = 0;

// Use / disable the addressable LED at runtime
bool ws2812Enabled = false;

// Add state variables for press-and-hold and double-tap detection
bool lastSwitchUpState = false;
bool lastSwitchDnState = false;
bool lastTempUpState = false;
bool lastTempDnState = false;

unsigned long lastTapTimeUp = 0;
unsigned long lastTapTimeDown = 0;
unsigned long lastTapTimeTempUp = 0;
unsigned long lastTapTimeTempDown = 0;

const unsigned long doubleTapInterval = 300;  // Max time for a double-tap in milliseconds
const unsigned long dimholdDelay = 8;        // Time to wait before considering a hold
const unsigned long tempholdDelay = 5;

// Quad-tap tracking for the on/off touch key
uint8_t ledTapCount = 0;
unsigned long ledLastTapTime = 0;
const unsigned long multiTapInterval = 400; // ms gap allowed between taps

// Long-hold detection for ON/OFF pad (only when turning ON)
unsigned long ledHoldStart     = 0;
bool          ledHoldArmed     = false;
bool          wsHoldFired      = false;
const unsigned long ws2812ToggleHoldMs = 3000; // 3 seconds

unsigned long switchUpHoldStart = 0;
unsigned long switchDnHoldStart = 0;
unsigned long tempUpHoldStart = 0;
unsigned long tempDnHoldStart = 0;

unsigned long tempBothHoldStart = 0;
bool tempBothHoldArmed  = false;   // tracking while both are held
bool tempBothHoldFired  = false;   // prevent repeat firing until released
const unsigned long tempBothResetHoldMs = 2000;  // 2 seconds

// WS2812 LED Configuration
const uint16_t numPixels = 1;  // Number of WS2812 LEDs
Adafruit_NeoPixel ws2812 = Adafruit_NeoPixel(numPixels, ws2812Pin, NEO_GRB + NEO_KHZ800);

// Constants and Variables
const float gammaValue = 2.2;
const int step = 2;

uint16_t brightness = 512;  // Brightness (0–1023). Start default at 50%
uint16_t minBrightness = 200; // 10%
uint16_t lastBrightness = 512;   // To track previous brightness level

const unsigned long brightnessUpdateInterval = 2; // Fast update interval for smooth changes


float tempRatio = 0.5;
float tempRatioStep = 0.0025;
unsigned long lastTempChange = 0;
const unsigned long tempDelay = 10;
unsigned long lastBrightnessChange = 0;
const unsigned long brightnessDelay = 10;
unsigned long lastColorChange = 0;
const unsigned long colorDelay = 500;  // Time between WS2812 color changes
uint8_t colorIndex = 0;
bool ledState = false;            // LED on/off state
bool lastLedSwitchState = false;  // on/off switch state
bool ledSwitchState = 0;
bool switchUpState = 0;
bool switchDnState = 0;
bool tempUpState = 0;
bool tempDnState = 0;

// Filtered touch values
float filteredLedSwitch = 0;
float filteredTouchUp = 0;
float filteredTouchDown = 0;
float filteredTouchTempUp = 0;
float filteredTouchTempDown = 0;

// Constants for restart check
const int maxRepeatedReads = 10;      // Number of consecutive same values before restarting
uint16_t previousLedSwitchValue = 0;  // To store the previous touchRead value
int repeatedCount = 0;                // Counter for consecutive repeated values

// PWM Configuration
const uint32_t pwmFrequency = 20000;
const uint8_t pwmResolution = 10;
const uint32_t pwmMaxDuty = (1 << pwmResolution) - 1;

const float    maxBrightnessPct = 0.95f;  // cap at 95% post-gamma
const uint16_t maxBrightness    = (uint16_t)round(pwmMaxDuty * maxBrightnessPct);  // raw 10-bit limit (~972)

// LEDC Channels
const ledc_channel_t coolChannel = LEDC_CHANNEL_0;
const ledc_channel_t warmChannel = LEDC_CHANNEL_1;


// Define the pin for the ALS-PT19-315C sensor
const int lightSensorPin = 5; // GPIO5 for the light sensor
const float resistorValue = 10000.0; // 10 kΩ resistor in the voltage divider
//const unsigned long lightReadInterval = 10000; // 10 seconds interval
//unsigned long lastLightReadTime = 0;


void readAmbientLight() {
  // Read the ADC value
  int adcValue = analogRead(lightSensorPin);

  // Calculate voltage (assuming 12-bit ADC and 3.3V reference voltage)
  float voltage = (adcValue / 4095.0) * 3.3;

  // Calculate photocurrent (Iph in microamps)
  float Iph = (voltage / resistorValue) * 1e6; // Convert to microamps

  // Calculate illuminance (lux) using typical sensitivity of 100 µA at 1000 lux
  float illuminance = (Iph / 100.0) * 1000.0;

  // Print the results
  DEBUG_PRINT("[Light Sensor] Raw ADC Value: ");
  DEBUG_PRINTLN(adcValue);
  DEBUG_PRINT("[Light Sensor] Voltage: ");
  DEBUG_PRINTLN(voltage, 3);
  Serial.print("[Light Sensor] Photocurrent (µA): ");
  Serial.println(Iph, 2);
  Serial.print("[Light Sensor] Illuminance (lux): ");
  Serial.println(illuminance, 2);
}


// Constants for NTC and ADC
const int adcPin = 16;                  // GPIO16 for ADC
const float adcMaxValue = 4095.0;       // 12-bit ADC resolution
const float vRef = 3.3;                 // Reference voltage
const float seriesResistor = 10000;     // 10kΩ series resistor
const float nominalResistance = 10000;  // 10kΩ at 25°C
const float nominalTemperature = 25.0;  // Nominal temperature in °C
const float bCoefficient = 3434;        // B-coefficient of the thermistor

unsigned long lastSensorReadTime = 0;            // Tracks the last time temperature was read
const unsigned long sensorReadInterval = 10000;  // Interval to read temperature (10 seconds)





float readTemperature() {
  // Read the ADC value
  int adcValue = analogRead(adcPin);

  // Avoid divide by zero errors
  if (adcValue == 0 || adcValue == adcMaxValue) {
    DEBUG_PRINTLN("ADC value is invalid.");
    return NAN;  // Return 'Not a Number' to indicate error
  }

  // Calculate the voltage across the thermistor
  float voltage = (adcValue / adcMaxValue) * vRef;

  // Calculate the resistance of the thermistor
  float thermistorResistance = seriesResistor * (voltage / (vRef - voltage));

  // Ensure the resistance is valid
  if (thermistorResistance <= 0) {
    DEBUG_PRINTLN("Calculated resistance is invalid.");
    return NAN;
  }

  // Apply the Steinhart-Hart equation to calculate the temperature
  float steinhart;
  steinhart = thermistorResistance / nominalResistance;  // (R/Ro)
  steinhart = log(steinhart);                            // ln(R/Ro)
  steinhart /= bCoefficient;                             // 1/B * ln(R/Ro) using B=3434K
  steinhart += 1.0 / (nominalTemperature + 273.15);      // + (1/To)
  steinhart = 1.0 / steinhart;                           // Invert
  steinhart -= 273.15;                                   // Convert to °C


  DEBUG_PRINT("[Thermistor] ADC Value: ");
  DEBUG_PRINTLN(adcValue);
  DEBUG_PRINT("[Thermistor] Voltage: ");
  DEBUG_PRINTLN(voltage, 3);
  Serial.print("[Thermistor] Resistance: ");
  Serial.println(thermistorResistance, 2);
  Serial.print("[Thermistor] Temperature: ");
  Serial.println(steinhart, 2);

  return steinhart;
}


void setupPWM() {
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_10_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = pwmFrequency,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t cool_channel = {
    .gpio_num = dimOut1,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = coolChannel,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&cool_channel);

  ledc_channel_config_t warm_channel = {
    .gpio_num = dimOut2,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = warmChannel,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = pwmMaxDuty / 2
  };
  ledc_channel_config(&warm_channel);
}

float applyGammaCorrection(float brightnessNormalized) {
  // Gamma correction formula: Vout = Vin^(1/gamma)
  return pow(brightnessNormalized, gammaValue);
}


// Function to adjust and update brightness
void adjustBrightness(int stepChange) {
  // Adjust brightness and clamp to valid range
  uint16_t newBrightness = constrain(brightness + stepChange, minBrightness, maxBrightness);

  // Update only if the brightness has changed
  if (newBrightness != brightness) {
    brightness = newBrightness;
    updatePWM();
    DEBUG_PRINT("Brightness adjusted to: ");
    DEBUG_PRINTLN(brightness);
  }
}


void updatePWM() {
  static uint32_t lastColor = ws2812.Color(0, 0, 0);  // To store the last color state
  static uint8_t lastBrightness = 0;                  // To store the last brightness state

  if (!ledState) {
    // Turn off LED
    ledc_set_duty(LEDC_LOW_SPEED_MODE, coolChannel, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, warmChannel, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, coolChannel);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, warmChannel);

    if (ws2812Enabled) {
      ws2812.clear();  // Clear all pixels (set to black)
      ws2812.show();   // Update the WS2812 LED
    }

    return;
  }

  //if led is turned off and brightness is 0 then turn it back on at 50%
  //TODO: I dont think this will ever happen...
  if (ledState && brightness == 0) {
    DEBUG_PRINTLN("LED turned on at 0% brightness, resetting to 50%");
    brightness = 512;
  }

  // Normalize brightness (0-1), apply gamma correction, then scale back to 10-bit resolution
    // float normalizedBrightness = brightness / 1023.0f;
  // float gammaCorrected = applyGammaCorrection(normalizedBrightness);

  float normalizedBrightness = brightness / static_cast<float>(maxBrightness);
  if (normalizedBrightness > 1.0f) {
    normalizedBrightness = 1.0f;
  }

  float gammaCorrected = applyGammaCorrection(normalizedBrightness) * maxBrightnessPct;
  if (gammaCorrected > maxBrightnessPct) gammaCorrected = maxBrightnessPct;
  
  uint32_t totalDuty = (uint32_t)(gammaCorrected * pwmMaxDuty);
  uint32_t coolDuty = (uint32_t)(totalDuty * tempRatio);
  uint32_t warmDuty = totalDuty - coolDuty;

  ledc_set_duty(LEDC_LOW_SPEED_MODE, coolChannel, coolDuty);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, warmChannel, warmDuty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, coolChannel);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, warmChannel);


// WS2812 Update (brightness scaled to 255)
  uint8_t wsBrightness = (uint8_t)(gammaCorrected * 255.0f);
  uint8_t red = (uint8_t)((1.0f - tempRatio) * wsBrightness);
  uint8_t blue = (uint8_t)(tempRatio * wsBrightness);

  // Save the last state before updating
  lastColor = ws2812.Color(red, 0, blue);
  lastBrightness = wsBrightness;

  // Update WS2812 LED only if enabled
  // Only update if enabled
  if (ws2812Enabled) {
    ws2812.setPixelColor(0, lastColor);
    ws2812.show();
  }
}

void cycleColors() {
  if (!ws2812Enabled) return;
  // Define a simple color wheel (Red, Green, Blue)
  uint32_t colors[] = { ws2812.Color(255, 0, 0), ws2812.Color(0, 255, 0), ws2812.Color(0, 0, 255) };

  if (millis() - lastColorChange > colorDelay) {
    ws2812.setPixelColor(0, colors[colorIndex]);  // Set the current color
    ws2812.show();                                // Update the WS2812 LED
    colorIndex = (colorIndex + 1) % 3;            // Cycle to the next color
    lastColorChange = millis();
  }
}

// Define buffer size for moving average
const int bufferSize = 5;                       // Number of samples to average
float touchBuffers[5][bufferSize] = { { 0 } };  // Buffers for each touch input
int bufferIndex = 0;                            // Circular buffer index

// Filtered values
float filteredValues[5] = { 0 };

// Function to update moving average for a given input
void updateMovingAverage(float rawValues[], int numInputs) {
  // Update the circular buffer and calculate the moving average
  for (int i = 0; i < numInputs; i++) {
    touchBuffers[i][bufferIndex] = rawValues[i];  // Store the raw value in the buffer
    filteredValues[i] = 0;                        // Reset the filtered value

    // Calculate the moving average
    for (int j = 0; j < bufferSize; j++) {
      filteredValues[i] += touchBuffers[i][j];
    }
    filteredValues[i] /= bufferSize;
  }

  // Advance the circular buffer index
  bufferIndex = (bufferIndex + 1) % bufferSize;
}

// Function to check repeated values
void checkRepeatedValues(uint16_t currentValue) {
  if (currentValue == previousLedSwitchValue) {
    repeatedCount++;
  } else {
    repeatedCount = 0;  // Reset counter if the value changes
  }

  previousLedSwitchValue = currentValue;

  if (repeatedCount >= maxRepeatedReads) {
    DEBUG_PRINTLN("Repeated value detected 10 times. Restarting...");
    ESP.restart();  // Restart the ESP32
  }
}


// Calibration Constants
const int calibrationSamples = 100; // Number of samples for calibration
float baselineValues[5] = {0};  // Baseline values for touch sensors
float touchThresholds[5] = {0}; // Thresholds for touch detection


// Calibration Function
void calibrateTouchSensors() {
  Serial.println("Calibrating touch sensors...");

  //   //get out the bad initial values
  // for (int i = 0; i < 10; i++) {
  //   processTouchInputs();
  // }
  //   if (!ignoreRepeats) {
  //   checkRepeatedValues(rawTouchInputs[0]);
  // }

  for (int i = 0; i < 5; i++) baselineValues[i] = 0;



  for (int sample = 0; sample < calibrationSamples; sample++) {
    baselineValues[0] += touchRead(ledSwitch);
    // Serial.println(touchRead(ledSwitch));
    // Serial.println(baselineValues[0]);
    baselineValues[1] += touchRead(switchUp);
    baselineValues[2] += touchRead(switchDn);
    baselineValues[3] += touchRead(tempUp);
    baselineValues[4] += touchRead(tempDn);
    checkRepeatedValues(touchRead(ledSwitch));

    delay(10);
  }

  
  // Average the values and set thresholds
  for (int i = 0; i < 5; i++) {
    baselineValues[i] /= calibrationSamples;
    touchThresholds[i] = baselineValues[i] * 1.1; // add 10% threshold
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": Baseline = ");
    Serial.print(baselineValues[i]);
    Serial.print(", Threshold = ");
    Serial.println(touchThresholds[i]);
  }
  Serial.println("Calibration complete!");
}


void processTouchInputs() {
  // Read raw touch inputs
  float rawTouchInputs[] = {
    touchRead(ledSwitch),
    touchRead(switchUp),
    touchRead(switchDn),
    touchRead(tempUp),
    touchRead(tempDn)
  };

  // Update moving averages
  updateMovingAverage(rawTouchInputs, 5);

  // Map filtered values to variables
  filteredLedSwitch = filteredValues[0];
  filteredTouchUp = filteredValues[1];
  filteredTouchDown = filteredValues[2];
  filteredTouchTempUp = filteredValues[3];
  filteredTouchTempDown = filteredValues[4];

  
  
  // DEBUG_PRINT(rawTouchInputs[1]);
  // DEBUG_PRINTLN(" raw");
  // DEBUG_PRINT(filteredTouchUp);
  // DEBUG_PRINTLN(" filtered");
  // DEBUG_PRINT("threshold ");
  // DEBUG_PRINTLN(touchThresholds[1]);

  if (rawTouchInputs[0] > (touchThresholds[0]*1.0)) ledSwitchState = true;
  else ledSwitchState = (filteredLedSwitch > touchThresholds[0]);

  if (rawTouchInputs[1] > (touchThresholds[1]*1.0)) switchUpState = true;
  else switchUpState = (filteredTouchUp > touchThresholds[1]);

  if (rawTouchInputs[2] > (touchThresholds[2]*1.0)) switchDnState = true;
  else switchDnState = (filteredTouchDown > touchThresholds[2]);

  if (rawTouchInputs[3] > (touchThresholds[3]*1.0)) tempUpState = true;
  else tempUpState = (filteredTouchTempUp > touchThresholds[3]);

  if (rawTouchInputs[4] > (touchThresholds[4]*1.0)) tempDnState = true;
  else tempDnState = (filteredTouchTempDown > touchThresholds[4]);


}



void setup() {
  setupPWM();
  Serial.begin(115200);
  delay(100);  // Allow time for things to settle
  Serial.println("");
  Serial.println("-------------------------------");
  Serial.println("Starting Program OTL2 [Oct 6 2025]");
  Serial.println("-------------------------------");

  startTime = millis();  // capture the time at boot
  

  ws2812.begin();  // Initialize WS2812 LED
  ws2812.clear();  // Clear all pixels (set to black)
  ws2812.show();   // Clear WS2812 LED
  setupPWM();
  updatePWM();

    if ((millis() - startTime) > 2000) {
    ignoreRepeats = false;
  }

  calibrateTouchSensors();
}

void loop() {
  // Process touch inputs
  processTouchInputs();

  // Get the current time for tap detection
  unsigned long currentTime = millis();

  // Handle on/off + long-hold to toggle WS2812
if (ledSwitchState && !lastLedSwitchState) {
  // Rising edge (finger touched)
  bool wasOff = !ledState;

  ledState = !ledState;
  updatePWM();
  DEBUG_PRINT("LED turned: ");
  DEBUG_PRINTLN(ledState ? "ON" : "OFF");

  // Arm the 5s hold ONLY if we just went OFF -> ON
  if (wasOff && ledState) {
    ledHoldArmed = true;
    wsHoldFired  = false;
    ledHoldStart = millis();
  } else {
    // If we went ON -> OFF, do not arm
    ledHoldArmed = false;
    wsHoldFired  = false;
  }

  delay(10);
}
lastLedSwitchState = ledSwitchState;

// If armed, and finger is still down, check for 5s hold to toggle WS2812
if (ledHoldArmed && ledSwitchState && !wsHoldFired) {
  if (millis() - ledHoldStart >= ws2812ToggleHoldMs) {
    ws2812Enabled = !ws2812Enabled;
    
    if (ws2812Enabled) {
      // Turn the pixel on immediately to match current brightness/temp
      updatePWM();  // immediate WS2812 show because ws2812Enabled is now true
    } else {
      // Turn the pixel off immediately
      ws2812.clear();
      ws2812.show();
    }

    DEBUG_PRINT("Addressable LED ");
    DEBUG_PRINTLN(ws2812Enabled ? "ENABLED (3s hold on power-on)" : "DISABLED (3s hold on power-on)");
    wsHoldFired = true;  // prevent repeat until release
  }
}

// Disarm on release
if (!ledSwitchState) {
  ledHoldArmed = false;
}

  // if (ledSwitchState && !lastLedSwitchState) {
  //   ledState = !ledState;
  //   updatePWM();
  //   DEBUG_PRINT("LED turned: ");
  //   DEBUG_PRINTLN(ledState ? "ON" : "OFF");
  //   delay(10);
  // }
  // lastLedSwitchState = ledSwitchState;


  // ------------------ Brightness Increase (Switch Up) ------------------
  if (switchUpState) {
    ledState = true;                                          // turn on LED if off
    if (!lastSwitchUpState) {                                 // Button just pressed
      if (currentTime - lastTapTimeUp < doubleTapInterval) {  // Double-tap detected
        brightness = maxBrightness;  
        DEBUG_PRINT("Brightness set to MAX (");
        DEBUG_PRINT(maxBrightnessPct * 100.0f, 1);  // 1 decimal place (e.g., 95.0)
        DEBUG_PRINTLN("%)");
        updatePWM();
      } else {
        switchUpHoldStart = currentTime;  // Start hold timer
      }
      lastTapTimeUp = currentTime;
    } else if (currentTime - switchUpHoldStart > dimholdDelay) {  // Press-and-hold detected
      brightness = min<uint16_t>(brightness + step, maxBrightness);                 // Increment brightness
      DEBUG_PRINT("Brightness increased to: ");
      DEBUG_PRINTLN(brightness);
      updatePWM();
      switchUpHoldStart = currentTime;  // Reset hold timer
    }
  }
  lastSwitchUpState = switchUpState;

  // ------------------ Brightness Decrease (Switch Down) ------------------
  if (switchDnState) {
    ledState = true;                                            // turn on LED if off
    if (!lastSwitchDnState) {                                   // Button just pressed
      if (currentTime - lastTapTimeDown < doubleTapInterval) {  // Double-tap detected
        brightness = minBrightness;                                         // Min brightness
        DEBUG_PRINTLN("Brightness set to MIN (1%)");
        updatePWM();
      } else {
        switchDnHoldStart = currentTime;  // Start hold timer
      }
      lastTapTimeDown = currentTime;
    } else if (currentTime - switchDnHoldStart > dimholdDelay) {  // Press-and-hold detected
      brightness = max((uint16_t)(brightness - step), minBrightness);                     // Decrement brightness
      DEBUG_PRINT("Brightness decreased to: ");
      DEBUG_PRINTLN(brightness);
      updatePWM();
      switchDnHoldStart = currentTime;  // Reset hold timer
    }
  }
  lastSwitchDnState = switchDnState;

  // --- Hold BOTH temp buttons 2s => reset to equal ratio (50/50) ---
  bool bothTempPressed = tempUpState && tempDnState && ledState;
  if (bothTempPressed) {
    if (!tempBothHoldArmed) {
      tempBothHoldArmed = true;
      tempBothHoldStart = currentTime;
      tempBothHoldFired = false;
    } else if (!tempBothHoldFired && (currentTime - tempBothHoldStart >= tempBothResetHoldMs)) {
      tempRatio = 0.5f;           // equal warm/cool
      updatePWM();
      DEBUG_PRINTLN("White balance reset to 50/50 (held warm+cool for 2s).");
      tempBothHoldFired = true;   // only fire once per hold
    }
  } else {
    // released: re-arm for next time
    tempBothHoldArmed = false;
    tempBothHoldFired = false;
  }


  // ------------------ Temperature Increase (Temp Up) ------------------
  if (!bothTempPressed && tempUpState && ledState) {              // Only adjust temperature if LED is on
    if (!lastTempUpState) {                                       // Button just pressed
      if (currentTime - lastTapTimeTempUp < doubleTapInterval) {  // Double-tap detected
        tempRatio = 1.0;                                          // Max cool
        DEBUG_PRINTLN("Temperature set to MAX COOL (1.0)");
        updatePWM();
      } else {
        tempUpHoldStart = currentTime;  // Start hold timer
      }
      lastTapTimeTempUp = currentTime;
    } else if (currentTime - tempUpHoldStart > tempholdDelay) {  // Press-and-hold detected
      tempRatio = min(tempRatio + tempRatioStep, 1.0f);                  // Increment cool ratio
      DEBUG_PRINT("White temp increase: ");
      DEBUG_PRINT(tempRatio, 4);
      DEBUG_PRINT(" | Warm temp decrease: ");
      DEBUG_PRINTLN(1.0f - tempRatio, 4);
      updatePWM();
      tempUpHoldStart = currentTime;  // Reset hold timer
    }
  }
  lastTempUpState = tempUpState;

  // ------------------ Temperature Decrease (Temp Down) ------------------
  if (!bothTempPressed && tempDnState && ledState) {                // Only adjust temperature if LED is on
    if (!lastTempDnState) {                                         // Button just pressed
      if (currentTime - lastTapTimeTempDown < doubleTapInterval) {  // Double-tap detected
        tempRatio = 0.0;                                            // Max warm
        DEBUG_PRINTLN("Temperature set to MAX WARM (0.0)");
        updatePWM();
      } else {
        tempDnHoldStart = currentTime;  // Start hold timer
      }
      lastTapTimeTempDown = currentTime;
    } else if (currentTime - tempDnHoldStart > tempholdDelay) {  // Press-and-hold detected
      tempRatio = max(tempRatio - tempRatioStep, 0.0f);                  // Decrement cool ratio
      DEBUG_PRINT("Cool temp decrease: ");
      DEBUG_PRINT(tempRatio, 4);
      DEBUG_PRINT(" | Warm temp increase: ");
      DEBUG_PRINTLN(1.0f - tempRatio, 4);
      updatePWM();
      tempDnHoldStart = currentTime;  // Reset hold timer
    }
  }
  lastTempDnState = tempDnState;


  // Get the current time for temperature reading
  unsigned long currentMillis = millis();

  // Read and print temperature every 10 seconds
  if (currentMillis - lastSensorReadTime >= sensorReadInterval) {
    lastSensorReadTime = currentMillis;
    readTemperature();
    readAmbientLight();
  }

  delay(5);  // Short debounce delay
}
