#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <driver/ledc.h>

// Pin Definitions
const uint8_t ledSwitch = T7;  // Capacitive touch input for on/off
const uint8_t switchUp = T8;   // Capacitive touch input for increasing brightness
const uint8_t switchDn = T9;   // Capacitive touch input for decreasing brightness
const uint8_t tempUp = T10;    // Capacitive touch input for increasing cool color
const uint8_t tempDn = T11;    // Capacitive touch input for increasing warm color
const uint8_t dimOut1 = 36;    // Cool white channel (PWM pin)
const uint8_t dimOut2 = 35;    // Warm white channel (PWM pin)
const uint8_t ws2812Pin = 40;  // WS2812 data pin

const uint8_t MAX_BRIGHTNESS = 95;   


bool ignoreRepeats = true;
uint32_t startTime = 0;

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
const unsigned long dimholdDelay = 10;        // Time to wait before considering a hold
const unsigned long tempholdDelay = 10;

unsigned long switchUpHoldStart = 0;
unsigned long switchDnHoldStart = 0;
unsigned long tempUpHoldStart = 0;
unsigned long tempDnHoldStart = 0;

// WS2812 LED Configuration
const uint16_t numPixels = 1;  // Number of WS2812 LEDs
Adafruit_NeoPixel ws2812 = Adafruit_NeoPixel(numPixels, ws2812Pin, NEO_GRB + NEO_KHZ800);

// Constants and Variables
const float gammaValue = 2.2;
const int step = 1;
int brightness = 50;
float tempRatio = 0.5;
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
  Serial.print("Raw ADC Value: ");
  Serial.println(adcValue);
  Serial.print("Sensor Voltage: ");
  Serial.println(voltage, 3);
  Serial.print("Photocurrent (µA): ");
  Serial.println(Iph, 2);
  Serial.print("Illuminance (lux): ");
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
    Serial.println("ADC value is invalid.");
    return NAN;  // Return 'Not a Number' to indicate error
  }

  // Calculate the voltage across the thermistor
  float voltage = (adcValue / adcMaxValue) * vRef;

  // Calculate the resistance of the thermistor
  float thermistorResistance = seriesResistor * (voltage / (vRef - voltage));

  // Ensure the resistance is valid
  if (thermistorResistance <= 0) {
    Serial.println("Calculated resistance is invalid.");
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


  Serial.print("ADC Value: ");
  Serial.println(adcValue);
  Serial.print("Voltage: ");
  Serial.println(voltage, 3);
  Serial.print("Thermistor Resistance: ");
  Serial.println(thermistorResistance, 2);
  Serial.print("Calculated Temperature: ");
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

void updatePWM() {
  static uint32_t lastColor = ws2812.Color(0, 0, 0);  // To store the last color state
  static uint8_t lastBrightness = 0;                  // To store the last brightness state

  if (!ledState) {
    // Turn off LED
    ledc_set_duty(LEDC_LOW_SPEED_MODE, coolChannel, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, warmChannel, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, coolChannel);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, warmChannel);

    //neopixel. can remove later
    ws2812.clear();  // Clear all pixels (set to black)
    ws2812.show();   // Update the WS2812 LED
    return;
  }

  //if led is turned off and brightness is 0 then turn it back on at 50%
  if (ledState && brightness == 0) {
    Serial.println("LED turned on at 0% brightness, resetting to 50%");
    brightness = 50;
  }

  float brightnessRatio = brightness / 100.0f;
  uint32_t totalDuty = (uint32_t)(brightnessRatio * pwmMaxDuty);
  uint32_t coolDuty = (uint32_t)(totalDuty * tempRatio);
  uint32_t warmDuty = totalDuty - coolDuty;

  ledc_set_duty(LEDC_LOW_SPEED_MODE, coolChannel, coolDuty);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, warmChannel, warmDuty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, coolChannel);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, warmChannel);


  // Calculate WS2812 brightness and color
  uint8_t wsBrightness = (uint8_t)(brightnessRatio * 255);
  uint8_t red = (uint8_t)((1.0f - tempRatio) * wsBrightness);  // Red for warm
  uint8_t blue = (uint8_t)(tempRatio * wsBrightness);          // Blue for cool

  // Save the last state before updating
  lastColor = ws2812.Color(red, 0, blue);
  lastBrightness = wsBrightness;

  // Update WS2812 LED
  ws2812.setPixelColor(0, lastColor);
  ws2812.show();
}

void cycleColors() {
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
const int bufferSize = 6;                       // Number of samples to average
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
    Serial.println("Repeated value detected 10 times. Restarting...");
    // ESP.restart();  // Restart the ESP32
  }
}

constexpr uint8_t PAD_COUNT = 5;
uint8_t touchPins[PAD_COUNT] = { ledSwitch, switchUp, switchDn, tempUp, tempDn };

uint32_t baseline[PAD_COUNT];
uint32_t threshold[PAD_COUNT];
const float factor[PAD_COUNT] = { 0.95, 0.85, 0.85, 0.85, 0.85 };

void calibrateTouchPads()
{
  const uint16_t samples = 100;
  for (int i = 0; i < PAD_COUNT; ++i) {
    uint64_t sum = 0;
    for (int s = 0; s < samples; ++s) {
      sum += touchRead(touchPins[i]);
      delay(2);               // 2 ms between samples → 200 ms total
    }
    baseline[i]  = sum / samples;         // average “no-touch” value
    threshold[i] = baseline[i] * factor[i];
  }

  Serial.println(F("Touch baselines / thresholds:"));
  for (int i = 0; i < PAD_COUNT; ++i) {
    Serial.printf("Pad %d  base=%u  thresh=%u\n",
                  touchPins[i], baseline[i], threshold[i]);
  }
}

void processTouchInputs() {

  

// ── 1. store as integers (touchRead returns uint16_t) ─────────────────────
uint16_t rawTouch[PAD_COUNT] = {
  touchRead(ledSwitch),
  touchRead(switchUp),
  touchRead(switchDn),
  touchRead(tempUp),
  touchRead(tempDn)
};

// ── 2. ONE decision per pad, based on the dynamic threshold ───────────────
ledSwitchState = (rawTouch[0] < threshold[0]);   // flip < to > if touch raises the value
switchUpState  = (rawTouch[1] < threshold[1]);
switchDnState  = (rawTouch[2] < threshold[2]);
tempUpState    = (rawTouch[3] < threshold[3]);
tempDnState    = (rawTouch[4] < threshold[4]);

// // ── 3. optional debug print with the correct %u specifier ────────────────
// Serial.printf("raw  LS=%u SU=%u SD=%u TU=%u TD=%u\n",
//               rawTouch[0], rawTouch[1], rawTouch[2],
//               rawTouch[3], rawTouch[4]);

  // // Update moving averages
  // updateMovingAverage(rawTouchInputs, 5);

  // // Map filtered values to variables
  // filteredLedSwitch = filteredValues[0];
  // filteredTouchUp = filteredValues[1];
  // filteredTouchDown = filteredValues[2];
  // filteredTouchTempUp = filteredValues[3];
  // filteredTouchTempDown = filteredValues[4];

  // Add immediate detection for significant changes
  // Serial.print("raw ");
  // Serial.println(rawTouchInputs[0]);
  // Serial.print("filtered ");
  // Serial.println(filteredLedSwitch);
//   if (rawTouchInputs[0] > 100000) ledSwitchState = true;
//   else ledSwitchState = (filteredLedSwitch > 75000);

//   if (rawTouchInputs[1] > 90000) switchUpState = true;
//   else switchUpState = (filteredTouchUp > 55000);

//   if (rawTouchInputs[2] > 90000) switchDnState = true;
//   else switchDnState = (filteredTouchDown > 55000);

//   if (rawTouchInputs[3] > 90000) tempUpState = true;
//   else tempUpState = (filteredTouchTempUp > 60000);

//   if (rawTouchInputs[4] > 100000) tempDnState = true;
//   else tempDnState = (filteredTouchTempDown > 63000);

//   if (!ignoreRepeats) {
//     checkRepeatedValues(rawTouchInputs[0]);
//   }
}



void setup() {
  setupPWM();
  Serial.begin(115200);
  delay(100);  // Allow time for things to settle
  Serial.println("Starting Program diyson_stable...");

  calibrateTouchPads();

  startTime = millis();  // capture the time at boot

  ws2812.begin();  // Initialize WS2812 LED
  ws2812.clear();  // Clear all pixels (set to black)
  ws2812.show();   // Clear WS2812 LED
  
  updatePWM();

  //get out the bad initial values
  for (int i = 0; i < 10; i++) {
    processTouchInputs();
  }
}

void loop() {

  if ((millis() - startTime) > 2000) {
    ignoreRepeats = false;
  }

  // Process touch inputs
  processTouchInputs();

  // Handle on/off
  if (ledSwitchState && !lastLedSwitchState) {
    ledState = !ledState;
    updatePWM();
    Serial.print("LED turned: ");
    Serial.println(ledState ? "ON" : "OFF");
    delay(10);
  }
  lastLedSwitchState = ledSwitchState;


  // Get the current time for tap detection
  unsigned long currentTime = millis();

  // ------------------ Brightness Increase (Switch Up) ------------------
  if (switchUpState) {
    ledState = true;                                          // turn on LED if off
    if (!lastSwitchUpState) {                                 // Button just pressed
      if (currentTime - lastTapTimeUp < doubleTapInterval) {  // Double-tap detected
        brightness = 100;                                     // Max brightness
        Serial.println("Brightness set to MAX (100%)");
        updatePWM();
      } else {
        switchUpHoldStart = currentTime;  // Start hold timer
      }
      lastTapTimeUp = currentTime;
    } else if (currentTime - switchUpHoldStart > dimholdDelay) {  // Press-and-hold detected
      brightness = min(brightness + step, 100);                   // Increment brightness
      Serial.print("Brightness increased to: ");
      Serial.println(brightness);
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
        brightness = 1;                                         // Min brightness
        Serial.println("Brightness set to MIN (1%)");
        updatePWM();
      } else {
        switchDnHoldStart = currentTime;  // Start hold timer
      }
      lastTapTimeDown = currentTime;
    } else if (currentTime - switchDnHoldStart > dimholdDelay) {  // Press-and-hold detected
      brightness = max(brightness - step, 1);                     // Decrement brightness
      Serial.print("Brightness decreased to: ");
      Serial.println(brightness);
      updatePWM();
      switchDnHoldStart = currentTime;  // Reset hold timer
    }
  }
  lastSwitchDnState = switchDnState;

  // ------------------ Temperature Increase (Temp Up) ------------------
  if (tempUpState && ledState) {                                  // Only adjust temperature if LED is on
    if (!lastTempUpState) {                                       // Button just pressed
      if (currentTime - lastTapTimeTempUp < doubleTapInterval) {  // Double-tap detected
        tempRatio = 1.0;                                          // Max cool
        Serial.println("Temperature set to MAX COOL (1.0)");
        updatePWM();
      } else {
        tempUpHoldStart = currentTime;  // Start hold timer
      }
      lastTapTimeTempUp = currentTime;
    } else if (currentTime - tempUpHoldStart > tempholdDelay) {  // Press-and-hold detected
      tempRatio = min(tempRatio + 0.01f, 1.0f);                  // Increment cool ratio
      Serial.print("White temp increase: ");
      Serial.print(tempRatio, 2);
      Serial.print(" | Warm temp decrease: ");
      Serial.println(1.0f - tempRatio, 2);
      updatePWM();
      tempUpHoldStart = currentTime;  // Reset hold timer
    }
  }
  lastTempUpState = tempUpState;

  // ------------------ Temperature Decrease (Temp Down) ------------------
  if (tempDnState && ledState) {                                    // Only adjust temperature if LED is on
    if (!lastTempDnState) {                                         // Button just pressed
      if (currentTime - lastTapTimeTempDown < doubleTapInterval) {  // Double-tap detected
        tempRatio = 0.0;                                            // Max warm
        Serial.println("Temperature set to MAX WARM (0.0)");
        updatePWM();
      } else {
        tempDnHoldStart = currentTime;  // Start hold timer
      }
      lastTapTimeTempDown = currentTime;
    } else if (currentTime - tempDnHoldStart > tempholdDelay) {  // Press-and-hold detected
      tempRatio = max(tempRatio - 0.01f, 0.0f);                  // Decrement cool ratio
      Serial.print("Cool temp decrease: ");
      Serial.print(tempRatio, 2);
      Serial.print(" | Warm temp increase: ");
      Serial.println(1.0f - tempRatio, 2);
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

  delay(10);  // Short debounce delay
}