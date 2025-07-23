/*
 * LoRa Piano LED Visualizer
 *
 * This program receives MIDI-like note data over LoRa radio and displays it using
 * a FastLED strip representing 88 piano keys. Features dynamic visual effects,
 * configurable parameters, and automatic screensaver mode switching.
 */

#include <Arduino.h>
#include <FastLED.h>
#include <heltec_unofficial.h>

#define DEBUG true

// =============================================================================
// HARDWARE CONFIGURATION
// =============================================================================
#define NUM_KEYS 88      // Number of piano keys
#define NUM_LEDS 90      // Number of LEDs (88 keys + 2 extra)
#define KEY_LED_OFFSET 1 // Offset to center keys in LED strip
#define MUX_SELECT_PIN 7 // Pin controlling LED data source multiplexer
#define LED_DATA_PIN 47  // SK9822 LED strip data pin
#define LED_CLOCK_PIN 48 // SK9822 LED strip clock pin

// =============================================================================
// TIMING CONFIGURATION
// =============================================================================
#define FRAME_RATE 30                     // LED refresh rate (FPS)
#define EIGHT_HOURS 28800000              // 8 hours in milliseconds
#define DISPLAY_ON_TIME EIGHT_HOURS       // How long display stays on
#define DISPLAY_TOGGLE_TIME 3000          // Time holding both end keys to toggle display
#define SCREENSAVER_TIMEOUT 20000         // Inactivity timeout before screensaver mode
#define STUCK_NOTE_TIMEOUT 8000           // Timeout to prevent stuck notes
const double colorCycleTime_ms = 12000.0; // Base timing for color effects

// =============================================================================
// LORA RADIO CONFIGURATION
// =============================================================================
#define LORA_FREQUENCY 905.2    // LoRa frequency in MHz (US band)
#define LORA_BANDWIDTH 250.0    // LoRa bandwidth in kHz
#define LORA_SPREADING_FACTOR 8 // LoRa spreading factor
#define LORA_TRANSMIT_POWER 0   // LoRa transmit power in dBm
const byte loraSyncWord = 0x69; // LoRa synchronization word

// =============================================================================
// VISUAL EFFECT PARAMETERS
// =============================================================================
#define TENACITY_MAX 0.151     // Maximum fade rate
#define TENACITY_SCALE 0.15    // Scaling factor for tenacity
#define ENTROPY_MIN 0.0        // Minimum entropy effect
#define ENTROPY_SCALE 0.49     // Scaling factor for entropy
#define CHROMA_MIN 0.0         // Minimum chroma randomness
#define CHROMA_SCALE 0.99      // Scaling factor for chroma
#define CHROMA_CYCLE_RATE_SCALE 900.0  // Scaling factor for chroma cycle rate calculation
#define CHROMA_MIN_CYCLE_LENGTH 300.0  // Minimum cycle length (determines maximum cycle rate)
#define MIN_NOTE_INTENSITY 100 // Minimum note intensity

// =============================================================================
// GLOBAL STATE VARIABLES
// =============================================================================

// Note tracking arrays
bool isNoteFirstPress[NUM_KEYS] = {false}; // Track if note was just pressed
int noteIntensities[NUM_KEYS] = {0};       // Current note intensities (0-255)
double noteActiveHues[NUM_KEYS] = {0.0};   // Hue assigned when note first pressed
long noteOnTimestamps[NUM_KEYS] = {0};     // When each note was pressed

// LED visualization arrays
CRGB leds[NUM_LEDS];               // FastLED array
double ledIntensities[NUM_LEDS];   // Current LED intensities (0.0-1.0)
double deltaIntensities[NUM_LEDS]; // Intensity changes for entropy effect
double ledHues[NUM_LEDS] = {-1.0}; // Current LED hues (0.0-1.0, -1 = unset)
double deltaHues[NUM_LEDS];        // Hue changes for entropy effect
double hueCycle = 0.0;             // Normalized time parameter for effects

// LoRa communication
volatile bool hasReceivedData = false; // Flag set by LoRa interrupt
byte rxBuffer[100];                    // LoRa receive buffer
byte txBuffer[sizeof(float) * 2];      // LoRa transmit buffer
float rssiValue, snrValue;             // Radio signal quality metrics
byte receivedKey, receivedValue;       // Parsed LoRa message data

// System state
bool isInSelfMode = false; // True = show own LEDs, False = screensaver mode

// Timing variables
long currentMillis = 0;                     // Current time
long deltaTime = 0;                         // Time since last loop
long lastMillis = 0;                        // Previous loop time
long renderTime = 0;                        // Accumulated time for frame timing
long frameTime = 0;                         // Unused (kept for compatibility)
long lastLoRaReceiveTime;                   // Time of last LoRa message
long lastDisplayUpdate = 0;                 // Last display update time
const int frameDelayMs = 1000 / FRAME_RATE; // Milliseconds per frame

// Display and power management
long displayTimeRemaining = DISPLAY_ON_TIME; // Time before display turns off
long powerToggleTimer = DISPLAY_TOGGLE_TIME; // Timer for display toggle sequence

// Visual effect parameters (remotely configurable)
uint16_t tenacityValue, entropyValue, chromaValue; // Raw parameter values (0-31)
double tenacityRate, entropyRate, chromaCycleRate; // Calculated effect values

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

/**
 * Convert floating point value (0.0-1.0) to 8-bit integer (0-255)
 */
uint8_t floatToUint8(double value)
{
  return (uint8_t)(value * 255);
}

/**
 * Convert 8-bit integer (0-255) to floating point (0.0-1.0)
 */
double uint8ToFloat(uint8_t value)
{
  return (double)value / 255;
}

/**
 * Convert 5-bit integer (0-31) to floating point (0.0-1.0)
 */
double uint5ToFloat(uint8_t value)
{
  return (double)value / 31;
}

/**
 * Modulo operation that always returns positive result (0.0-1.0)
 */
double normalizeToUnit(double value)
{
  double result = fmod(value, 1);
  if (result < 0)
  {
    result += 1;
  }
  return result;
}

/**
 * Calculate shortest path between two hues on color wheel (0.0-1.0)
 * Returns signed distance (-0.5 to 0.5) where sign indicates direction
 */
double calculateHueDistance(double fromHue, double toHue)
{
  double delta = fromHue - toHue;

  // If distance is > 0.5, go the other way around the color wheel
  if (delta > 0.5)
  {
    delta -= 1.0; // Go backwards (negative direction)
  }
  else if (delta < -0.5)
  {
    delta += 1.0; // Go forwards (positive direction)
  }

  return delta;
}

// =============================================================================
// CONFIGURATION AND CONTROL FUNCTIONS
// =============================================================================

/**
 * Update visual effect parameter based on remotely received setting
 * @param paramIndex 0=tenacity, 1=entropy, 2=chroma
 * @param value Raw value from 0-31
 */
void updateVisualParameter(int paramIndex, uint16_t value)
{
  switch (paramIndex)
  {
  case 0: // Tenacity (fade rate)
    tenacityValue = value;
    // Adjust exponent for smoother response with slightly faster fade rate
    tenacityRate = TENACITY_MAX * pow(1.0 - uint5ToFloat(tenacityValue), 1.5);
    break;

  case 1: // Entropy (color bleeding)
    entropyValue = value;
    entropyRate = ENTROPY_MIN + ENTROPY_SCALE * uint5ToFloat(entropyValue);
    break;

  case 2: // Chroma (color cycle speed)
    chromaValue = value;
    if(chromaValue == 0)
    {
      chromaCycleRate = 0.0; // Disable chroma effect
    }
    else{
      chromaCycleRate = 1.0/((double)(31.0-chromaValue) * CHROMA_CYCLE_RATE_SCALE + CHROMA_MIN_CYCLE_LENGTH);
    }
    break;

  default:
    break;
  }
}

/**
 * Process received MIDI note data
 * @param noteIndex Piano key index (0-87)
 * @param velocity Note velocity (0 = note off, 1-127 = note on)
 */
void processNote(byte noteIndex, byte velocity)
{
  if (velocity > 0)
  {
    // Note on: record timing and calculate intensity
    noteOnTimestamps[noteIndex] = millis();
    isNoteFirstPress[noteIndex] = true;
    noteIntensities[noteIndex] = min(2 * velocity + MIN_NOTE_INTENSITY, 255); // Scale velocity
  }
  else
  {
    // Note off
    noteIntensities[noteIndex] = 0;
  }
}

/**
 * LoRa interrupt handler - called when data is received
 */
void onLoRaReceive()
{
  hasReceivedData = true;
}

/**
 * Draw current settings and status on OLED display
 */
void updateDisplay()
{
  display.clear();

  // Show current mode
  display.drawString(20, 0, isInSelfMode ? "Self" : "PB");

  // Show time remaining for display
  display.drawString(100, 0, String(displayTimeRemaining / 1000));

  // Show tenacity parameter and progress bar
  display.drawString(64, 0, "T: " + String(tenacityValue));
  display.drawProgressBar(0, 14, 128, 4, (tenacityValue * 100) / 31);

  // Show entropy parameter and progress bar
  display.drawString(64, 20, "E: " + String(entropyValue));
  display.drawProgressBar(0, 36, 128, 4, (entropyValue * 100) / 31);

  // Show chroma parameter and progress bar
  display.drawString(64, 40, "C: " + String(chromaValue));
  display.drawProgressBar(0, 58, 128, 4, (chromaValue * 100) / 31);

  display.display();
}

// =============================================================================
// SETUP FUNCTION
// =============================================================================

void setup()
{
  // Initialize LED strip
  LEDS.addLeds<SK9822, LED_DATA_PIN, LED_CLOCK_PIN>(leds, NUM_LEDS);

  // Configure multiplexer control pin for LED source switching
  pinMode(MUX_SELECT_PIN, OUTPUT);
  digitalWrite(MUX_SELECT_PIN, isInSelfMode);

  // Initialize Heltec board (radio, display, etc.)
  heltec_setup();

  // Configure LoRa radio
  both.println("Radio init");
  RADIOLIB_OR_HALT(radio.begin());
  radio.setDio1Action(onLoRaReceive);

  RADIOLIB_OR_HALT(radio.setSyncWord(loraSyncWord));
  RADIOLIB_OR_HALT(radio.setFrequency(LORA_FREQUENCY));
  RADIOLIB_OR_HALT(radio.setBandwidth(LORA_BANDWIDTH));
  RADIOLIB_OR_HALT(radio.setSpreadingFactor(LORA_SPREADING_FACTOR));
  RADIOLIB_OR_HALT(radio.setOutputPower(LORA_TRANSMIT_POWER));

  // Start listening for incoming messages
  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));

  // Initialize visual parameters to default values (16/31 ≈ middle range)
  for (int i = 0; i < 3; i++)
  {
    updateVisualParameter(i, 16);
  }

  // Configure and show initial display
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  updateDisplay();
}

// =============================================================================
// MAIN LOOP FUNCTION
// =============================================================================

void loop()
{
  heltec_loop();

  // Update timing
  currentMillis = millis();
  deltaTime = currentMillis - lastMillis;
  lastMillis = currentMillis;

  // Process incoming LoRa messages
  heltec_led(0);
  if (hasReceivedData)
  {
    hasReceivedData = false;

    radio.readData(rxBuffer, 0);
    if (_radiolib_status == RADIOLIB_ERR_NONE)
    {
      int bufferIndex = 0;
      heltec_led(20);

      // Parse message pairs (key, value)
      while (rxBuffer[bufferIndex] != 0)
      {
        receivedKey = rxBuffer[bufferIndex];
        receivedValue = rxBuffer[bufferIndex + 1];

#ifdef DEBUG
        Serial.printf("[%i\t%i]\n", receivedKey, receivedValue);
#endif

        if (receivedKey < 100)
        {
          // Note data received (key 0-99)
          processNote(NUM_KEYS - receivedKey, receivedValue == 255 ? 0 : receivedValue);

          // Switch to self mode when receiving note data
          lastLoRaReceiveTime = currentMillis;
          if (!isInSelfMode)
          {
            isInSelfMode = true;
            digitalWrite(MUX_SELECT_PIN, isInSelfMode);
            updateDisplay();
          }
        }
        else if (receivedKey == 200)
        {
          // Signal quality request - respond with RSSI/SNR
          rssiValue = radio.getRSSI();
          snrValue = radio.getSNR();

          memcpy(txBuffer, &rssiValue, sizeof(float));
          memcpy(txBuffer + sizeof(float), &snrValue, sizeof(float));

          radio.transmit(txBuffer, 2 * sizeof(float));
        }
        else
        {
          // Parameter update (key 100-102 for tenacity, entropy, chroma)
          updateVisualParameter(receivedKey - 100, receivedValue);
          updateDisplay();
        }

        bufferIndex += 2; // Move to next key-value pair
      }
    }

    // Clear buffer and resume listening
    memset(rxBuffer, 0, 100);
    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
  }

  // Handle display power toggle (hold both end keys for 3 seconds)
  if (noteIntensities[0] > 0 && noteIntensities[87] > 0)
  {
    if (powerToggleTimer > 0)
    {
      powerToggleTimer -= deltaTime;
      if (powerToggleTimer <= 0)
      {
        // Toggle display power state
        if (displayTimeRemaining > 0)
        {
          displayTimeRemaining = 0; // Turn off display
        }
        else
        {
          displayTimeRemaining = DISPLAY_ON_TIME; // Turn on display
          updateDisplay();
        }
      }
    }
  }
  else
  {
    powerToggleTimer = DISPLAY_TOGGLE_TIME; // Reset timer when keys released
  }

  // Update display state
  if (displayTimeRemaining > 0)
  {
    displayTimeRemaining -= deltaTime;

    // Trigger display update every 1000 milliseconds
    static long lastDisplayUpdate = 0;
    if (currentMillis - lastDisplayUpdate >= 1000)
    {
      lastDisplayUpdate = currentMillis;
      updateDisplay();
    }

    if (isInSelfMode)
    {
      // Self mode - render LED visualization
      renderLEDVisualization();

      // Check for screensaver timeout
      if (currentMillis - lastLoRaReceiveTime > SCREENSAVER_TIMEOUT)
      {
        isInSelfMode = false;
        digitalWrite(MUX_SELECT_PIN, isInSelfMode);
        updateDisplay();
      }
    }
  }
  else
  {
    isInSelfMode = true;
    digitalWrite(MUX_SELECT_PIN, isInSelfMode);
    display.clear();
    display.drawString(64, 20, "OFF");
    display.display();
    FastLED.clear(true); // Clear LEDs when display is off
  }
}
// =============================================================================
// LED VISUALIZATION RENDERING
// =============================================================================

/**
 * Main LED visualization rendering function
 * Handles note fading, color bleeding, entropy effects, and LED output
 * Uses LED-centric approach with offset mapping (88 keys mapped to 90 LEDs)
 */
void renderLEDVisualization()
{
  // Only render at target frame rate
  renderTime += deltaTime;
  hueCycle = fmod(hueCycle + (chromaCycleRate * deltaTime), 1.0);
  if (renderTime <= frameDelayMs)
  {
    return;
  }

  renderTime = 0; // Reset render time for next frame


  // Process each LED
  for (int ledIdx = 0; ledIdx < NUM_LEDS; ledIdx++)
  {

    // Calculate corresponding note index (with offset)
    int noteIdx = ledIdx - KEY_LED_OFFSET;

    // === FADE OUT INACTIVE LEDS ===
    if (ledIntensities[ledIdx] > 0)
    {
      if (ledIntensities[ledIdx] > tenacityRate && ledIntensities[ledIdx] > (1.0 / 255.0))
      {
        // Gradual fade based on tenacity setting
        ledIntensities[ledIdx] -= tenacityRate;
      }
      else
      {
        // Below threshold - turn off completely
        ledIntensities[ledIdx] = 0;
        ledHues[ledIdx] = -1.0; // Mark hue as unset
      }
    }

    // === PROCESS ACTIVE NOTES ===
    // Only process if this LED corresponds to a valid note (not the extra LEDs on ends)
    if (noteIdx >= 0 && noteIdx < NUM_KEYS && noteIntensities[noteIdx] > 0)
    {

      // Check for stuck notes and auto-release them
      if (currentMillis - noteOnTimestamps[noteIdx] >= STUCK_NOTE_TIMEOUT)
      {
        processNote(noteIdx, 0); // Force note off
      }
      else
      {

        // Assign hue for newly pressed notes
        if (isNoteFirstPress[noteIdx])
        {
          noteActiveHues[noteIdx] =  hueCycle;
#ifdef DEBUG
          Serial.printf("chromaValue=%d, hueCycle = %.4f, rate = %.6f\n", chromaValue, hueCycle,  chromaCycleRate);
#endif
        }
        ledHues[ledIdx] = noteActiveHues[noteIdx];

        // Set LED intensity based on note velocity
        ledIntensities[ledIdx] = uint8ToFloat(noteIntensities[noteIdx]);
      }
    }

    // === ENTROPY EFFECT ===
    // Spreads intensity and hue between neighboring LEDs for more organic look
    if (ledIdx > 0)
    {
      double intensityDifference = (ledIntensities[ledIdx] - ledIntensities[ledIdx - 1]) * entropyRate;
      deltaIntensities[ledIdx - 1] += intensityDifference;
      deltaIntensities[ledIdx] -= intensityDifference;

      // Hue propagation and smoothing logic
      if (ledHues[ledIdx - 1] < 0)
      {
        if (ledHues[ledIdx] >= 0)
        {
          ledHues[ledIdx - 1] = ledHues[ledIdx];
        }
      }
      else
      {
        if (ledHues[ledIdx] < 0)
        {
          ledHues[ledIdx] = ledHues[ledIdx - 1];
        }
        else
        {
          double hueDifference = calculateHueDistance(ledHues[ledIdx - 1], ledHues[ledIdx]);
          if (abs(hueDifference) > (1.0 / 255.0))
          {
            hueDifference *= 0.3;
            deltaHues[ledIdx - 1] -= hueDifference * ledIntensities[ledIdx];
            deltaHues[ledIdx] += hueDifference * ledIntensities[ledIdx - 1];
          }
        }
      }
    }
  }

  // === APPLY EFFECTS AND OUTPUT TO LEDS ===
  for (int ledIdx = 0; ledIdx < NUM_LEDS; ledIdx++)
  {

    // Calculate corresponding note index (with offset)
    int noteIdx = ledIdx - KEY_LED_OFFSET;

    // Apply entropy intensity changes
    ledIntensities[ledIdx] += deltaIntensities[ledIdx];
    deltaIntensities[ledIdx] = 0.0;

    // Apply entropy hue changes
    ledHues[ledIdx] = normalizeToUnit(ledHues[ledIdx] + deltaHues[ledIdx]);
    deltaHues[ledIdx] = 0.0;

    // Set saturation (0 = white flash for newly pressed notes)
    uint8_t saturation = 255;
    if (noteIdx >= 0 && noteIdx < NUM_KEYS && isNoteFirstPress[noteIdx])
    {
      saturation = 0; // White flash on first press
      isNoteFirstPress[noteIdx] = false;
    }

    // Convert to FastLED format and set LED color
    leds[ledIdx] = CHSV(floatToUint8(ledHues[ledIdx]), saturation, floatToUint8(ledIntensities[ledIdx]));
  }

  // Update physical LEDs
  FastLED.show();
}
