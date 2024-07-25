#include <Arduino.h>
#include <FastLED.h>
#include <heltec_unofficial.h>

#define DEBUG

#define NUM_KEYS 88
#define P_SELECT 7
#define DATA_PIN 47
#define CLOCK_PIN 48
#define FRAME_RATE 30

#define EIGHT_HOURS 28800000
#define ON_TIME 5000
#define DISPLAY_TOGGLE_TIME 3000

#define IDLE_MS 20000
#define STUCK_NOTE_TIMEOUT 8000
const double cycle_ms = 12000.0;

#define FREQUENCY 905.2 // for US
#define BANDWIDTH 250.0 // kHz
#define SPREADING_FACTOR 8
#define TRANSMIT_POWER 0
const byte syncword = 0x69;

bool noteFirstOn[NUM_KEYS] = {false}, outputSelf = false;
int notes[NUM_KEYS] = {0};
double activeHue[NUM_KEYS] = {0.0};
long noteOnTime[NUM_KEYS] = {0};

volatile bool rxFlag = false;
byte rxdata[100], txdata[sizeof(float) * 2];
float rssi, snr;
byte key, value;

int leftSide = 0, rightSide;
bool didLeftEnd = false;

bool test = false, test2 = false;

long lastTime = 0, rt = 0, ft = 0, lastRXtime;
const int frame_delay = 1000 / FRAME_RATE;

CRGB leds[NUM_KEYS];
double intensity[NUM_KEYS], d_intensity[NUM_KEYS], hue[NUM_KEYS] = {-1.0}, d_hue[NUM_KEYS], t = 0.0, targetHue[NUM_KEYS];

uint16_t tenacity_val, entropy_val, chroma_val;
double tenacity_f, entropy_f, chrome_f;

#define TENACITY_MAX 0.151
#define TENACITY_SCALE 0.15

#define ENTROPY_MIN 0.0
#define ENTROPY_SCALE 0.49

#define CHROMA_MIN 0.0
#define CHROMA_SCALE 0.99

void doSetting(int idx, uint16_t value)
{
  switch (idx)
  {
  case 0:
    tenacity_val = value;
    tenacity_f = TENACITY_MAX - TENACITY_SCALE * u5ToF(tenacity_val);
    Serial.printf("Tenacity: %f\n", tenacity_f);
    break;
  case 1:
    entropy_val = value;
    entropy_f = ENTROPY_MIN + ENTROPY_SCALE * u5ToF(entropy_val);
    Serial.printf("Entropy: %f\n", entropy_f);
    break;
  case 2:
    chroma_val = value;
    chrome_f = CHROMA_MIN + CHROMA_SCALE * u5ToF(chroma_val);
    Serial.printf("Chroma: %f\n", chrome_f);
    break;
  default:
    break;
  }
}

void note(byte note, byte velocity)
{
  if (velocity > 0)
  {
    noteOnTime[note] = millis();
    noteFirstOn[note] = true;
    notes[note] = min(2 * velocity + 55, 255);
  }
  else
  {
    notes[note] = 0;
  }

  // Serial.printf("\nkey %i\t%i", note, velocity);

  // if (velocity)
  // {
  //   Serial.printf("Note: %i\tVelocity: %i\n", note, notes[note]);
  // }
}

void rx()
{
  rxFlag = true;
}

double mod1(double x)
{
  double r = fmod(x, 1);
  if (r < 0)
  {
    r += 1;
  }
  return r;
}

uint8_t fToU8(double x)
{
  return (uint8_t)(x * 255);
}

double u8ToF(uint8_t x)
{
  return (double)x / 255;
}

double u5ToF(uint8_t x)
{
  return (double)x / 31;
}
bool print = true;

double closestPath(double a, double b)
{
  double d = b - a;
  if (abs(d) > (1 / 255))
  {
    if (abs(d) > 0.5)
    {
      d = (d > 0 ? 1 : -1) - d;
      d *= -1;
    }
  }

  // if (print)
  // {
  //   Serial.printf("\nClosest Path from %.3f to %.3f is %.3f\n", a, b, d);
  // }
  return d;
}

void drawSettings()
{
  display.clear();
  display.drawString(64, 0, "T: " + String(tenacity_val));
  display.drawProgressBar(0, 14, 128, 4, (tenacity_val * 100) / 31);
  display.drawString(64, 20, "E: " + String(entropy_val));
  display.drawProgressBar(0, 36, 128, 4, (entropy_val * 100) / 31);
  display.drawString(64, 40, "C: " + String(chroma_val));
  display.drawProgressBar(0, 58, 128, 4, (chroma_val * 100) / 31);
  display.display();
}

void setup()
{
  LEDS.addLeds<SK9822, DATA_PIN, CLOCK_PIN>(leds, NUM_KEYS);
  pinMode(P_SELECT, OUTPUT);
  digitalWrite(P_SELECT, outputSelf);

  pinMode(42, INPUT_PULLDOWN);

  heltec_setup();
  both.println("Radio init");
  RADIOLIB_OR_HALT(radio.begin());
  radio.setDio1Action(rx);
  both.printf("Sync Word: %x\n", syncword);
  RADIOLIB_OR_HALT(radio.setSyncWord(syncword));
  both.printf("Frequency: %.2f MHz\n", FREQUENCY);
  RADIOLIB_OR_HALT(radio.setFrequency(FREQUENCY));
  both.printf("Bandwidth: %.1f kHz\n", BANDWIDTH);
  RADIOLIB_OR_HALT(radio.setBandwidth(BANDWIDTH));
  both.printf("Spreading Factor: %i\n", SPREADING_FACTOR);
  RADIOLIB_OR_HALT(radio.setSpreadingFactor(SPREADING_FACTOR));
  both.printf("TX power: %i dBm\n", TRANSMIT_POWER);
  RADIOLIB_OR_HALT(radio.setOutputPower(TRANSMIT_POWER));
  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));

  for (int i = 0; i < 3; i++)
  {
    doSetting(i, 16);
  }

  display.setTextAlignment(TEXT_ALIGN_CENTER);
  drawSettings();
}

long lastPrint = 0;

long currentTime, dt, displayTimeRemaining = 0, powerTimer = DISPLAY_TOGGLE_TIME;

bool displaying = true;

void loop()
{
  heltec_loop();

  currentTime = millis();
  dt = currentTime - lastTime;
  lastTime = currentTime;
  ft += dt;

  if (notes[0] > 0 && notes[87] > 0 && powerTimer > 0)
  {
    powerTimer -= dt;
    if (powerTimer <= 0)
    {
      if (displayTimeRemaining > 0)
      {
        displayTimeRemaining = 0;
      }
      else
      {
        displayTimeRemaining = ON_TIME;
      }
    }
  }
  else
  {
    powerTimer = DISPLAY_TOGGLE_TIME;
  }

  if (displayTimeRemaining > 0)
  {
    displaying = true;
    displayTimeRemaining -= dt;
  }
  else
  {
    displaying = false;
  }

  if (rxFlag)
  {
    rxFlag = false;

    radio.readData(rxdata, 0);
    if (_radiolib_status == RADIOLIB_ERR_NONE)
    {
      int i = 0;
      while (rxdata[i] != 0)
      {
        key = rxdata[i];
        value = rxdata[i + 1];

        // Serial.printf("[%i\t%i]\t", key, value);

        if (key < 100)
        {
          // Note recieved
          note(key - 1, value == 255 ? 0 : value);

          lastRXtime = currentTime;
          if (!outputSelf)
          {
            outputSelf = true;
            digitalWrite(P_SELECT, outputSelf);
          }
        }
        else if (key == 200)
        {
          rssi = radio.getRSSI();
          snr = radio.getSNR();

          memcpy(txdata, &rssi, sizeof(float));
          memcpy(txdata + sizeof(float), &snr, sizeof(float));

          both.printf("  RSSI: %.2f dBm\n", rssi);
          both.printf("  SNR: %.2f dB\n", snr);

          radio.transmit(txdata, 2 * sizeof(float));
        }
        else
        {
          doSetting(key - 100, value);
          drawSettings();
        }
        i += 2;
      }
      Serial.println();
    }
    memset(rxdata, 0, 100);
    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
  }

  if (!displaying)
  {
    outputSelf = true;
    digitalWrite(P_SELECT, outputSelf);
    FastLED.clear();
    FastLED.show();
  }
  else if (outputSelf)
  {
    // Show LED Frame
    if (ft > frame_delay)
    {

      // print = false;
      // if (currentTime - lastPrint > 400)
      // {
      //   print = true;
      //   Serial.print(".");
      //   lastPrint = currentTime;
      // }

      ft -= frame_delay;
      t = fmod((double)currentTime / cycle_ms, 1.0);

      didLeftEnd = false;
      for (int i = 0; i < NUM_KEYS; i++)
      {
        // Fade all or turn off
        if (intensity[i] > 0)
        {
          if (intensity[i] > tenacity_f && intensity[i] > 0.01)
          {
            if (i == 3)
            {
              Serial.printf("\n%.3f - %.3f = ", intensity[i], tenacity_f);
            }
            intensity[i] -= tenacity_f;
            if (i == 3)
            {
              Serial.printf("%.3f", intensity[i]);
            }
          }
          else
          {
            intensity[i] = 0;
            hue[i] = -1.0;

            if (i == 3)
            {
              Serial.printf("\n-/- %.3f", intensity[i]);
            }
          }
        }

        // Process each "on" note
        if (notes[i] > 0)
        {
          // Check if note has been stuck on
          if (currentTime - noteOnTime[i] >= STUCK_NOTE_TIMEOUT)
          {
            note(i, 0);
          }
          else
          {
            // Set it's hue if it was just pressed
            if (noteFirstOn[i])
            {
              activeHue[i] = fmod(t + (float)random(0, (long)(chrome_f * 1000)) / 1000, 1.0);
            }
            hue[i] = activeHue[i];

            // Restore intensity for note still held
            intensity[i] = u8ToF(notes[i]);

            rightSide = i;
            targetHue[i] = hue[i];
            double hueIncrement = 0.0;

            if (!didLeftEnd)
            {
              for (int j = 0; j < rightSide; j++)
              {
                targetHue[j] = hue[rightSide];
              }
              didLeftEnd = true;
            }
            else if (rightSide - leftSide > 1)
            {
              hueIncrement = closestPath(hue[leftSide], hue[rightSide]) / (rightSide - leftSide);
              for (int j = leftSide + 1; j < rightSide; j++)
              {
                targetHue[j] = mod1(targetHue[j - 1] + hueIncrement);
              }
            }
            leftSide = rightSide;
          }
        }

        // Process entropy
        if (i > 0)
        {
          double d = (intensity[i] - intensity[i - 1]) * entropy_f;
          d_intensity[i - 1] += d;
          d_intensity[i] -= d;
        }
      }
      // At least one key pressed, so fill in the right side
      if (didLeftEnd)
      {
        for (int j = leftSide + 1; j < NUM_KEYS; j++)
        {
          targetHue[j] = hue[leftSide];
        }
      }

      for (int i = 0; i < NUM_KEYS; i++)
      {
        intensity[i] += d_intensity[i];
        d_intensity[i] = 0.0;

        if (intensity[i])
        {
          if (hue[i] < 0)
          {
            hue[i] = targetHue[i];
          }
          else
          {
            hue[i] += (targetHue[i] - hue[i]) * 0.3;
          }
        }

        // Serial.printf("%.2f,", targetHue[i]);

        // hue[i] = mod1(hue[i] + d_hue[i]);
        // d_hue[i] = 0.0;

        uint8_t s = 255;
        if (noteFirstOn[i])
        {
          s = 0;
          noteFirstOn[i] = false;
        }

        if (i == 3 && intensity[i] > 0)
        {
          Serial.printf("\n%.3f\t%i", intensity[i], fToU8(intensity[i]));
        }
        leds[i] = CHSV(fToU8(hue[i]), s, fToU8(intensity[i]));
      }
      FastLED.show();
    }

    if (currentTime - lastRXtime > IDLE_MS)
    {
      outputSelf = false;
      digitalWrite(P_SELECT, outputSelf);
    }
  }
}
