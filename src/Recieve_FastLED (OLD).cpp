#include <Arduino.h>
#include <FastLED.h>
#include <heltec_unofficial.h>

#define DEBUG

#define NUM_LEDS 120
#define NUM_KEYS 88
#define P_SELECT 7
#define DATA_PIN 47
#define CLOCK_PIN 48
#define FRAME_RATE 30


#define IDLE_MS 4000
#define STUCK_NOTE_TIMEOUT 4000

#define FREQUENCY 905.2 // for US
#define BANDWIDTH 250.0 // kHz
#define SPREADING_FACTOR 8
#define TRANSMIT_POWER 0
const byte syncword = 0x69;

#define FADE_MIN 0.001
#define FADE_SCALE 0.1

bool noteFirstOn[NUM_KEYS] = {false}, outputSelf = true;
int notes[NUM_KEYS] = {0};
double activeHue[NUM_KEYS] = {0.0};
uint32_t noteOnTime[NUM_KEYS] = {0};
CRGB leds[NUM_LEDS];

volatile bool rxFlag = false;
byte rxdata[100], txdata[sizeof(float) * 2];
float rssi, snr;
double entropy = 0.2, fade_rate = 0.008;
const int cycle_ms = 2500;
byte key, value;

bool test = false, test2 = false;

long lastTime = 0, rt = 0, ft = 0, lastRXtime;
const int frame_delay = 1000 / FRAME_RATE;
double intensity[NUM_LEDS], d_intensity[NUM_LEDS], hue[NUM_LEDS] = {-1.0}, d_hue[NUM_LEDS], t = 0.0;

void note(byte note, byte velocity)
{
  notes[note] = 2 * velocity;
  noteFirstOn[note] = true;
  if (velocity > 0)
  {
    noteOnTime[note] = millis();
  }
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
  return (uint8_t)(x * 256);
}

double u8ToF(uint8_t x)
{
  return (double)x / 256;
}

double u5ToF(uint8_t x)
{
  return (double)x / 32;
}

void setup()
{
  LEDS.addLeds<SK9822, DATA_PIN, CLOCK_PIN>(leds, NUM_LEDS);
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
}

void loop()
{
  heltec_loop();

  long currentTime = millis();
  long dt = currentTime - lastTime;
  lastTime = currentTime;
  ft += dt;
  rt += dt;

#ifdef DEBUG
  digitalWrite(P_SELECT, outputSelf);
#endif

  if (rxFlag)
  {
    rxFlag = false;

    lastRXtime = currentTime;
    if (!outputSelf)
    {
      outputSelf = true;
      digitalWrite(P_SELECT, outputSelf);
    }

    radio.readData(rxdata, 0);
    if (_radiolib_status == RADIOLIB_ERR_NONE)
    {
      int i = 0;
      while (rxdata[i] != 0)
      {
        key = rxdata[i];
        value = rxdata[i + 1];

        Serial.printf("[%i\t%i]\t", key, value);

        if (key < 100)
        {
          // Note recieved
          note(key - 1, value == 255 ? 0 : value);
        }
        else
        {
          switch (key)
          {
          case 100: // Fade
            fade_rate = (u5ToF(value) * FADE_SCALE) + FADE_MIN;
            Serial.printf("Fade rate: %.4f\n", fade_rate);
            break;
          case 101: // Entropy
            fade_rate = u5ToF(value);
            break;
          case 102: // Speed
            fade_rate = u5ToF(value);
            break;
          case 200: // Test mode
            rssi = radio.getRSSI();
            snr = radio.getSNR();

            memcpy(txdata, &rssi, sizeof(float));
            memcpy(txdata + sizeof(float), &snr, sizeof(float));

            both.printf("  RSSI: %.2f dBm\n", rssi);
            both.printf("  SNR: %.2f dB\n", snr);

            radio.transmit(txdata, 2 * sizeof(float));
            break;

          default:
            break;
          }
        }
        i += 2;
      }
      Serial.println();
    }
    memset(rxdata, 0, 100);
    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
  }

  // CHECK FOR IDLE
  if (currentTime - lastRXtime > IDLE_MS)
  {
    if (outputSelf)
    {
      outputSelf = false;
      digitalWrite(P_SELECT, outputSelf);
    }
  }
  
  // Show LED Frame
  else if (ft > frame_delay)
    {
      ft -= frame_delay;
      t = fmod((double)currentTime / (double)cycle_ms, 1.0);

      for (int i = 0; i < NUM_LEDS; i++)
      {
        if (intensity[i] > fade_rate)
        {
          intensity[i] -= fade_rate;
        }
        else
        {
          intensity[i] = 0;
          hue[i] = -1.0;
        }
      }

      for (int i = 0; i < 88; i++)
      {
        if (notes[i])
        {
          if (noteFirstOn[i])
          {
            activeHue[i] = mod1(t + ((double)i / NUM_KEYS));
          }
          hue[i] = activeHue[i];
          intensity[i] = u8ToF(notes[i]);
          // Check if note has been stuck on
          if (currentTime - noteOnTime[i] >= STUCK_NOTE_TIMEOUT)
          {
            note(i, 0);
          }
        }
      }

      for (int i = 0; i < (NUM_LEDS - 1); i++)
      {
        double d = (intensity[i + 1] - intensity[i]) * entropy;
        d_intensity[i] += d;
        d_intensity[i + 1] -= d;

        if (hue[i] < 0)
        {
          if (hue[i + 1] >= 0)
          {
            hue[i] = hue[i + 1];
          }
        }
        else
        {
          if (hue[i + 1] < 0)
          {
            hue[i + 1] = hue[i];
          }
          else
          {
            d = (hue[i] - hue[i + 1]);
            if (d > 0.001)
            {
              if (abs(d) > 0.5)
              {
                d = (d > 0 ? 1 : -1) - d;
                d *= -1;
              }
              d *= 0.3;
              d_hue[i] -= d * intensity[i + 1];
              d_hue[i + 1] += d * intensity[i];
            }
          }
        }
      }

      for (int i = 0; i < NUM_KEYS; i++)
      {
        intensity[i] += d_intensity[i];
        d_intensity[i] = 0.0;

        hue[i] = mod1(hue[i] + d_hue[i]);
        d_hue[i] = 0.0;

        leds[i] = CHSV(fToU8(hue[i]), noteFirstOn[i] ? 0 : 255, fToU8(intensity[i]));
        noteFirstOn[i] = false;
      }

      leds[NUM_KEYS] = CHSV(t, 255, 255);
      FastLED.show();
    }
  }
}
