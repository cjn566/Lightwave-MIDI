#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <MIDI.h>
#include <WS2812Serial.h>
#define USE_WS2812SERIAL
#include <FastLED.h>

#define NUM_LEDS 120
#define NUM_KEYS 88
#define DATA_PIN 8
#define CLOCK_PIN 3
#define LED 13 // LED pin on Arduino Uno
#define FRAME_RATE 60
#define MIDI_OFFSET 21

MIDI_CREATE_INSTANCE(HardwareSerial, Serial3, MIDI);
int notes[NUM_KEYS];
CRGB leds[NUM_LEDS];

void noteOn(byte channel, byte note, byte velocity)
{
  notes[note - MIDI_OFFSET] = velocity;
}

void noteOff(byte channel, byte note, byte velocity)
{
  notes[note - MIDI_OFFSET] = 0;
}

void setup()
{
  pinMode(LED, OUTPUT);
  LEDS.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);

  MIDI.turnThruOff();
  MIDI.setHandleNoteOn(noteOn);
  MIDI.setHandleNoteOff(noteOff);
  MIDI.begin();
}

double mod1(double x){
  double r = fmod(x, 1);
  if(r < 0) {
    r += 1;
  }
  return r;
}

uint8_t fToU8(double x){
  return (uint8_t)(x * 255);
}
double u8ToF(uint8_t x){
  return (double)x / 255;
}

const int fade_rate = 250;
const double entropy = 0.01;

void fadeall() { for(int i = 0; i < NUM_LEDS; i++) { leds[i].nscale8(fade_rate); } }

long lastTime = 0, rt = 0, ft = 0;
const int frame_delay = 1000 / FRAME_RATE;
double intensity[NUM_LEDS], d_intensity[NUM_LEDS];
void loop()
{
  MIDI.read();

  // Set Frame
  long currentTime = millis();
  long dt = currentTime - lastTime;
  lastTime = currentTime;
  ft += dt;

  if(ft > frame_delay) {
    ft -= frame_delay;

    fadeall();

    uint8_t t = (256 * (currentTime % 4000)) / 4000;

    // Do LEDs
    for(int i = 0; i < 88; i++) {
      if(notes[i]){
        intensity[i] = u8ToF(notes[i]);
      }
    }

    // Entropy
    for(int i = 0; i < (NUM_LEDS-1); i++) {
      double d = intensity[i] - intensity[i+1];
      if(d > 0) {
        d_intensity[i+1] = d*entropy;
      } else {
        d_intensity[i] = -d*entropy;
      }
    }
    for(int i = 0; i < NUM_LEDS; i++) {
      intensity[i] += d_intensity[i];
      leds[i] = CHSV(t, 255, fToU8(intensity[i]));
    }


    // Last LED to verify LEDs are working
    leds[NUM_KEYS] = CHSV(t, 255, 255);

    // Show Frame
    FastLED.show();
  }
}