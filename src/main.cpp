#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <MIDI.h>
#include <FastLED.h>

#define NUM_LEDS 88
#define DATA_PIN 0
#define CLOCK_PIN 3
#define LED 13 // LED pin on Arduino Uno

MIDI_CREATE_INSTANCE(HardwareSerial, Serial3, MIDI);
byte notes[NUM_LEDS];
CRGB leds[NUM_LEDS];

void noteOn(byte channel, byte note, byte velocity)
{
  Serial.printf("Note On: %d - %d\n", note, velocity);
  leds[note] = CRGB(velocity, 0, 0);
}

void noteOff(byte channel, byte note, byte velocity)
{
  Serial.printf("Note Off: %d\n", note);
  leds[note] = 0;
}

void setup()
{
  pinMode(LED, OUTPUT);
  FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);

  MIDI.turnThruOff();
  MIDI.setHandleNoteOn(noteOn);
  MIDI.setHandleNoteOff(noteOff);
  MIDI.begin();
}

long lastTime = 0;
void loop()
{
  MIDI.read();

  // Set Frame
  long currentTime = millis();
  long dt = currentTime - lastTime;
  lastTime = currentTime;
  uint8_t cycle = (256 * (currentTime % 4000)) / 4000;

  // Do LEDs
  for(int i = 0; i < 10; i++) {
    leds[i] = CHSV(cycle, 255, 255);
  }

  // Show Frame
  FastLED.show();
}