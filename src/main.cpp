#include <Arduino.h>
// #include <SPI.h>
// #include <EEPROM.h>
#include <MIDI.h>
#include <FastLED.h>

#define NUM_LEDS 89
#define DATA_PIN 0
#define CLOCK_PIN 3
#define LED 13 // LED pin on Arduino Uno

MIDI_CREATE_INSTANCE(HardwareSerial, Serial3, MIDI);
byte notes[NUM_LEDS];
CRGB leds[NUM_LEDS];

void noteOn(byte channel, byte note, byte velocity)
{
  notes[note] = velocity;
}

void noteOff(byte channel, byte note, byte velocity)
{
  notes[note] = 0;
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

long lastTime = 0, rt = 0;

char keyString[88] = "";
void loop()
{
  MIDI.read();

  // Set Frame
  long currentTime = millis();
  long dt = currentTime - lastTime;
  rt += dt;
  if(rt > 100) {
    for(int i = 0; i < 88; i++) {
      keyString[i] = notes[i]? '#' : '-';
    }
    Serial.println(keyString);
    rt-=100;
  }
  lastTime = currentTime;
  uint8_t cycle = (256 * (currentTime % 4000)) / 4000;

  // Do LEDs
  for(int i = 0; i < 88; i++) {
    leds[i] = CHSV(cycle, 255, notes[i]);
  }
  leds[88] = CHSV(cycle, 255, 255);

  // Show Frame
  FastLED.show();
}