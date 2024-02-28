#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <MIDI.h>

// Simple tutorial on how to receive and send MIDI messages.
// Here, when receiving any message on channel 4, the Arduino
// will blink a led and play back a note for 1 second.

MIDI_CREATE_INSTANCE(HardwareSerial, Serial3, MIDI);

#define LED 13 // LED pin on Arduino Uno

void setup()
{
  pinMode(LED, OUTPUT);
  MIDI.turnThruOff();
  MIDI.setHandleNoteOn([](byte channel, byte note, byte velocity)
    {
      Serial.printf("Note On, ch=%d, note=%d, v=%d", channel, note, velocity);
      digitalWrite(LED,HIGH);
      delay(50);
      digitalWrite(LED,LOW);
      delay(50); 
    });
  MIDI.begin(MIDI_CHANNEL_OMNI); // Initiate MIDI communications, listen to all channels
}

void loop()
{
  MIDI.read();
}