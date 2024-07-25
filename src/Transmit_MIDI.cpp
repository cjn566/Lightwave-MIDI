#include <Arduino.h>
#include <heltec_unofficial.h>
#include <MIDI.h>

#define DEBUG

#define NUM_KEYS 88
#define MIDI_OFFSET 21
#define NUM_POTS 3

#define P_FADE 2
#define P_ENTROPY 3
#define P_SPEED 4
#define P_TESTMODE 42
#define P_SF1 39
#define P_SF2 40
#define P_SF3 41

#define AVERAGING_LEN 8
const byte syncword = 0x69;

// Turns the 'PRG' button into the power button, long press is off
#define HELTEC_POWER_BUTTON // must be before "#include <heltec.h>"
#define FREQUENCY 905.2     // for US

// Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0 and 500.0 kHz.
#define BANDWIDTH 250.0

// Number from 5 to 12. Higher means slower but higher "processor gain",
#define SPREADING_FACTOR 8

// Transmit power in dBm. 0 dBm = 1 mW, enough for tabletop-testing. This value can be
// set anywhere between -9 dBm (0.125 mW) to 22 dBm (158 mW). Note that the maximum ERP
// (which is what your antenna maximally radiates) on the EU ISM band is 25 mW, and that
// transmissting without an antenna can damage your hardware.
#define TRANSMIT_POWER 0

byte testMessage[1] = {0xC8};

volatile bool rxFlag = false;
void rx()
{
  rxFlag = true;
}

byte rxdata[sizeof(float) * 2];

MIDI_CREATE_DEFAULT_INSTANCE();
volatile bool midiFlag = false;

unsigned long lastCheckTime = 0;
int adc_pin[NUM_POTS] = {P_FADE, P_ENTROPY, P_SPEED};

int currentADCidx = 0;
int adc_vals[NUM_POTS][AVERAGING_LEN] = {0};
byte adc_newval[NUM_POTS], adc_oldval[NUM_POTS], adc_prevTXval[NUM_POTS] = {0};
bool isChanging[NUM_POTS] = {false};

bool testMode = false;

char message[100];
int msgLength = 0;

void addToMessage(byte key, byte value)
{
  msgLength++;
  message[(msgLength - 1) * 2] = key;
  message[((msgLength - 1) * 2) + 1] = value;
}

void noteOn(byte channel, byte note, byte velocity)
{
#ifdef DEBUG
  display.printf("Note: %i, Velocity: %i\n", note, velocity);
#endif
  addToMessage(note - MIDI_OFFSET, velocity);
}

void noteOff(byte channel, byte note, byte velocity)
{
  addToMessage(note - MIDI_OFFSET, 0);
}

uint64_t last_tx = 0;
uint64_t tx_time;

void setup()
{
  heltec_setup();
  display.println("Radio init");
  RADIOLIB_OR_HALT(radio.begin());
  // Set radio parameters
  display.printf("Sync Word: %x\n", syncword);
  RADIOLIB_OR_HALT(radio.setSyncWord(syncword));
  display.printf("Frequency: %.2f MHz\n", FREQUENCY);
  RADIOLIB_OR_HALT(radio.setFrequency(FREQUENCY));
  display.printf("Bandwidth: %.1f kHz\n", BANDWIDTH);
  RADIOLIB_OR_HALT(radio.setBandwidth(BANDWIDTH));
  display.printf("Spreading Factor: %i\n", SPREADING_FACTOR);
  RADIOLIB_OR_HALT(radio.setSpreadingFactor(SPREADING_FACTOR));
  display.printf("TX power: %i dBm\n", TRANSMIT_POWER);
  RADIOLIB_OR_HALT(radio.setOutputPower(TRANSMIT_POWER));
  // Start receiving

  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_16);

  MIDI.turnThruOff();
  MIDI.setHandleNoteOn(noteOn);
  MIDI.setHandleNoteOff(noteOff);
  MIDI.begin();

  pinMode(P_FADE, INPUT);
  pinMode(P_ENTROPY, INPUT);
  pinMode(P_SPEED, INPUT);
  pinMode(P_TESTMODE, INPUT_PULLDOWN);
  //pinMode(P_SPREADFACTOR, INPUT_PULLDOWN);
}

long lastTime = 0, rt = 0, ft = 0;

void loop()
{
  heltec_loop();

  // Check if analog value has changed and passed debounce time
  if (millis() - lastCheckTime > 100)
  {
    lastCheckTime = millis();

    // Check if it's in test mode
    bool testMode = digitalRead(P_TESTMODE);

    // Check the setting Pots
    for (int i = 0; i < NUM_POTS; i++)
    {
      currentADCidx = (currentADCidx + 1) % AVERAGING_LEN;
      adc_vals[i][currentADCidx] = analogRead(adc_pin[i]);

      int result = 0;
      for (int j = 0; j < AVERAGING_LEN; j++)
      {
        result += adc_vals[i][j];
      }
      adc_newval[i] = result >> 10;
      if (adc_newval[i] != adc_oldval[i])
      {
        isChanging[i] = true;
        display.clear();
        String title;
        switch(i){
          case 0:
            title = String("Tenacity = ");
            break;
          case 1:
            title = String("Entropy = ");
            break;
          case 2:
            title = String("Variability = ");
            break;
        }
        title = String(title + String(adc_newval[i]));
        display.drawString(64, 0, title);
        display.drawProgressBar(0, 48, 128, 16, adc_newval[i]*3 + 4);
        display.display();
      }
      else
      {
        if (isChanging[i])
        {
          if (adc_newval[i] != adc_prevTXval[i])
          {
            addToMessage(100 + i, adc_newval[i]);
            adc_prevTXval[i] = adc_newval[i];
          }
          isChanging[i] = false;
        }
      }
      adc_oldval[i] = adc_newval[i];
    }

    // Send the message
    if (msgLength > 0)
    {
#ifdef DEBUG
      // display.printf("Sending: %s\n", message);
#endif
      heltec_led(10);
      RADIOLIB(radio.startTransmit(message, msgLength * 2));
      heltec_led(0);
      msgLength = 0;
      memset(message, 0, 100);
    }
  }

  if (testMode)
  {
    if (rxFlag)
    {
      rxFlag = false;
      radio.clearDio1Action();
      radio.readData(rxdata, sizeof(float) * 2);
      if (_radiolib_status == RADIOLIB_ERR_NONE)
      {
        float rmtRSSI, rmtSNR;
        memcpy(&rmtRSSI, rxdata, sizeof(float));
        memcpy(&rmtSNR, rxdata + sizeof(float), sizeof(float));
        display.printf("Remote reception:\n");
        display.printf("RSSI: %.2f | SNR: %.2f\n", rmtRSSI, rmtSNR);
      }
    }
    if (millis() - last_tx > 5000)
    {
      last_tx = millis();
      heltec_led(50);
      RADIOLIB_OR_HALT(radio.transmit(testMessage, 1));
      radio.setDio1Action(rx);
      RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
      display.printf("pinging...\n");
      heltec_led(0);
    }
  }
  else
  {
    MIDI.read();
  }
}