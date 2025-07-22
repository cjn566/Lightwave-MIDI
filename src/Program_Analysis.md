# LoRa Piano LED Visualizer Analysis

## Overview
This Arduino program is a **wireless piano LED visualizer** that receives MIDI-like note data over LoRa radio and displays it using a FastLED strip representing 88 piano keys.

## Key Components

### Hardware Setup
- **Microcontroller**: Heltec WiFi LoRa board
- **LED Strip**: 90 LEDs using SK9822 chipset (connected to pins 47 & 48)
- **Display**: OLED display for settings visualization
- **Radio**: LoRa radio operating at 905.2 MHz (US frequency)

### Core Functionality

#### 1. **LoRa Communication**
- Receives wireless MIDI note data from a transmitter
- Frequency: 905.2 MHz with 250 kHz bandwidth
- Uses spreading factor 8 and sync word 0x69
- Can also respond with RSSI/SNR data when requested

#### 2. **LED Visualization**
The program creates dynamic LED effects based on received piano notes:

- **90 LEDs** represent the 88 keys of a piano plus one led to either side
- **Color Mapping**: Each note gets a hue based on timing and randomness
- **Intensity**: Based on MIDI velocity (how hard the key was pressed)
- **Visual Effects**: 
  - Notes fade out over time (controlled by "tenacity")
  - Color bleeding between adjacent active notes
  - Entropy effects that spread intensity between neighboring LEDs

#### 3. **Configurable Parameters**
Three main visual parameters can be adjusted remotely:

- **Tenacity** (0-31): How quickly notes fade out
- **Entropy** (0-31): How much colors/intensity bleed between adjacent keys  
- **Chroma** (0-31): Amount of randomness in color selection

#### 4. **Power Management**
- Display turns off after 8 hours to save power
- Can be awakened by pressing lowest and highest keys simultaneously

#### 5. **LED Source Switching (Screensaver Mode)**
- Uses a multiplexer controlled by pin 7 (P_SELECT) to switch LED data sources
- After 20 seconds of inactivity, switches from "Self" mode to another microcontroller
- The other microcontroller runs a different program (acts as a screensaver)
- Returns to "Self" mode when LoRa activity is detected

## Program Flow

1. **Setup**: Initialize LoRa radio, LED strip, and display
2. **Main Loop**:
   - Process incoming LoRa messages (notes or settings)
   - Update LED colors and intensities with visual effects
   - Handle power management and display updates
   - Refresh LEDs at 30 FPS

## Use Case
This appears to be designed for **wireless piano visualization performances** where:
- A pianist plays on a MIDI keyboard with LoRa transmitter
- This receiver creates a real-time LED light show on an 88-LED strip
- The visual effects can be adjusted remotely during performance
- Multiple receivers could create synchronized light shows

## Technical Details
- **Frame Rate**: 30 FPS LED updates
- **Note Timeout**: 8 seconds (prevents stuck notes)
- **Idle Timeout**: 20 seconds before switching modes
- **Color Cycle**: 12-second base timing for hue effects
- **Radio Range**: Typical LoRa range (hundreds of meters to kilometers depending on environment)

This is a sophisticated real-time audio-visual system that bridges traditional piano performance with modern LED art installations.
