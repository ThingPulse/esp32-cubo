# ESP32-Cubo

The ESP32-Cubo is a device based on the Espressif ESP32 and comes with the following hardware:
- ESP32 Wrover-B Microcontroller module with 4MB Flash and 8MB SPIRAM memory
- Good Display 1.54" epaper display with 200x200 Black and White Pixels
- Battery charger and connector for LiPo battery
- USB connector for programming and charging
- CP2104 USB-to-Serial Chip
- 2x User controllable LEDs, red and green
- MAX17055 Fuel Gauge Chip for measuring battery voltage and estimating remaining capacity
- DS3231 Real Time Clock Chip for keeping time, powered by coin cell battery
- Reset Button

## Prerequisites
- git installed on your command line


## Setup Development environment

This sample code should in theory work with the Arduino IDE. Instead we suggest to use the Platformio IDE since it makes it
much easier to handle the correct libraries.

Platform IO IDE is a plugin for Microsoft's Visual Studio Code Editor. To install the Platformio IDE please follow
these instructions: https://platformio.org/install/ide?install=vscode

## Get the Code

To get the sample code run 

```
git clone https://github.com/thingpulse/esp32-cubo
```
from your command line or download the zip file from https://github.com/ThingPulse/esp32-cubo/archive/master.zip and unpack it. 