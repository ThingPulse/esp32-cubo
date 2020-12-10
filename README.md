# ESP32-Cubo

The ESP32-Cubo is a device based on the Espressif ESP32 and comes with the following hardware:
- ESP32 Wrover-B Microcontroller module with 4MB Flash and 8MB SPIRAM memory
- Good Display 1.54" epaper display with 200x200 Black and White Pixels
- Battery charger and connector for LiPo battery
- USB connector for programming and charging
- CP2104 USB-to-Serial Chip
- 2x User controllable LEDs, red and green
- Inertial Measurement Unit ADXL345
- MAX17055 Fuel Gauge Chip for measuring battery voltage and estimating remaining capacity
- DS3231 Real Time Clock Chip for keeping time, powered by coin cell battery
- Reset Button

![Visual Studio Code Footer](/images/cubo-hardware.jpg)

## Template for building a paper mockup
[Download paper template](/images/Cubo_Vorlage.pdf)

## Prerequisites
- esp32-cubo hardware
- usb cable to connect your computer to the device. 
- git installed on your command line
- experience in programming the ESP32 by using the Arduino platform


## Install the CP2102 driver

This driver is required to programm the esp32-cubo. It enables your computer to talk to the USB-to-Serial chip on the PCB.
To install the driver please follow the instructions under https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers

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


## Components Explained

### The ePaper Display

The 1.54" ePaper display has 200x200 black and white pixels. The display can maintain the state of the pixels even without
supporting power. This makes is a great tool for devices running from batteries since it only consumes energy during updates.

In this sample we use the ThingPulse Minigrafx library to control the display. You can find more information for programming
the display here: https://github.com/ThingPulse/minigrafx/blob/master/API.md

### The ADXL345 IMU

The ADXL345 chip can measure movement/accelereation in three axes. It can wakeup the main processor from deep sleep
when movement is detected. It can also be setup to detect taps  or double taps in one of the three axes. This repository
contains a slightly modified copy of a ADXL345 driver by Korneliusz Jarzebski

## Uploading Filesystem

In order to have the files under data/ available they need to be uploaded to the device first. Platformio creates a .bin file
which is then flashed to the device. Everytime you change the content under data/ this needs to be done. To this execute the following
task: env:esp-wrover-kit > Platform > Upload Filesystem Image

![File Upload](/images/Fileupload.png)

## Upload Code

To compile the code you can click on the checkmark icon in VS Code's footer. The arrow to the right will do the same and upload
the binary to the device. To plug icon turns on the serial console to monitor what is happening on the device.

![Visual Studio Code Footer](/images/Footer.png)