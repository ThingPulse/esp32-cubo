/**
The MIT License (MIT)
Copyright (c) 2017 by ThingPulse GmbH, Daniel Eichhorn
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#include <SPI.h>
#include "ADXL345.h"
#include "EPD_WaveShare_154D67.h"
#include "MiniGrafx.h"
#include "DisplayDriver.h"
#include "artwork.h"
#include "settings.h"

#define MINI_BLACK 0
#define MINI_WHITE 1

#define BITS_PER_PIXEL 1

#define sleepSeconds 60

extern const char bonjour01Start[] asm("_binary_images_1bit_Bonjour_01_bmp_start");

RTC_DATA_ATTR uint8_t rotationBeforeSleep = -1;


uint16_t palette[] = {0, 1};
boolean isFastRefreshEnabled = false;

EPD_WaveShare154D67 epd(EPD_CS, EPD_RST, EPD_DC, EPD_BUSY);
MiniGrafx screenGfx = MiniGrafx(&epd, BITS_PER_PIXEL, palette, EPD_WIDTH, EPD_HEIGHT);

uint8_t counter = 0;
uint8_t rotation = 0;
uint8_t mode = 0;
boolean modeChanged = true;

#define REPETITIONS 10
#define MODES 4

ADXL345 accelerometer;

void wakeup() {

}

uint8_t getRotation() {
    uint8_t sampleCount = 10;
    Vector norm = accelerometer.readNormalize();
    Serial.printf("x: %d, y: %d, z: %d\n", norm.XAxis, norm.YAxis, norm.ZAxis);
    // We need to read activities to flush FIFO buffer
    Activites activites = accelerometer.readActivites();
    uint8_t rotation = 0;
    uint8_t unchangedRotationCount = 0;
    uint8_t currentRotation = 0;
    while (true) {
      if (norm.ZAxis > 8) {
        return 4;
      } else if (norm.ZAxis < -8) {
        return 5;
      } else if (norm.XAxis > 8) {
        currentRotation = 1;
      } else if (norm.XAxis < -8) {
        currentRotation = 3;
      } else if (norm.YAxis > 8) {
        currentRotation = 2;
      } else if (norm.YAxis < -8) {
        currentRotation = 0;
      } else {
        currentRotation = 6;
      }
      if (rotation == currentRotation) {
        unchangedRotationCount++;
      } else {
        rotation = currentRotation;
        unchangedRotationCount = 0;
      }

      if (unchangedRotationCount > sampleCount) {
        Serial.printf("Rotation: %d\n", rotation);
        return rotation;
      }
      delay (100);
    }
    return 0;

}

void initIMU() {
    if (!accelerometer.begin(IMU_SDA, IMU_SCL))
    {
      Serial.println("Could not find a valid 27 sensor, check wiring!");
      delay(500);
    }

      // Set tap detection on Z-Axis
    accelerometer.setTapDetectionX(0);       // Don't check tap on X-Axis
    accelerometer.setTapDetectionY(0);       // Don't check tap on Y-Axis
    accelerometer.setTapDetectionZ(0);       // Check tap on Z-Axis
    // or
    // accelerometer.setTapDetectionXYZ(1);  // Check tap on X,Y,Z-Axis

    accelerometer.setTapThreshold(2.5);      // Recommended 2.5 g
    accelerometer.setTapDuration(0.02);      // Recommended 0.02 s
    accelerometer.setDoubleTapLatency(0.10); // Recommended 0.10 s
    accelerometer.setDoubleTapWindow(0.30);  // Recommended 0.30 s

    accelerometer.setActivityThreshold(1.1);    // Recommended 2 g
    accelerometer.setInactivityThreshold(1.5);  // Recommended 2 g
    accelerometer.setTimeInactivity(5);         // Recommended 5 s

    // Set activity detection only on X,Y,Z-Axis
    //accelerometer.setActivityXYZ(1);         // Check activity on X,Y,Z-Axis
    // or
    accelerometer.setActivityX(1);        // Check activity on X_Axis
    accelerometer.setActivityY(1);        // Check activity on Y-Axis
    accelerometer.setActivityZ(0);        // Check activity on Z-Axis

    // Set inactivity detection only on X,Y,Z-Axis
    //accelerometer.setInactivityXYZ(1);       // Check inactivity on X,Y,Z-Axis

    // Select INT 1 for get activities
    accelerometer.useInterrupt(ADXL345_INT1);
    pinMode(IMU_INT, INPUT);

    esp_sleep_enable_ext0_wakeup(IMU_INT,1);
    attachInterrupt(digitalPinToInterrupt(IMU_INT), wakeup, CHANGE);
}

void drawScreen() {
    uint8_t rotation = getRotation();
    // Only refresh screen, if rotation has changed
    if (rotation != rotationBeforeSleep) {
      screenGfx.fillBuffer(MINI_WHITE);
      screenGfx.setColor(MINI_BLACK);
      //screenGfx.drawString(100, 100, String(getRotation()));
      switch (rotation) {
        case 0:
          screenGfx.setRotation(3);
          screenGfx.drawPalettedBitmapFromPgm(0, 0, date02);
          
          break;
        case 1:
          screenGfx.setRotation(0);
          screenGfx.drawPalettedBitmapFromPgm(0, 0, weather);
          
          break;
        case 2:
          screenGfx.setRotation(1);
          screenGfx.drawPalettedBitmapFromPgm(0, 0, spruch01);
          
          break;
        case 3:
          screenGfx.setRotation(2);
          screenGfx.drawPalettedBitmapFromPgm(0, 0, kippen);
          break;
        case 4:
          screenGfx.setRotation(1);
          screenGfx.drawPalettedBitmapFromPgm(0, 0, willkommen);
          break;
        case 5:
          screenGfx.setRotation(1);
          screenGfx.drawPalettedBitmapFromPgm(0, 0, bonjour01Start);
          break;
        default:
          screenGfx.setRotation(1);
          screenGfx.drawPalettedBitmapFromPgm(0, 0, bonjour01Start);
          break;
      }
      screenGfx.setColor(MINI_BLACK);

      screenGfx.commit();
      rotationBeforeSleep = rotation;
    }
}


void setup() {
  Serial.begin(115200);

  Serial.printf("Rotation before last sleep: %d\n", rotationBeforeSleep);

  screenGfx.init();
  screenGfx.fillBuffer(MINI_WHITE);
  screenGfx.setColor(MINI_BLACK);
  screenGfx.drawString(100, 100, String(millis()));

  initIMU();

  drawScreen();

  Serial.printf_P(PSTR("Going to sleep for: %d[s]\n"),  sleepSeconds);
  esp_sleep_enable_timer_wakeup(sleepSeconds * 1000000);
  esp_deep_sleep_start();

}

void loop() {

}
