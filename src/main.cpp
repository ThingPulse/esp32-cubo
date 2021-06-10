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
#include <JPEGDecoder.h>
#include "ADXL345.h"
#include "EPD_WaveShare_154D67.h"
#include "MiniGrafx.h"
#include "DisplayDriver.h"
#include "artwork.h"
#include "settings.h"
#include "upng.h"
#include "FS.h"
#include "SPIFFS.h"

#define MINI_BLACK 0
#define MINI_WHITE 1

#define BITS_PER_PIXEL 1

#define sleepSeconds 600

extern const char willkommen[] asm("_binary_data_willkommen_hack_start");



RTC_DATA_ATTR uint8_t rotationBeforeSleep = -1;


uint16_t palette[] = {0, 1};
boolean isFastRefreshEnabled = false;

EPD_WaveShare154D67 epd(EPD_CS, EPD_RST, EPD_DC, EPD_BUSY);
MiniGrafx screenGfx = MiniGrafx(&epd, BITS_PER_PIXEL, palette, EPD_WIDTH, EPD_HEIGHT);

upng_t* upng;
uint16_t width, height, depth, iconWidth;
#define FORMAT_SPIFFS_IF_FAILED true

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
      delay (50);
    }
    return 0;

}

void closeFile() {

}

void drawJpeg(String filename, int xpos, int ypos) {
  char buffer[filename.length() + 1];
  filename.toCharArray(buffer, filename.length() + 1);
  
  JpegDec.decodeFile(buffer);
  uint16_t  *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  Serial.printf("MCU W/H: %d, %d\n", mcu_w, mcu_h);
  // uint32_t mcu_pixels = mcu_w * mcu_h; // total pixels
  // TODO immplmenet something to track drawtime performance
  // uint32_t drawTime = millis();

  while( JpegDec.read()){
    
    pImg = JpegDec.pImage;
    int mcu_x = (JpegDec.MCUx * mcu_w) + xpos;
    int mcu_y = (JpegDec.MCUy * mcu_h) + ypos;

      
    for (uint8_t y = 0; y < mcu_h; y++) {
      for (uint8_t x = 0; x < mcu_w; x++) {

          
          int absX = mcu_x + x;
          int absY = mcu_y + y;
          if (absX >= 0 && absX < screenGfx.getWidth() && absY >= 0 && absY < screenGfx.getHeight()) {
            // Threshold is at 50% grey to either display black or white pixel
            uint8_t colorIdx = *pImg < 32767 ? 0 : 1;
            screenGfx.setColor(colorIdx);
            //Serial.println(colorIdx);
            screenGfx.setPixel(absX, absY);

          }
          //Serial.printf("x: %d, y: %d, color: %d\n", absX, absY, pImg);
        *pImg++;
      }
    }
    
    yield();

  }

}

void drawPng(String filename, uint8_t xk, uint8_t yk) {
  upng = upng_new_from_file(filename.c_str());
  if (upng != NULL) {
    upng_decode(upng);
    upng_error errorCode = upng_get_error(upng);
    if (errorCode == UPNG_EOK) {
      width = upng_get_width(upng);
      height = upng_get_height(upng);
      depth = upng_get_bpp(upng) / 8;
      iconWidth = width / 8;
      log_d("Opened file %s", filename.c_str());
      log_d("Width: %d, height: %d, bit depth: %d", width, height, depth);
    } else {
      log_d("Failed to open file: %d", errorCode);
      return;
    }
  } else {
      log_i("Failed to load file");
      return;
  }

  log_d("Drawing %d, %d", xk, yk);
  uint16_t d, xOffset, yOffset;
  uint16_t greyLevel;


  if (upng_get_error(upng) == UPNG_EOK) {
    xOffset = xk * 8;
    yOffset = yk * 8;
    if (upng_get_format(upng) == UPNG_RGB8 || upng_get_format(upng) == UPNG_RGBA8) {
      for (uint8_t y = 0; y < height; y++) {
        for (uint8_t x = 0; x < width; x++) {
          greyLevel = 0;
          for (d = 0; d != depth; ++d) {
            greyLevel += upng_get_buffer(upng)[(y - yOffset - 1) * width * depth + (x + xOffset) * depth + (depth - d - 1)];
          }
          greyLevel = greyLevel / 3;
          screenGfx.setColor(greyLevel > 128 ? MINI_WHITE : MINI_BLACK);
          screenGfx.setPixel(x,y);
        }
      }
      log_d("Freeing resources");
    } else {
      log_d("Wrong bit depth");
    }
  }
  upng_free(upng);
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
      switch (rotation) {
        case 0:
          screenGfx.setRotation(3);
          drawJpeg("/Cubo_DPS_01.jpg", 0, 0);
          break;
        case 1:
          screenGfx.setRotation(0);
          drawJpeg("/Cubo_DPS_02.jpg", 0, 0);
          break;
        case 2:
          screenGfx.setRotation(1);
          drawJpeg("/Cubo_DPS_03.jpg", 0, 0);
          break;
        case 3:
          screenGfx.setRotation(2);
          drawJpeg("/Cubo_DPS_04.jpg", 0, 0);
          break;
        case 4:
          screenGfx.setRotation(1);
          drawJpeg("/Cubo_DPS_05.jpg", 0, 0);
          break;
        case 5:
          screenGfx.setRotation(1);
          drawJpeg("/Cubo_DPS_06.jpg", 0, 0);
          break;
        default:
          screenGfx.setRotation(1);
          drawJpeg("/Cubo_DPS_01.jpg", 0, 0);
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

  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
        Serial.println("SPIFFS Mount Failed");
        return;
  }

  screenGfx.init();
  screenGfx.setFastRefresh(false);
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
