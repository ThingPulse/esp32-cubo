/*
MIT License

Copyright (c) 2020 ThingPulse GmbH

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
#include <WiFi.h>
#include <Wire.h> 
#include "max17055.h"
#include "EPD_WaveShare_154D67.h"
#include <RtcDS3231.h>
#include "MiniGrafx.h"
#include "DisplayDriver.h"
#include <esp_wifi.h>
#include <esp_bt.h>
#include "driver/adc.h"
#include "settings.h"
#include "ADXL345.h"

#include <sys/time.h>

//#define LOG(fmt, ...) (Serial.printf("%09llu: " fmt "\n", GetTimestamp(), ##__VA_ARGS__))


uint16_t palette[] = {0, 1};

EPD_WaveShare154D67 epd(EPD_CS, EPD_RST, EPD_DC, EPD_BUSY);
MiniGrafx gfx = MiniGrafx(&epd, BITS_PER_PIXEL, palette, EPD_WIDTH, EPD_HEIGHT);

RtcDS3231<TwoWire> Rtc(Wire);
MAX17055 gauge;
MAX17055::platform_data design_param;
ADXL345 accelerometer;

/*****************************************************************************/
/* Function declarat                                                         */
/*****************************************************************************/

String convertToDateTimeString(const RtcDateTime &dt);
void drawDebugData();
void goToDeepSleep();
void initDeviceId();
void initFuelGauge();
void initGfx();
void initRtc();
void initIMU();
uint8_t getRotation(uint8_t sampleCount);
void drawScreen();

int64_t GetTimestamp();

void drawValue(uint8_t line, String label, String value, String unit);


/*****************************************************************************/
/* Main processing                                                           */
/*****************************************************************************/
void setup() {
  WiFi.mode(WIFI_OFF);
  btStop();
  Serial.begin(115200);

  log_d("Compile time (local system time zone): %s %s", __DATE__, __TIME__);
  log_d("Millis at start: %d\n", millis());

  initGfx();
  initIMU();
  uint8_t rotation = getRotation(5);
  log_d("Current rotation: %d", rotation);

  // due to limited number of pins, use pin 0 and 2 for SDA, SCL
  Wire.begin(RTC_SDA, RTC_SCL);
  initRtc();
  initFuelGauge();

  RtcDateTime now = Rtc.GetDateTime();
  gfx.setFont(ArialMT_Plain_16);

  drawScreen();

  uint32_t startTime = millis();
  log_d("Commit time: %d\n", millis() - startTime);
  goToDeepSleep();
}

void loop() {}

/*****************************************************************************/
/* Functions                                                                 */
/*****************************************************************************/

void drawScreen() {
  gfx.fillBuffer(MINI_WHITE);
  gfx.setColor(MINI_BLACK);
  gfx.drawString(10, 10, "Hello world");
  gfx.commit();
}

int64_t GetTimestamp() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
}

String convertToDateTimeString(const RtcDateTime &dt) {
  char dateString[22];

  snprintf_P(dateString, countof(dateString), PSTR("%04u-%02u-%02uT%02u:%02u:%02uZ"),
             dt.Year(),
             dt.Month(),
             dt.Day(),
             dt.Hour(),
             dt.Minute(),
             dt.Second());
  return String(dateString);
}

void goToDeepSleep() {
  log_d("Going to sleep...");

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();
  adc_power_off();
  epd.Sleep();

  uint8_t mode = OUTPUT;
  pinMode(EPD_CS, mode);
  pinMode(EPD_DC, mode);
  pinMode(EPD_BUSY, mode);
  pinMode(EPD_DC, mode);
  pinMode(EPD_CLK, mode);
  pinMode(EPD_DIN, mode);

  pinMode(GAUGE_SDA, mode);
  pinMode(GAUGE_SCL, mode);

  pinMode(USR_LED_1, mode);
  pinMode(USR_LED_2, mode);

  log_d("Time before sleep: %d\n", millis());
  //esp_sleep_enable_ext0_wakeup(RTC_INT, 0);

  esp_deep_sleep_start();
}

void initIMU() {
  if (!accelerometer.begin(IMU_SDA, IMU_SCL))
  {
    Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
    delay(500);
  }


  // Set tap detection on Z-Axis
  accelerometer.setTapDetectionX(1);       // Don't check tap on X-Axis
  accelerometer.setTapDetectionY(1);       // Don't check tap on Y-Axis
  accelerometer.setTapDetectionZ(1);       // Check tap on Z-Axis
  // or
  // accelerometer.setTapDetectionXYZ(1);  // Check tap on X,Y,Z-Axis

  accelerometer.setTapThreshold(2.5);      // Recommended 2.5 g
  accelerometer.setTapDuration(0.02);      // Recommended 0.02 s
  accelerometer.setDoubleTapLatency(0.10); // Recommended 0.10 s
  accelerometer.setDoubleTapWindow(0.30);  // Recommended 0.30 s

  accelerometer.setActivityThreshold(2.0);    // Recommended 2 g
  accelerometer.setInactivityThreshold(2.0);  // Recommended 2 g
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
  //pinMode(IMU_INT, INPUT);
  esp_sleep_enable_ext0_wakeup(IMU_INT, 1);
}

uint8_t getRotation(uint8_t sampleCount) {
  Vector norm = accelerometer.readNormalize();
  // We need to read activities to flush FIFO buffer
  Activites activites = accelerometer.readActivites();
  uint8_t rotation = 0;
  uint8_t unchangedRotationCount = 0;
  uint8_t currentRotation = 0;
  while (true) {
    if (norm.ZAxis > 8) {
      return 4;
    } else if (norm.XAxis > 8) {
      currentRotation = 1;
    } else if (norm.XAxis < -8) {
      currentRotation = 3;
    } else if (norm.YAxis > 8) {
      currentRotation = 2;
    } else if (norm.YAxis < -8) {
      currentRotation = 0;
    } else {
      currentRotation =  3;
    }
    if (rotation == currentRotation) {
      unchangedRotationCount++;
    } else {
      rotation = currentRotation;
      unchangedRotationCount = 0;
    }

    if (unchangedRotationCount > sampleCount) {
      return rotation;
    }
    delay (100);
  }
  return 0;
}


void initFuelGauge() {
  // 2500mAH, https://pdfserv.maximintegrated.com/en/an/AN6358.pdf, 1.9.1.2
  design_param.designcap  = 0x9C4;  
  // 20mA charge termination current, https://pdfserv.maximintegrated.com/en/an/AN6358.pdf, 1.9.1.4
  design_param.ichgterm  = 0x0020; 
  // 3.1V vEmpty (9 bit), 3.88 vRecovery (7 bit), https://pdfserv.maximintegrated.com/en/an/AN6358.pdf, 1.9.1.1
  design_param.vempty  = 0x9B61; 
  design_param.vcharge  = 4200;  
  // rSense resistor, 10mOhms
  design_param.rsense = 10; 

  MAX17055::Registers_e array_addresses[7] = {MAX17055::MODELCFG_REG,
      MAX17055::DESIGNCAP_REG,
      MAX17055::FULLCAPNOM_REG,
      MAX17055::DPACC_REG,
      MAX17055::DQACC_REG,
      MAX17055::VEMPTY_REG,
      MAX17055::ICHGTERM_REG};

  uint16_t regValue[7];
  for(int i = 0 ; i < 7 ; i++) {
    regValue[i] = gauge.get_regInfo(array_addresses[i]);
    log_d("REg: %X, value: %X ", array_addresses[i], regValue[i]);
  }

  log_v("Percentage: %d", gauge.get_SOC());
}

void initGfx() {
  gfx.init();
  gfx.setRotation(1);
  gfx.setFastRefresh(false);
  gfx.fillBuffer(MINI_WHITE);
  gfx.setColor(MINI_BLACK);
  gfx.setFont(ArialMT_Plain_10);
}

void initRtc() {
  Rtc.Begin();
  RtcDateTime compileTimeUtc = RtcDateTime(__DATE__, __TIME__) + COMPILE_TIME_OFFSET_SECONDS - UTC_OFFSET_SECONDS;
  log_d("Compile time in UTC: %s", convertToDateTimeString(compileTimeUtc).c_str());

  if (!Rtc.IsDateTimeValid()) {
    if (Rtc.LastError() != 0) {
      // we have a communications error
      // see https://www.arduino.cc/en/Reference/WireEndTransmission for
      // what the number means
      log_d("RTC communications error = %d", Rtc.LastError());
    } else {
      // Common Causes:
      //    1) first time you ran and the device wasn't running yet
      //    2) the battery on the device is low or even missing
      log_d("RTC lost confidence in the DateTime!");
      // following line sets the RTC to the date & time this sketch was compiled
      // it will also reset the valid flag internally unless the Rtc device is
      // having an issue
      Rtc.SetDateTime(compileTimeUtc);
    }
  }

  if (!Rtc.GetIsRunning()) {
    log_d("RTC was not actively running, starting now");
    Rtc.SetIsRunning(true);
  }

  RtcDateTime now = Rtc.GetDateTime();
  if (now < compileTimeUtc) {
    log_d("RTC is older than compile time. Updating date and time.");
    Rtc.SetDateTime(compileTimeUtc);
  } else if (now > compileTimeUtc) {
    log_d("RTC is newer than compile time; this is expected.");
  } else if (now == compileTimeUtc) {
    log_d("RTC is the same as compile time! This is not expected but all is fine.");
  }

  // Never assume the RTC was last configured by you, so just clear them to your needed state.
  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmOne);

  // Set the RTC alarm to the next interval sync time regardless of the current time (i.e. rather
  // than just rolling forward).
  // 1. Calculate the number of (past) full interval cycles
  // 2. Multiply with interval duration
  // 3. Add one interval
  RtcDateTime alarmTime = ((int)(now / WAKEUP_INTERVAL_SECONDS)) * WAKEUP_INTERVAL_SECONDS + WAKEUP_INTERVAL_SECONDS;
  DS3231AlarmOne alarm1(
      alarmTime.Day(),
      alarmTime.Hour(),
      alarmTime.Minute(),
      alarmTime.Second(),
      DS3231AlarmOneControl_HoursMinutesSecondsMatch);
  Rtc.SetAlarmOne(alarm1);

  Rtc.LatchAlarmsTriggeredFlags();
}
