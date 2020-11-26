#include <SPI.h>
#include <WiFi.h>
#include <Wire.h> 
#include "max17055.h"
#include "qrcode.h"
#include "EPD_WaveShare_154D67.h"
#include <RtcDS3231.h>
#include "MiniGrafx.h"
#include "DisplayDriver.h"
#include <esp_wifi.h>
#include <esp_bt.h>
#include <bsec.h>
#include <SPIFFS.h>
#include "BigNumber.h"
#include "MD5.h"
#include "driver/adc.h"
#include "settings.h"

/* Configure the BSEC library with information about the sensor
    18v/33v = Voltage at Vdd. 1.8V or 3.3V
    3s/300s = BSEC operating mode, BSEC_SAMPLE_RATE_LP or BSEC_SAMPLE_RATE_ULP
    4d/28d = Operating age of the sensor in days
    generic_18v_3s_4d
    generic_18v_3s_28d
    generic_18v_300s_4d
    generic_18v_300s_28d
    generic_33v_3s_4d
    generic_33v_3s_28d
    generic_33v_300s_4d
    generic_33v_300s_28d
*/
const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_4d/bsec_iaq.txt"
};
#include <sys/time.h>

//#define LOG(fmt, ...) (Serial.printf("%09llu: " fmt "\n", GetTimestamp(), ##__VA_ARGS__))

Bsec sensor;

RTC_DATA_ATTR uint8_t sensor_state[BSEC_MAX_STATE_BLOB_SIZE] = {0};
RTC_DATA_ATTR int64_t sensor_state_time = 0;
RTC_DATA_ATTR uint16_t bootCounter = 0;
RTC_DATA_ATTR uint16_t accuracyCounter = 0;
RTC_DATA_ATTR uint16_t lastAccuracy = 0;

#define BME_STATE_FILE_1 "/bme1.bin"
#define BME_STATE_FILE_2 "/bme2.bin"
#define BME_STATE_FILE_3 "/bme3.bin"

bsec_virtual_sensor_t sensor_list[] = {
  BSEC_OUTPUT_RAW_TEMPERATURE,
  BSEC_OUTPUT_RAW_PRESSURE,
  BSEC_OUTPUT_RAW_HUMIDITY,
  BSEC_OUTPUT_RAW_GAS,
  BSEC_OUTPUT_IAQ,
  BSEC_OUTPUT_STATIC_IAQ,
  BSEC_OUTPUT_CO2_EQUIVALENT,
  BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
  BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
  BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
};




// 16 chars but we need 1 extra character as it's a 0-terminated string
char deviceId[17];
uint16_t palette[] = {0, 1};

EPD_WaveShare154D67 epd(EPD_CS, EPD_RST, EPD_DC, EPD_BUSY);
MiniGrafx gfx = MiniGrafx(&epd, BITS_PER_PIXEL, palette, EPD_WIDTH, EPD_HEIGHT);

RtcDS3231<TwoWire> Rtc(Wire);
MAX17055 gauge;
MAX17055::platform_data design_param;

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
void initBme();
bool CheckSensor();
int64_t GetTimestamp();
void drawAirQuality();
void drawValue(uint8_t line, String label, String value, String unit);
void storeState(String name);
void loadState(String name);

/*****************************************************************************/
/* Main processing                                                           */
/*****************************************************************************/
void setup() {
  WiFi.mode(WIFI_OFF);
  btStop();
  Serial.begin(115200);
  bootCounter++;

  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  log_d("Compile time (local system time zone): %s %s", __DATE__, __TIME__);
  log_d("Millis at start: %d\n", millis());

  initGfx();

  Wire.begin(BME_SDA, BME_SCL);
  initBme();
  drawAirQuality();

  // due to limited number of pins, use pin 0 and 2 for SDA, SCL
  Wire.begin(RTC_SDA, RTC_SCL);
  initRtc();
  initFuelGauge();


  String deviceIdString = String(deviceId);
  RtcDateTime now = Rtc.GetDateTime();
  gfx.setFont(ArialMT_Plain_16);


  //drawQrCode(text);

  uint32_t startTime = millis();
  log_d("Commit time: %d\n", millis() - startTime);
  //goToDeepSleep();
  uint64_t time_us = ((sensor.nextCall - GetTimestamp()) * 1000) - esp_timer_get_time();
  log_d("Deep sleep for %llu ms. BSEC next call at %llu ms.", time_us / 1000, sensor.nextCall);
  esp_sleep_enable_timer_wakeup(time_us + 5 * 1000 * + 1000);
  esp_deep_sleep_start();
}

void loop() {}

/*****************************************************************************/
/* Functions                                                                 */
/*****************************************************************************/
void DumpState(const char* name, const uint8_t* state) {
  log_d("%s:", name);
  for (int i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
    Serial.printf("%02x ", state[i]);
    if (i % 16 == 15) {
      Serial.print("\n");
    }
  }
  Serial.print("\n");
}

void drawValue(uint8_t line, String label, String value, String unit) {
  uint16_t y = line * 16;
  gfx.setFont(ArialMT_Plain_16);
  gfx.setTextAlignment(TEXT_ALIGN_LEFT);
  gfx.drawString(0, y, label);
  gfx.drawString(175, y, unit);
  gfx.setTextAlignment(TEXT_ALIGN_RIGHT);
  gfx.drawString(170, y, value);
}

void drawAirQuality() {
  if (sensor.run(GetTimestamp())) {
    log_d("Temperature raw %.2f compensated %.2f", sensor.rawTemperature, sensor.temperature);
    log_d("Humidity raw %.2f compensated %.2f", sensor.rawHumidity, sensor.humidity);
    log_d("Pressure %.2f kPa", sensor.pressure / 1000);
    log_d("IAQ %.0f accuracy %d", sensor.iaq, sensor.iaqAccuracy);
    log_d("Static IAQ %.0f accuracy %d", sensor.staticIaq, sensor.staticIaqAccuracy);
    log_d("Gas resistance %.2f kOhm", sensor.gasResistance / 1000);
    log_d("CO2 equivalent %.2f", sensor.co2Equivalent);
    log_d("Breath VOC equivalent %.2f", sensor.breathVocEquivalent);
    log_d("BootCounter %d", bootCounter);
    if (sensor.iaqAccuracy > lastAccuracy) {
      accuracyCounter = bootCounter;
      switch(sensor.iaqAccuracy) {
        case 1:
          storeState(String(BME_STATE_FILE_1));
          break;
        case 2:
          storeState(String(BME_STATE_FILE_2));
          break;
        case 3:
          storeState(String(BME_STATE_FILE_3));
          break;
      }
    }
    lastAccuracy = sensor.iaqAccuracy;
    log_d("Loops until acc > 0 %d", accuracyCounter);
    gfx.fillBuffer(1);
    gfx.setColor(0);
    drawValue(1, "Time", String(sensor.rawTemperature), "°C");
    drawValue(1, "Temp raw", String(sensor.rawTemperature), "°C");
    drawValue(2, "Temp comp", String(sensor.temperature), "°C");
    drawValue(3, "Pressure", String(sensor.pressure / 1000), "kPa");
    drawValue(4, "IAQ", String(sensor.iaq), "");
    drawValue(5, "Accuracy", String(sensor.iaqAccuracy), "");
    drawValue(6, "Static IAQ", String(sensor.staticIaq), "");
    drawValue(7, "Accuracy", String(sensor.staticIaqAccuracy), "");
    drawValue(8, "Gas resistance", String(sensor.gasResistance / 1000), "kOhm");
    drawValue(9, "eCO2", String(sensor.co2Equivalent), "");
    drawValue(10, "VOC", String(sensor.breathVocEquivalent), "");
    drawValue(11, "#Boots", String(bootCounter), "");
    //gfx.commit();

    sensor_state_time = GetTimestamp();
    sensor.getState(sensor_state);
    //DumpState("getState", sensor_state);
    log_d("Saved state to RTC memory at %lld", sensor_state_time);
    CheckSensor();

    uint64_t time_us = ((sensor.nextCall - GetTimestamp()) * 1000) - esp_timer_get_time();

  }
}

void storeState(String name) {
    log_d("Writing file: %s\n", name.c_str());

    File file = SPIFFS.open(name.c_str(), FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    sensor.getState(sensor_state);
    if(file.write(sensor_state, BSEC_MAX_STATE_BLOB_SIZE)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}
void loadState(String name) {
    Serial.printf("Reading file: %s\n", name.c_str());

    File file = SPIFFS.open(name.c_str());
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        file.read(sensor_state, BSEC_MAX_STATE_BLOB_SIZE);
    }
    sensor.setState(sensor_state);
    DumpState("fromFile", sensor_state);
    CheckSensor();
    file.close();
}

bool CheckSensor() {
  if (sensor.status < BSEC_OK) {
    log_d("BSEC error, status %d!", sensor.status);
    return false;;
  } else if (sensor.status > BSEC_OK) {
    log_d("BSEC warning, status %d!", sensor.status);
  }

  if (sensor.bme680Status < BME680_OK) {
    log_d("Sensor error, bme680_status %d!", sensor.bme680Status);
    return false;
  } else if (sensor.bme680Status > BME680_OK) {
    log_d("Sensor warning, status %d!", sensor.bme680Status);
  }

  return true;
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
  esp_sleep_enable_ext0_wakeup(RTC_INT, 0);

  esp_deep_sleep_start();
}

void initBme() {

  sensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
  if (!CheckSensor()) {
    log_d("Failed to init BME680, check wiring!");
    return;
  }

  log_d("BSEC version %d.%d.%d.%d", sensor.version.major, sensor.version.minor, sensor.version.major_bugfix, sensor.version.minor_bugfix);

  sensor.setConfig(bsec_config_iaq);
  if (!CheckSensor()) {
    log_d("Failed to set config!");
    return;
  }

  if (sensor_state_time) {
    DumpState("setState", sensor_state);
    sensor.setState(sensor_state);
    if (!CheckSensor()) {
      log_d("Failed to set state!");
      return;
    } else {
      log_d("Successfully set state from %lld", sensor_state_time);
    }
  } else if (SPIFFS.exists(BME_STATE_FILE_3)) {
    loadState(String(BME_STATE_FILE_3));
  } else if (SPIFFS.exists(BME_STATE_FILE_2)) {
    loadState(String(BME_STATE_FILE_2));
  } else if (SPIFFS.exists(BME_STATE_FILE_1)) {
    loadState(String(BME_STATE_FILE_1));
  } else {
    log_d("Saved state missing");
  }

  sensor.updateSubscription(sensor_list, sizeof(sensor_list) / sizeof(sensor_list[0]), BSEC_SAMPLE_RATE_LP);
  if (!CheckSensor()) {
    log_d("Failed to update subscription!");
    return;
  }

  log_d("Sensor init done");
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
