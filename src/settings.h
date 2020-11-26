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

#define BASE_URL "https://check.tempo-tag.com/"
#define URI_VERSION 1

// Does not just define the absolute *interval* but actually every nth interval of time. It behaves
// like */n in a cron expression. Example: 900s -> 15min => */15 => every 15th minute of every hour.
#define WAKEUP_INTERVAL_SECONDS 60
// Compensate for the seconds it takes to compile and upload the binary. Depends on the specific 
// machine the operation is executed on.
#define COMPILE_TIME_OFFSET_SECONDS 19
// The whole application uses UTC exclusively. The compile time, however, uses local system time.
#define UTC_OFFSET_SECONDS 7200

#define MINI_BLACK 0
#define MINI_WHITE 1

#define EPD_CS 4 
#define EPD_RST 14 
#define EPD_DC 12   
#define EPD_BUSY 27 
#define EDP_RES 14
#define EPD_DC 12
#define EPD_CLK 18
#define EPD_DIN 23

#define GAUGE_SDA 33
#define GAUGE_SCL 22

#define IMU_INT GPIO_NUM_36
#define IMU_SDA 15
#define IMU_SCL 13

#define RTC_SDA 33
#define RTC_SCL 22
#define RTC_INT GPIO_NUM_39

#define BME_SCL 21
#define BME_SDA 26

#define USR_LED_1 32
#define USR_LED_2 25

#define BITS_PER_PIXEL 1

#define countof(a) (sizeof(a) / sizeof(a[0]))
