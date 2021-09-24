
// Dual alphanum 14-segment display
// scrolling time, day, date, and temperature.

#include <Wire.h>
#include <Adafruit_DotStar.h>
#include "Adafruit_LEDBackpack.h"
#include <TimeLib.h>
#include "RTClib.h"
#include "Adafruit_MCP9808.h"

#define DOTSTAR_DATA_PIN 7
#define DOTSTAR_CLOCK_PIN 8
#define DOTSTAR_BRIGHTNESS 32

Adafruit_AlphaNum4 alpha4Left = Adafruit_AlphaNum4();
Adafruit_AlphaNum4 alpha4Right = Adafruit_AlphaNum4();
Adafruit_DotStar dotstar = Adafruit_DotStar(1, DOTSTAR_DATA_PIN, DOTSTAR_CLOCK_PIN, DOTSTAR_BGR);

RTC_PCF8523 rtc;

Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

char digit[] = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};

const char* message = "Jackson & Julie Together Again    ";
const char* days[] = {"SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY"};
const int brightness[] = { 2, 2, 2, 2, 2, 2, 4, 6, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 8, 6, 4 };

void flashDotStar(uint32_t color, int count)
{
  for (int x = 0; x < count; x++)
  {
    dotstar.setPixelColor(0, color);
    dotstar.show();
    delay(50);

    dotstar.setPixelColor(0, 0);
    dotstar.show();
    delay(50);
  }
}

uint32_t Wheel(byte WheelPos)
{
  if (WheelPos < 85)
  {
    return dotstar.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
  else if (WheelPos < 170)
  {
    WheelPos -= 85;
    return dotstar.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  else
  {
    WheelPos -= 170;
    return dotstar.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

void error(int pos)
{
  flashDotStar(Wheel(pos), 5);
  abort();
}

void initSerial()
{
  Serial.begin(9600);
  delay(2000); // Wait 2 seconds to establish thes connection before printing.
}

void initAlphaDisplay()
{
  alpha4Left.begin(0x70);
  alpha4Left.setBrightness(15);
  Serial.println("Left quad alphanumeric LED initialized.");
  alpha4Right.begin(0x71);
  alpha4Right.setBrightness(15);
  Serial.println("Right quad alphanumeric LED initialized.");
}

void initDotStar()
{
  dotstar.begin();
  dotstar.show();
  dotstar.setBrightness(DOTSTAR_BRIGHTNESS);
  Serial.println("Trinket DotStar initialized.");
}

void initRTC()
{
  if (rtc.begin() == false) error(80);
  if (rtc.initialized()) Serial.print("PCF8523 RTC initialized.");
  if (rtc.lostPower()) Serial.print("PCF8523 RTC lost power.");

  if (! rtc.initialized() || rtc.lostPower())
  {
    Serial.println("RTC is NOT initialized, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    //
    // Note: allow 2 seconds after inserting battery or applying external power
    // without battery before calling adjust(). This gives the PCF8523's
    // crystal oscillator time to stabilize. If you call adjust() very quickly
    // after the RTC is powered, lostPower() may still return true.
  }

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

  // When the RTC was stopped and stays connected to the battery, it has
  // to be restarted by clearing the STOP bit. Let's do this to ensure
  // the RTC is running.
  rtc.start();

  Serial.println("RTC is running.");
}

void initTempSensor()
{
  // Make sure the sensor is found, you can also pass in a different i2c
  // address with tempsensor.begin(0x19) for example, also can be left in blank for default address use
  // Also there is a table with all addres possible for this sensor, you can connect multiple sensors

  if (!tempsensor.begin(0x18))
  {
    Serial.println("MCP9808 high-accuracy temperature sensor is NOT available.");
    while (1);
  }

  Serial.println("MCP9808 high-accuracy temperature sensor is available.");

  tempsensor.setResolution(3);

  // sets the resolution mode of reading, the modes are defined in the table below:
  // Mode Resolution SampleTime
  //  0    0.5°C       30 ms
  //  1    0.25°C      65 ms
  //  2    0.125°C     130 ms
  //  3    0.0625°C    250 ms
}


void setup()
{
  initSerial();
  initAlphaDisplay();
  initDotStar();
  initTempSensor();
  initRTC();
  testDisplay();
  //displayCharacterSet();
}

void push(char c)
{
  digit[0] = digit[1];
  digit[1] = digit[2];
  digit[2] = digit[3];
  digit[3] = digit[4];
  digit[4] = digit[5];
  digit[5] = digit[6];
  digit[6] = digit[7];
  digit[7] = c;

  for (int i = 0; i < 4; i++)
  {
    alpha4Left.writeDigitAscii(i, digit[i]);
    alpha4Right.writeDigitAscii(i, digit[i + 4]);
  }

  alpha4Left.writeDisplay();
  alpha4Right.writeDisplay();
  delay(130);
}

uint8_t previousState = -1;

void setBrightness(int i)
{
  alpha4Left.setBrightness(i);
  alpha4Right.setBrightness(i);
}

void loop()
{
  DateTime now = rtc.now();

  uint8_t state = now.second() % 20;

  if (state != previousState)
  {
    Serial.println(now.timestamp());

    setBrightness(brightness[now.hour()]);

    switch (state)
    {
      case 0:
        displayTime(now);
        break;
      case 9:
        //clearDisplay();
        break;
      case 10:
        displayDate(now);
        break;
      case 14:
        //clearDisplay();
        break;
      case 15:
        displayTemperature();
        break;
      case 19:
        //clearDisplay();
        break;
    }

    uint32_t color = Wheel(now.second() * 4.2);
    color = dotstar.gamma32(color);
    dotstar.setPixelColor(0, color);
    dotstar.show();

    previousState = state;
  }

  delay(100);
}

void clearDisplay()
{
  push(' ');
  push(' ');
  push(' ');
  push(' ');
}

void displayTime(DateTime now)
{
  char format[] = "hh mm AP";
  now.toString(format);

  if (format[0] == 48) format[0] = 32;

  displayMessage(format);
}


void displayDate(DateTime now)
{
  char format[] = "MM/DD/YY";

  push(' ');
  displayMessage(days[now.dayOfTheWeek()]);
  push(' ');
  displayMessage(now.toString(format));
}

void displayTemperature()
{
  tempsensor.wake();
  delay(40);
  float f = tempsensor.readTempF();
  tempsensor.shutdown_wake(1);

  int ff = (int) f;

  push(' ');
  push(' ');
  push(ff / 10 + 48);
  push(ff % 10 + 48);
  push(27);
  push(' ');
  push('F');
  push(' ');
}

void displayMessage(const char* message)
{
  int i = 0;
  while (message[i] != 0)
  {
    push(message[i]);
    i++;
  }
}

uint16_t testValues[] =
{
  0x1, 0x2, 0x4, 0x8, 0x10, 0x20,
  0x200, 0x400, 0x80, 0x2000, 0x1000, 0x800, 0x40, 0x100,
  0x1, 0x3, 0x7, 0xF, 0x1F, 0x3F,
  0x23F, 0x63F, 0x6BF, 0x26BF, 0x36BF, 0x3EBF, 0x3EFF, 0x3FFF
};

void testDisplay()
{
  for (int i = 0; i < 140; i++)
  {
    uint16_t v = testValues[i % 28];
    alpha4Left.writeDigitRaw(0, v);
    alpha4Left.writeDigitRaw(1, v);
    alpha4Left.writeDigitRaw(2, v);
    alpha4Left.writeDigitRaw(3, v);
    alpha4Left.writeDisplay();
    alpha4Right.writeDigitRaw(0, v);
    alpha4Right.writeDigitRaw(1, v);
    alpha4Right.writeDigitRaw(2, v);
    alpha4Right.writeDigitRaw(3, v);
    alpha4Right.writeDisplay();
    delay(25);
  }
  alpha4Left.blinkRate(HT16K33_BLINK_2HZ);
  alpha4Right.blinkRate(HT16K33_BLINK_2HZ);
  delay(1000);
  alpha4Left.blinkRate(HT16K33_BLINK_OFF);
  alpha4Right.blinkRate(HT16K33_BLINK_OFF);
}

void displayCharacterSet()
{
  for (char c = 0; c < 128; c++)
  {
    push(c / 100 + 48);
    push(c / 10 + 48);
    push(c % 10 + 48);
    push(c);
    delay(1000);
  }
}
