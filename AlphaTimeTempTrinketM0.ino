// Demo the quad alphanumeric display LED backpack kit
// scrolls through every character, then scrolls Serial
// input onto the display

#include <Wire.h>
#include <Adafruit_DotStar.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <TimeLib.h>
#include "RTClib.h"
#include "Adafruit_MCP9808.h"

#define DOTSTAR_DATA_PIN 7
#define DOTSTAR_CLOCK_PIN 8
#define DOTSTAR_BRIGHTNESS 32

Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();
Adafruit_DotStar dotstar = Adafruit_DotStar(1, DOTSTAR_DATA_PIN, DOTSTAR_CLOCK_PIN, DOTSTAR_BGR);

RTC_PCF8523 rtc;

Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

char digit[] = {' ', ' ', ' ', ' ' };
char* message = "Jackson & Julie Together Again    ";
char* days[] = {"SUN","MON","TUE","WED","THU","FRI","SAT"};

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
}

void initAlphaDisplay()
{
  alpha4.begin(0x70);  // pass in the address
}

void initDotStar()
{
  dotstar.begin();
  dotstar.show();
  dotstar.setBrightness(DOTSTAR_BRIGHTNESS);
}

void initRTC()
{
  if (rtc.begin() == false) error(80);

  Serial.println("Something, something...");
  Serial.print("rtc.initialized() = ");
  Serial.println(rtc.initialized());
  Serial.print("rtc.lostPower() = ");
  Serial.println(rtc.lostPower());

  if (! rtc.initialized() || rtc.lostPower())
  {
    //   Serial.println("RTC is NOT initialized, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
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
}

void initTempSensor()
{
  // Make sure the sensor is found, you can also pass in a different i2c
  // address with tempsensor.begin(0x19) for example, also can be left in blank for default address use
  // Also there is a table with all addres possible for this sensor, you can connect multiple sensors
  // to the same i2c bus, just configure each sensor with a different address and define multiple objects for that
  //  A2 A1 A0 address
  //  0  0  0   0x18  this is the default address
  //  0  0  1   0x19
  //  0  1  0   0x1A
  //  0  1  1   0x1B
  //  1  0  0   0x1C
  //  1  0  1   0x1D
  //  1  1  0   0x1E
  //  1  1  1   0x1F
  if (!tempsensor.begin(0x18))
  {
    Serial.println("Couldn't find MCP9808! Check your connections and verify the address is correct.");
    while (1);
  }

  Serial.println("Found MCP9808!");

  tempsensor.setResolution(3); // sets the resolution mode of reading, the modes are defined in the table bellow:
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
}

void push(char c)
{
  digit[0] = digit[1];
  digit[1] = digit[2];
  digit[2] = digit[3];
  digit[3] = c;

  for (int i = 0; i < 4; i++)
  {
    alpha4.writeDigitAscii(i, digit[i]);
  }

  alpha4.writeDisplay();
  delay(250);
}

int level = 0;

void loop() {
  level = (level < 255) ? level + 1 : 0;
  dotstar.setPixelColor(0, level, 255, level);
  dotstar.show();

  //  displayMessage(message);
  //  displayCharacterSet();
  displayTime();
  delay(2000);
  displayDate();
  delay(2000);
  displayTemp();
  delay(2000);
}

//void displayTime()
//{
//  DateTime now = rtc.now();
//  char format[] = "hhmm";
//
//  displayMessage("    TIME");
//  displayMessage(now.toString(format));
//}

void displayTime()
{
  DateTime now = rtc.now();

  int hh = now.twelveHour() / 10;
  int h = now.twelveHour() % 10;
  int mm = now.minute() / 10;
  int m = now.minute() % 10;

  Serial.print("hh = ");
  Serial.print(hh);
  Serial.print(", h = ");
  Serial.print(h);
  Serial.print(", mm = ");
  Serial.print(mm);
  Serial.print(", m = ");
  Serial.println(m);

  displayMessage("    TIME");

  push((hh == 0) ? ' ' : hh + 48);
  push(h + 48);
  push(mm + 48);
  push(m + 48);
}

void displayDate()
{
  DateTime now = rtc.now();
  char format[] = "MMDD";
  Serial.print("now.dayOfTheWeek() = ");
  Serial.println(now.dayOfTheWeek());

  displayMessage("    DATE ");
  displayMessage(days[now.dayOfTheWeek()]);
  push(' ');
  displayMessage(now.toString(format));
}

void displayTemp()
{
  //  Serial.println("wake up MCP9808.... "); // wake up MCP9808 - power consumption ~200 mikro Ampere
  tempsensor.wake();   // wake up, ready to read!

  // Read and print out the temperature, also shows the resolution mode used for reading.
  //  Serial.print("Resolution in mode: ");
  //  Serial.println (tempsensor.getResolution());
  //  float c = tempsensor.readTempC();
  float f = tempsensor.readTempF();
  //  Serial.print("Temp: ");
  //  Serial.print(c, 4); Serial.print("*C\t and ");
  //  Serial.print(f, 4); Serial.println("*F.");

  int ff = (int) f;

  displayMessage("    TEMP");

  push(' ');
  push(ff / 10 + 48);
  push(ff % 10 + 48);
  push('F');

  //  delay(1000);
  //  Serial.println("Shutdown MCP9808.... ");
  tempsensor.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere, stops temperature sampling
  //  Serial.println("");
  //  delay(200);

}

void displayMessage(char* message)
{
  int i = 0;
  while (message[i] != 0)
  {
    push(message[i]);
    i++;
  }
}

void displayCharacterSet()
{
  for (char c = 0; c < 128; c++)
  {
    push(c);
  }
}
