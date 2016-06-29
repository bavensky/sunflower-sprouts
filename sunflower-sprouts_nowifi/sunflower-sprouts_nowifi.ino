#include "DHT.h"
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <MCP3221.h>
#include "RTClib.h"

File datalog;
RTC_DS1307 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

#define Moist  A0
#define DHTPIN 16
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE, 21);

byte i2cAddress = 0x4D;
const int I2CadcVRef = 4098;
MCP3221 i2cADC(i2cAddress, I2CadcVRef);

float t, h;
int lightAVG, Soil;

void readDHT();

void setup() {

#ifndef ESP8266
  while (!Serial);
#endif

  Serial.begin(115200);
  dht.begin();

  if (!SD.begin(2)) {
    Serial.println("initialization failed!");
    return;
  }
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  
  //  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  
  datalog = SD.open("datalog.csv", FILE_WRITE);
  if (datalog) {
    datalog.println("Date, Time, Temp, Humid, Moist, Lux");
    datalog.close();
    delay(100);
  }

}

void loop() {
  datalog = SD.open("datalog.csv", FILE_WRITE);
  if (datalog) {
    DateTime now = rtc.now();
    readDHT();
    Soil = map(analogRead(Moist), 0, 230, 1, 10);
    lightAVG = map(i2cADC.readI2CADC(), 4098, 0, 0, 2000);

    datalog.print(now.day());
    datalog.print("/");
    datalog.print(now.month());
    datalog.print("/");
    datalog.print(now.year());
    datalog.print(",");
    datalog.print(now.hour());
    datalog.print(":");
    datalog.print(now.minute());
    datalog.print(":");
    datalog.print(now.second());
    datalog.print(",");
    datalog.print(t);
    datalog.print(",");
    datalog.print(h);
    datalog.print(",");
    
    if(Soil >= 1 && Soil <= 3) datalog.print("DRY");
    else if (Soil >= 4 && Soil <= 7) datalog.print("Normal");
    else if (Soil >= 8 && Soil <= 10) datalog.print("WET");

    datalog.print(",");
    datalog.println(lightAVG);
    datalog.close();
    
    delay(5 * 10000);  // save data every 5 minute
    Serial.println("Done");
  }
}

void readDHT()  {
  h = dht.readHumidity();
  t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
}

