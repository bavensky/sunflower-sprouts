#include <Arduino.h>
#include "DHT.h"
#include <SPI.h>
#include <SD.h>
//#include <TimeLib.h>
#include <ESP8266WiFi.h>
//#include <WiFiUdp.h>

File datalog;

#define DHTPIN 16     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)

#define DEBUG
#define DEBUG_PRINTER Serial
#ifdef DEBUG
#define DEBUG_PRINT(...) { DEBUG_PRINTER.print(__VA_ARGS__); }
#define DEBUG_PRINTLN(...) { DEBUG_PRINTER.println(__VA_ARGS__); }
#else
#define DEBUG_PRINT(...) {}
#define DEBUG_PRINTLN(...) {}
#endif

const char* ssid     = "@ESPertAP_001";
const char* password = "espertap";

//static const char ntpServerName[] = "th.pool.ntp.org";
//const int timeZone = +7;
//WiFiUDP Udp;
//unsigned int localPort = 8888;
//time_t getNtpTime();
//void digitalClockDisplay();
//void printDigits(int digits);
//void sendNTPpacket(IPAddress &address);

unsigned int moi;

DHT * dht;

void connectWifi();
void reconnectWifiIfLinkDown();
void initDht(DHT **dht, uint8_t pin, uint8_t dht_type);
void readDht(DHT *dht, float *temp, float *humid);
void uploadThingsSpeak(float t, float h, float moi);

void setup() {
  Serial.begin(115200);
  delay(10);

//  if (!SD.begin(2)) {
//    Serial.println("initialization failed!");
//    return;
//  }

  pinMode(DHTPIN, INPUT_PULLUP);
  delay(100);
  initDht(&dht, DHTPIN, DHTTYPE);

  // connectWifi();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

//  Serial.print("IP number assigned by DHCP is ");
//  Serial.println(WiFi.localIP());
//  Serial.println("Starting UDP");
//  Udp.begin(localPort);
//  Serial.print("Local port: ");
//  Serial.println(Udp.localPort());
//  Serial.println("waiting for sync");
//  setSyncProvider(getNtpTime);
//  setSyncInterval(300);

//  datalog = SD.open("datalog.csv", FILE_WRITE);
//  if (datalog) {
//    datalog.println("Date, Time, Temp, Humid, Moi");
//    datalog.close();
//    delay(100);
//  }
}

time_t prevDisplay = 0;

void loop() {
  static float t_dht;
  static float h_dht;
  moi = map(analogRead(A0), 600, 800, 100, 0);
//  datalog = SD.open("datalog.csv", FILE_WRITE);
//  if (datalog) {
    readDht(dht, &t_dht, &h_dht);
//    uploadThingsSpeak(t_dht, h_dht, moi);
//    datalog.print(day());
//    datalog.print("/");
//    datalog.print(month());
//    datalog.print("/");
//    datalog.print(year());
//    datalog.print(",");
//    datalog.print(hour());
//    datalog.print(":");
//    datalog.print(minute());
//    datalog.print(":");
//    datalog.print(second());
//    datalog.print(",");
//    datalog.print(t_dht);
//    datalog.print(",");
//    datalog.print(h_dht);
//    datalog.print(",");
//    datalog.println(moi);
//    datalog.close();

//    delay(20 * 1000);
    reconnectWifiIfLinkDown();
//    digitalClockDisplay();
    Serial.println("done.");
//  }
}

void reconnectWifiIfLinkDown() {
  if (WiFi.status() != WL_CONNECTED) {
    DEBUG_PRINTLN("WIFI DISCONNECTED");
    connectWifi();
  }
}

void connectWifi() {
  DEBUG_PRINTLN();
  DEBUG_PRINTLN();
  DEBUG_PRINT("Connecting to ");
  DEBUG_PRINTLN(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    DEBUG_PRINT(".");
  }

  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("WiFi connected");
  DEBUG_PRINTLN("IP address: ");
  DEBUG_PRINTLN(WiFi.localIP());
}

void initDht(DHT **dht, uint8_t pin, uint8_t dht_type) {
  // Connect pin 1 (on the left) of the sensor to +5V
  // NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
  // to 3.3V instead of 5V!
  // Connect pin 2 of the sensor to whatever your DHTPIN is
  // Connect pin 4 (on the right) of the sensor to GROUND
  // Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

  // Initialize DHT sensor for normal 16mhz Arduino
  // NOTE: For working with a faster chip, like an Arduino Due or Teensy, you
  // might need to increase the threshold for cycle counts considered a 1 or 0.
  // You can do this by passing a 3rd parameter for this threshold.  It's a bit
  // of fiddling to find the right value, but in general the faster the CPU the
  // higher the value.  The default for a 16mhz AVR is a value of 6.  For an
  // Arduino Due that runs at 84mhz a value of 30 works.
  // Example to initialize DHT sensor for Arduino Due:
  //DHT dht(DHTPIN, DHTTYPE, 30);

  *dht = new DHT(pin, dht_type, 21);
  (*dht)->begin();
  DEBUG_PRINTLN(F("DHTxx test!"))  ;
}

void uploadThingsSpeak(float t, float h, float moi) {
  static const char* host = "api.thingspeak.com";
  static const char* apiKey = "OIJLSVMYOVZYK6V2";

  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(host, httpPort)) {
    DEBUG_PRINTLN("connection failed");
    return;
  }

  // We now create a URI for the request
  String url = "/update/";
  //  url += streamId;
  url += "?key=";
  url += apiKey;
  url += "&field1=";
  url += t;
  url += "&field2=";
  url += h;
  url += "&field3=";
  url += moi;

  DEBUG_PRINT("Requesting URL: ");
  DEBUG_PRINTLN(url);

  // This will send the request to the server
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Connection: close\r\n\r\n");
}

void readDht(DHT * dht, float * temp, float * humid) {

  if (dht == NULL) {
    DEBUG_PRINTLN(F("[dht22] is not initialised. please call initDht() first."));
    return;
  }

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht->readHumidity();

  // Read temperature as Celsius
  float t = dht->readTemperature();
  // Read temperature as Fahrenheit
  float f = dht->readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    DEBUG_PRINTLN("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index
  // Must send in temp in Fahrenheit!
  float hi = dht->computeHeatIndex(f, h);

  DEBUG_PRINT("Humidity: ");
  DEBUG_PRINT(h);
  DEBUG_PRINT(" %\t");
  DEBUG_PRINT("Temperature: ");
  DEBUG_PRINT(t);
  DEBUG_PRINT(" *C ");
  DEBUG_PRINT(f);
  DEBUG_PRINT(" *F\t");
  DEBUG_PRINT("\n");
  *temp = t;
  *humid = f;
}

//void digitalClockDisplay()
//{
//  // digital clock display of the time
//  Serial.print(hour());
//  printDigits(minute());
//  printDigits(second());
//  Serial.print(" ");
//  Serial.print(day());
//  Serial.print(".");
//  Serial.print(month());
//  Serial.print(".");
//  Serial.print(year());
//  Serial.println();
//}

//void printDigits(int digits)
//{
//  // utility for digital clock display: prints preceding colon and leading 0
//  Serial.print(":");
//  if (digits < 10)
//    Serial.print('0');
//  Serial.print(digits);
//}

/*-------- NTP code ----------*/

//const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
//byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

//time_t getNtpTime()
//{
//  IPAddress ntpServerIP; // NTP server's ip address
//
//  while (Udp.parsePacket() > 0) ; // discard any previously received packets
//  Serial.println("Transmit NTP Request");
//  // get a random server from the pool
//  WiFi.hostByName(ntpServerName, ntpServerIP);
//  Serial.print(ntpServerName);
//  Serial.print(": ");
//  Serial.println(ntpServerIP);
//  sendNTPpacket(ntpServerIP);
//  uint32_t beginWait = millis();
//  while (millis() - beginWait < 1500) {
//    int size = Udp.parsePacket();
//    if (size >= NTP_PACKET_SIZE) {
//      Serial.println("Receive NTP Response");
//      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
//      unsigned long secsSince1900;
//      // convert four bytes starting at location 40 to a long integer
//      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
//      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
//      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
//      secsSince1900 |= (unsigned long)packetBuffer[43];
//      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
//    }
//  }
//  Serial.println("No NTP Response :-(");
//  return 0; // return 0 if unable to get the time
//}

// send an NTP request to the time server at the given address
//void sendNTPpacket(IPAddress & address)
//{
//  // set all bytes in the buffer to 0
//  memset(packetBuffer, 0, NTP_PACKET_SIZE);
//  // Initialize values needed to form NTP request
//  // (see URL above for details on the packets)
//  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
//  packetBuffer[1] = 0;     // Stratum, or type of clock
//  packetBuffer[2] = 6;     // Polling Interval
//  packetBuffer[3] = 0xEC;  // Peer Clock Precision
//  // 8 bytes of zero for Root Delay & Root Dispersion
//  packetBuffer[12] = 49;
//  packetBuffer[13] = 0x4E;
//  packetBuffer[14] = 49;
//  packetBuffer[15] = 52;
//  // all NTP fields have been given values, now
//  // you can send a packet requesting a timestamp:
//  Udp.beginPacket(address, 123); //NTP requests are to port 123
//  Udp.write(packetBuffer, NTP_PACKET_SIZE);
//  Udp.endPacket();
//}
