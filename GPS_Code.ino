#include <ESP8266WiFi.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

static const int RXPin = 12, TXPin = 13;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial softSerial(RXPin, TXPin);


// WiFi parameters
#define WLAN_SSID       "SSID"
#define WLAN_PASS       "Password"
 
 
// Adafruit IO
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
//Enter the username and key from the Adafruit IO
#define AIO_USERNAME    "gw2001"
#define AIO_KEY         "Adafruitio Key" 
WiFiClient client;
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish GPSLocation = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/location/csv");



float speed_mph = 0;
float alltitude = 0;
float lati;     //Storing the Latitude
float longi;    //Storing the Longitude

char gpsdata[120];
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  softSerial.begin(GPSBaud);
  
  Serial.print(F("Connecting to "));
  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
 
  // connect to adafruit io
  connect();
}

// connect to adafruit io via MQTT
void connect() {
  Serial.print(F("Connecting to Adafruit IO... "));
  int8_t ret;
  while ((ret = mqtt.connect()) != 0) {
    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }
 
    if(ret >= 0)
      mqtt.disconnect();
 
    Serial.println(F("Retrying connection..."));
    delay(10000);
  }
  Serial.println(F("Adafruit IO Connected!"));
}

void loop() {
  // put your main code here, to run repeatedly:
if(! mqtt.ping(3)) {
    // reconnect to adafruit io
    if(! mqtt.connected())
      connect();
  }
 
  getCoordinates();

  Serial.print("Lati = ");
  Serial.print(lati,6);
  Serial.print("\tLongi = ");
  Serial.println(longi,6);
 
    if (!GPSLocation.publish(gpsdata)) {                     //Publish to Adafruit
      Serial.println(F("Failed"));
    }
    else {
      Serial.println(F("Sent!"));
    }
    delay(5000);
}



void getCoordinates()
{
   readGPSData();
  char *p = gpsdata;
  // add speed value
  dtostrf(speed_mph, 2, 6, p);
  p += strlen(p);
  p[0] = ','; p++;

  // concat latitude
  dtostrf(lati, 2, 6, p);
  p += strlen(p);
  p[0] = ','; p++;

  // concat longitude
  dtostrf(longi, 3, 6, p);
  p += strlen(p);
  p[0] = ','; p++;

  // concat altitude
  dtostrf(alltitude, 2, 6, p);
  p += strlen(p);

  // null terminate
  p[0] = 0;

}

void readGPSData()
{
  if(gps.location.isValid()){
    lati = gps.location.lat();
    longi = gps.location.lng();
    Serial.print("Lati: ");
    Serial.print(lati,6);
    Serial.print("\tLongi: ");
    Serial.println(longi,6);
  }
  waitGPS(1000);
  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println("Waiting for data...");
}

static void waitGPS(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (softSerial.available())
      gps.encode(softSerial.read());
  } while (millis() - start < ms);
}
