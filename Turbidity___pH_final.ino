#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// WiFi parameters
#define WLAN_SSID       "Sewmina"
#define WLAN_PASS       "Ranawaka2001"

// Adafruit IO
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "gw2001"
#define AIO_KEY         "aio_urnw36J4L5uXJNAHOBvpRDN159IJ"

 

WiFiClient client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Feeds to publish data to Adafruit IO
Adafruit_MQTT_Publish turbidity_level = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Turbidity_level");
Adafruit_MQTT_Publish pH_level = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pH_value");

// Feeds to subscribe to button presses
Adafruit_MQTT_Subscribe take_readings = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/take-readings");
Adafruit_MQTT_Subscribe reset = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/reset");

int sensorPin = 34;
const int adcPin = 36;
const float m = -5.436;  //Gradient
bool recording = false;

const int outputPin = 23;

//int RoboticArm=0;

void MQTT_callback(Adafruit_MQTT_Subscribe *subscription) {
  char *data = (char *)subscription->lastread;
  int value = atoi(data);
  
  if (subscription == &take_readings) {
    if (value == 1) {
      recording = true;
      digitalWrite(outputPin, value);
      Serial.println("Started recording.");
    }
  } else if (subscription == &reset) {
    if (value == 0) {
      recording = false;
      digitalWrite(outputPin, value);
      Serial.println("Stopped recording.");
    }
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("Adafruit IO Example"));
  
  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  delay(10);
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

  // Setup MQTT subscriptions
  mqtt.subscribe(&take_readings);
  mqtt.subscribe(&reset);

  // connect to adafruit io
  connect();

  //Set pin mode to output
  pinMode(outputPin, OUTPUT);
}

// connect to adafruit io via MQTT
void connect() {
  Serial.print(F("Connecting to Adafruit IO... "));
  int8_t ret;
  while ((ret = mqtt.connect()) != 0) {
    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavailable")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authorized")); break;
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
  // ping adafruit io a few times to make sure we remain connected
  if (!mqtt.ping(3)) {
    // reconnect to adafruit io
    if (!mqtt.connected()) {
      Serial.println("MQTT not connected! Reconnecting...");
      connect();
    }
  }

  // Check for incoming subscription messages
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    MQTT_callback(subscription);
  }

  int sensorValue = analogRead(sensorPin);
  int turbidity = map(sensorValue, 0, 750, 100, 0);
  
  Serial.print((int)turbidity); 
  Serial.println(" ntu");
  delay(8000);

  // Convert turbidity value to string
  String turbidityStr = String(turbidity);

  float Po = analogRead(adcPin) * 3.3 / 4095;
  float phValue = 7 - (2.88 - Po) * m;
  Serial.print("pH value = "); 
  Serial.println(phValue);
  delay(8000);

  String phStr = String(phValue);
   
  if(recording == true)
  {
    if (!turbidity_level.publish(turbidityStr.c_str())) // Publish to Adafruit
    {  
      Serial.println(F("Failed"));
    }
    if(!pH_level.publish(phStr.c_str()))
    {
      Serial.println(F("Failed"));
    }
    else
    {
      Serial.println(F("Sent!"));
    }
  }
}
