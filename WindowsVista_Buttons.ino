



// ***************************************
// ********** Global Variables ***********
// ***************************************


//Globals for Wifi Setup and OTA
#include <credentials.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

//WiFi Credentials
#ifndef STASSID
#define STASSID "your_ssid"
#endif
#ifndef STAPSK
#define STAPSK  "your_password"
#endif
const char* ssid = STASSID;
const char* password = STAPSK;

//MQTT
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#ifndef AIO_SERVER
#define AIO_SERVER      "your_MQTT_server_address"
#endif
#ifndef AIO_SERVERPORT
#define AIO_SERVERPORT  0000 //Your MQTT port
#endif
#ifndef AIO_USERNAME
#define AIO_USERNAME    "your_MQTT_username"
#endif
#ifndef AIO_KEY
#define AIO_KEY         "your_MQTT_key"
#endif
#define MQTT_KEEP_ALIVE 150
unsigned long previousTime;
float mqttConnectFlag = 0.0;

/**
   RIGHT HALF OF WINDOW
    Big buttons x3
    Big button lights x3
    Small singular leds x3
*/

//Globals for Switches
#define buttonRed D6
#define buttonYellow D7
#define buttonGreen 3

//Globals for Lights
#define lightRed D0
#define lightYellow D1
#define lightGreen D2
#define ledTop 1
#define ledMiddle D4
#define ledBottom D3
int leds[3] = {ledTop, ledMiddle, ledBottom};

//MQTT Startup
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Subscribe ledSwitchUp = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/WindowsVista_leds");
Adafruit_MQTT_Publish litButton = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/CharlitPipe_Button");

//Variables
int redState = 0;
int yellowState = 0;
int greenState = 0;
int lastLedPattern[3] = {0, 0, 0};
int blinkfest = 1;




// ***************************************
// *************** Setup *****************
// ***************************************


void setup() {

  //********** CHANGE PIN FUNCTION  TO GPIO **********
  //GPIO 1 (TX) swap the pin to a GPIO.
  pinMode(1, FUNCTION_3);
  //GPIO 3 (RX) swap the pin to a GPIO.
  pinMode(3, FUNCTION_3);
  //**************************************************

  //Initialize Serial
  Serial.begin(115200);
  Serial.println("Booting");

  //WiFi Initialization
  wifiSetup();

  //Initialize MQTT
  mqtt.subscribe(&ledSwitchUp);

  //Initialize
  pinMode(buttonRed, INPUT_PULLUP);
  pinMode(buttonYellow, INPUT_PULLUP);
  pinMode(buttonGreen, INPUT);

  //Initialize Lights
  pinMode(lightRed, OUTPUT);
  pinMode(lightYellow, OUTPUT);
  pinMode(lightGreen, OUTPUT);
  pinMode(ledTop, OUTPUT);
  pinMode(ledMiddle, OUTPUT);
  pinMode(ledBottom, OUTPUT);

  //Test Dem Lights
  lightsOff();
  delay(850);
  lightsOn();
  delay(850);
  lightsOff();
  delay(850);
  lightsOn();
  delay(850);
  lightsOff();
  delay(1000);
  ledRandomizer();
}




// ***************************************
// ************* Da Loop *****************
// ***************************************


void loop() {

  //Network Housekeeping
  ArduinoOTA.handle();
  MQTT_connect();

  //Randomizer Manager
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(10))) {
    Serial.println("Subscription Recieved");
    uint16_t value = atoi((char *)ledSwitchUp.lastread);
    // [LEDs Randomizer]
    if (value == 1) {
      Serial.println("Switching LEDs Up!");
      ledRandomizer();
    }
    if (value == 10) {
      Serial.println("Toggle Blinkfest");
      if (blinkfest == 0) {
        blinkfest = 1;
      }
      else if (blinkfest == 1) {
        blinkfest = 0;
      }
    }
  }

  //LEDs Blinkfest
  if (blinkfest == 1) {
    digitalWrite(leds[random(0, sizeof(leds) / sizeof(int))], HIGH);
    delay(random(20, 200));
    digitalWrite(leds[random(0, sizeof(leds) / sizeof(int))], LOW);
  }

  //Button Manager
  if (digitalRead(buttonRed) == LOW) {
    delay(10);
    if (redState == 0) {
      redState = 1;
    }
    else {
      redState = 0;
    }
    delay(500);
  }
  if (digitalRead(buttonYellow) == LOW) {
    delay(10);
    if (yellowState == 0) {
      yellowState = 1;
    }
    else {
      yellowState = 0;
    }
    delay(500);
  }
  if (digitalRead(buttonGreen) == LOW) {
    delay(10);
    if (greenState == 0) {
      greenState = 1;
    }
    else {
      greenState = 0;
    }
    delay(500);
  }

  //Normal Button Lights Manager
  if (redState == 0) {
    digitalWrite(lightRed, LOW);
  }
  else {
    digitalWrite(lightRed, HIGH);
  }
  if (yellowState == 0) {
    digitalWrite(lightYellow, LOW);
  }
  else {
    digitalWrite(lightYellow, HIGH);
  }
  if (greenState == 0) {
    digitalWrite(lightGreen, LOW);
  }
  else {
    digitalWrite(lightGreen, HIGH);
  }
}




// ***************************************
// ********** Backbone Methods ***********
// ***************************************


void ledRandomizer() {

  randomSeed(analogRead(8));
  int bit1 = random(100) % 2;
  int bit2 = random(100) % 2;
  int bit3 = random(100) % 2;

  //Check for Zeroes and No Change Scenarios and try again
  while (((lastLedPattern[0] == bit1) && (lastLedPattern[1] == bit2) && (lastLedPattern[2] == bit3)) ||
         ((bit1 == 0) && (bit2 == 0) && (bit3 == 0))) {
    randomSeed(analogRead(8));
    int bit1 = random(100) % 2;
    int bit2 = random(100) % 2;
    int bit3 = random(100) % 2;
  }

  if (bit1 == 0) {
    digitalWrite(ledTop, LOW);
  }
  else {
    digitalWrite(ledTop, HIGH);
  }

  if (bit2 == 0) {
    digitalWrite(ledMiddle, LOW);
  }
  else {
    digitalWrite(ledMiddle, HIGH);
  }

  if (bit1 == 0) {
    digitalWrite(ledBottom, LOW);
  }
  else {
    digitalWrite(ledBottom, HIGH);
  }
  lastLedPattern[0] = bit1;
  lastLedPattern[1] = bit2;
  lastLedPattern[2] = bit3;
}

void lightsOff() {

  digitalWrite(lightRed, LOW);
  digitalWrite(lightYellow, LOW);
  digitalWrite(lightGreen, LOW);
  digitalWrite(ledTop, LOW);
  digitalWrite(ledMiddle, LOW);
  digitalWrite(ledBottom, LOW);
}

void lightsOn() {

  digitalWrite(lightRed, HIGH);
  digitalWrite(lightYellow, HIGH);
  digitalWrite(lightGreen, HIGH);
  digitalWrite(ledTop, HIGH);
  digitalWrite(ledMiddle, HIGH);
  digitalWrite(ledBottom, HIGH);
}


void MQTT_connect() {

  int8_t ret;
  // Stop if already connected.
  if (mqtt.connected()) {
    if (mqttConnectFlag == 0) {
      //Serial.println("Connected");
      mqttConnectFlag++;
    }
    return;
  }
  Serial.println("Connecting to MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      //while (1);
      Serial.println("Wait 5 secomds to reconnect");
      delay(5000);
    }
  }
}

void wifiSetup() {

  //Serial
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println();
  Serial.println("****************************************");
  Serial.println("Booting");

  //WiFi and OTA
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  ArduinoOTA.setHostname("WindowsVista-Buttons");                                                          /** TODO **/
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}
