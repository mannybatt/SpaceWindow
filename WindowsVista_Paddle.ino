



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
   LEFT HALF OF WINDOW
    Paddle switch x2
    Indicator lights x3
    Low pressure x1
*/

//Globals for Switches
#define paddleUp D0
#define paddleDown D1

//Globals for Lights
#define indicatorBlue D4
#define indicatorYellow D3
#define indicatorGreen D2
#define lowPressure D7
int indicators[3] = {indicatorBlue,indicatorYellow,indicatorGreen};

//MQTT Startup
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Subscribe indicatorSwitchUp = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/WindowsVista_indicators");

//Variables
int lowPressureState = 0; //Paddle position
int lowPressureLightingState = 0; //Lighting on/off
int paddleOverride = 0;
unsigned long currentTime;
int lastLedPattern[3] = {0,0,0};
int blinkfest = 1;




// ***************************************
// *************** Setup *****************
// ***************************************


void setup() {

  //Initialize Serial
  Serial.begin(115200);
  Serial.println("Booting");

  //WiFi Initialization
  wifiSetup();

  //Initialize MQTT
  mqtt.subscribe(&indicatorSwitchUp);

  //Initialize Switches
  pinMode(paddleUp, INPUT_PULLUP);
  pinMode(paddleDown, INPUT_PULLUP);

  //Initialize Lights
  pinMode(indicatorBlue, OUTPUT);
  pinMode(indicatorYellow, OUTPUT);
  pinMode(indicatorGreen, OUTPUT);
  pinMode(lowPressure, OUTPUT);

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
  indicatorRandomizer();
}




// ***************************************
// ************* Da Loop *****************
// ***************************************


void loop() {

  //Network Housekeeping
  ArduinoOTA.handle();
  MQTT_connect();

  //State Manager
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(10))) {
    Serial.println("Subscription Recieved");
    uint16_t value = atoi((char *)indicatorSwitchUp.lastread);

    // [Indicators Randomizer]
    if (value == 1) {
      Serial.println("Switching Indicators Up!");
      indicatorRandomizer();
    }
    if(value == 10) {
      Serial.println("Toggle Blinkfest");
      if(blinkfest == 0){
        blinkfest = 1;
      }
      else if(blinkfest == 1){
        blinkfest = 0;
      }
    }
  }

  //Indicators Blinkfest
  if(blinkfest == 1){
    digitalWrite(indicators[random(0,sizeof(indicators)/sizeof(int))],HIGH);
    delay(random(1000,10000));
    digitalWrite(indicators[random(0,sizeof(indicators)/sizeof(int))],LOW);
  }

  //Paddle Manager
  if (digitalRead(paddleUp) == LOW) {
    Serial.println("lowPressureState = 1");
    lowPressureState = 1;
  }
  else {
    Serial.println("lowPressureState = 0");
    lowPressureState = 0;
  }
  lowPressureLighting();

}




// ***************************************
// ********** Backbone Methods ***********
// ***************************************


void lowPressureLighting() {

  if (lowPressureState == 0) { //ON
    digitalWrite(lowPressure, HIGH);
  }
  else if (lowPressureState == 1) { //BLINK
    digitalWrite(lowPressure, LOW);
    delay(1150);
    digitalWrite(lowPressure, HIGH);
    delay(1150);
  }
}

void indicatorRandomizer() {

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
    digitalWrite(indicatorBlue, LOW);
  }
  else {
    digitalWrite(indicatorBlue, HIGH);
  }

  if (bit2 == 0) {
    digitalWrite(indicatorYellow, LOW);
  }
  else {
    digitalWrite(indicatorYellow, HIGH);
  }

  if (bit3 == 0) {
    digitalWrite(indicatorGreen, LOW);
  }
  else {
    digitalWrite(indicatorGreen, HIGH);
  }
  lastLedPattern[0] = bit1;
  lastLedPattern[1] = bit2;
  lastLedPattern[2] = bit3;
}

void lightsOff() {

  digitalWrite(indicatorBlue, LOW);
  digitalWrite(indicatorYellow, LOW);
  digitalWrite(indicatorGreen, LOW);
  digitalWrite(lowPressure, LOW);
}

void lightsOn() {

  digitalWrite(indicatorBlue, HIGH);
  digitalWrite(indicatorYellow, HIGH);
  digitalWrite(indicatorGreen, HIGH);
  digitalWrite(lowPressure, HIGH);
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
  ArduinoOTA.setHostname("WindowsVista-Paddle");                                                          /** TODO **/
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
