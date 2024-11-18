



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
   CENTER OF WINDOW
    Small momentary button x2
    LCD x2
*/

//Globals for GPIO
#define smallButtonLeft D3
#define smallButtonRight D7
#define lcd_Data D1   //i2c
#define lcd_Clock D2  //i2c

//LCD
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

//MQTT Startup
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
//Adafruit_MQTT_Subscribe message = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/WindowsVista_message");
Adafruit_MQTT_Subscribe litButton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/CharlitPipe_Button");
Adafruit_MQTT_Publish indicators = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/WindowsVista_indicators");
Adafruit_MQTT_Publish leds = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/WindowsVista_leds");

//Variables
int manualTextMode = 0;
String newMessage = "";
int charlitMode = 0;
int ledBlinkfest = 0;
int indBlinkfest = 0;
int s = 0;
int m = 0;




// ***************************************
// *************** Setup *****************
// ***************************************


void setup() {

  //Initialize Switches
  pinMode(smallButtonLeft, INPUT_PULLUP);
  pinMode(smallButtonRight, INPUT_PULLUP);

  //Initialize Serial
  Serial.begin(115200);
  Serial.println("Booting");

  //Check for CHARLIT MODE
  delay(500);
  if (digitalRead(smallButtonLeft) == LOW && digitalRead(smallButtonRight) == LOW) {
    delay(10);
    charlitMode = 1;
  }

  //Initialize LCD
  lcd.init();                      // initialize the lcd
  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(1, 0);
  if (charlitMode == 1) {
    lcd.print("-Charlit Pipe-");
    lcd.setCursor(1, 1);
    lcd.print("+Secret Controls");
    delay(8000);
  }
  else {
    lcd.print(" __Warp Room__ ");
    lcd.setCursor(1, 1);
    lcd.print("Deez Nutz");
  }

  //WiFi Initialization
  wifiSetup();

  //Initialize MQTT
  //mqtt.subscribe(&message);
  mqtt.subscribe(&litButton);
  MQTT_connect();

  //Send Charlit Cmd
  if (charlitMode == 1) {
    leds.publish(69);
  }

  lcd.setCursor(1, 1);
  lcd.print("                ");
}




// ***************************************
// ************* Da Loop *****************
// ***************************************


void loop() {

  //Network Housekeeping
  ArduinoOTA.handle();
  MQTT_connect();

  if (charlitMode == 1) {
    //LCD State Manager
    Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(10))) {
      Serial.println("Subscription Recieved");
      uint16_t value = atoi((char *)litButton.lastread);
      //Serial.print("New Message: ");
      //Serial.println(value);
      //newMessage = value;
      //manualTextMode = 1;

      //Charlit Mode LCD Display Manager
      if (value == 1) {
        lcd.setCursor(1, 1);
        lcd.print("*Filling Water*");
      }
      if (value == 2) {
        lcd.setCursor(1, 1);
        lcd.print("  [LIFT OFF]  ");
        delay(500);
        lcd.noBacklight();
        delay(500);
        lcd.backlight();
      }
      if (value == 3) {
        lcd.setCursor(1, 1);
        lcd.print(">Expelling Smoke");
      }
      if (value == 4) {
        lcd.setCursor(1, 1);
        lcd.print("<RGB Mode Change");
        delay(1200);
      }
      if (value == 5) {
        lcd.setCursor(1, 1);
        lcd.print("=Dramatic Mode");
        delay(1200);
      }
      if (value == 0) {
        lcd.setCursor(1, 1);
        lcd.print("+Secret Controls");
      }
    }
  }
  else {
    //Print UpTime on LCD
    unsigned long upTime = millis();
    /*
    int h = upTime / 3600;
    int m = (upTime - h * 3600) / 60;
    int s = upTime - h * 3600 - m * 60;
*/
    s = upTime/1000 - (60 * m);
    
    if(s == 60) {
      m = s / 60;
    }
    
    String niceTime = ( /*String(h) + " : " +*/ "UpTime " + String(m) + " : " + String(s) + "   ");
    Serial.println(upTime);
    lcd.setCursor(0, 1);
    lcd.print(niceTime);
    

    //lcd.setCursor(1, 1);
    //lcd.print(upTime/1000);
  }

  //Button Manager
  if (digitalRead(smallButtonLeft) == LOW) {      //Indicators Change
    delay(600);
    if ((digitalRead(smallButtonLeft) == LOW) && ((digitalRead(smallButtonRight) == LOW))) {   //Indicators Blinkfest Toggle
      indicators.publish(10);
      Serial.println("Indicators Blinkfest Toggle");
      if (indBlinkfest == 0) {
        indBlinkfest = 1;
        lcd.setCursor(1, 1);
        lcd.print("IndBlinkfest ON");
      }
      else if (indBlinkfest == 1) {
        indBlinkfest = 0;
        lcd.setCursor(1, 1);
        lcd.print("IndBlinkfest OFF");
      }
      delay(3000);
    }
    else {
      indicators.publish(1);
      Serial.println("LEFT");
    }
  }
  if (digitalRead(smallButtonRight) == LOW) {     //LEDs Change
    delay(600);
    if ((digitalRead(smallButtonRight) == LOW) && ((digitalRead(smallButtonLeft) == LOW))) {   //LED Blinkfest Toggle
      leds.publish(10);

      Serial.println("LED Blinkfest Toggle");
      if (ledBlinkfest == 0) {
        ledBlinkfest = 1;
        lcd.setCursor(1, 1);
        lcd.print("LedBlinkfest ON");
      }
      else if (ledBlinkfest == 1) {
        ledBlinkfest = 0;
        lcd.setCursor(1, 1);
        lcd.print("LedBlinkfest OFF");
      }
      delay(3000);
    }
    else {
      leds.publish(1);
      Serial.println("RIGHT");
    }
  }
}




// ***************************************
// ********** Backbone Methods ***********
// ***************************************


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
  ArduinoOTA.setHostname("WindowsVista-Center");                                                          /** TODO **/
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
