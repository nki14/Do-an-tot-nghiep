#ifdef ENABLE_DEBUG
#define DEBUG_ESP_PORT Serial
#define NODEBUG_WEBSOCKETS
#define NDEBUG
#endif 

#include <Arduino.h>
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ESP32) || defined(ARDUINO_ARCH_RP2040)
#include <WiFi.h>
#endif

#include "SinricPro.h"
#include "SinricProDimSwitch.h"
#include "RGBLED.h"
#include "SinricProSwitch.h"
#include <FastLED.h>


#define WIFI_SSID        "Home 71-73"  //"FETEL@E105_SV"    
#define WIFI_PASS         "Home@71-73" //"bmdtE105SV"
#define APP_KEY           "13de0478-8cd6-4bee-a948-3c890f9af65f"      // Should look like "de0bxxxx-1x3x-4x3x-ax2x-5dabxxxxxxxx"
#define APP_SECRET        "116d11d0-d1be-4c67-8aeb-b510678caadc-8b3a782a-6e45-44c3-8d25-29b75293e5a9"   // Should look like "5f36xxxx-x3x7-4x3x-xexe-e86724a9xxxx-4c4axxxx-3x3x-x5xe-x9x3-333d65xxxxxx"

#define DIMLIGHT_ID  "668f94196e1af35935121bc5"    // Should look like "5dc1564130xxxxxxxxxxxxxx"
#define ONOFFLIGHT_ID  "668f93fb674e208e6ff495f0"
#define RGBLIGHT_ID  "668f93d86e1af35935121b89"

#define DIMLIGHT_PIN 5 //D1      
#define ONOFFLIGHT_PIN 15//D8
#define RGBLIGHT_PIN 16 //D0

#define SW_DIMLIGHT_PIN 14      //D5
#define SW_ONOFFLIGHT_PIN 12 //D6
#define SW_RGBLIGHT_PIN 13 //D7


#define NUM_LEDS 12

#define BAUD_RATE         115200                // Change baudrate to your need

// we use a struct to store all states and values for our dimmable switch
struct {
  bool powerState = false;
  int powerLevel = 0;
} device_state;

bool lastFlipSwitchState;
unsigned long lastFlipSwitchChange;

unsigned long lastBtnPress = 0;


bool onOffState = LOW;
bool dimState = LOW;
bool rgbState = LOW;

CRGB leds[NUM_LEDS];

SinricProDimSwitch &myDimSwitch = SinricPro[DIMLIGHT_ID];
SinricProSwitch& myOnOffSwitch = SinricPro[ONOFFLIGHT_ID];
RGBLED &rGBLED = SinricPro[RGBLIGHT_ID];

std::map<String, String> globalModes;

// ColorController
struct Color {
  byte r;
  byte g;
  byte b;
};

Color color;

bool onPowerState_Dim(const String &deviceId, bool &state) {
  Serial.printf("Device %s power turned %s \r\n", deviceId.c_str(), state?"on":"off");
  device_state.powerState = state;
  dimState = state;
  digitalWrite(DIMLIGHT_PIN, state? HIGH: LOW);             // set the new relay state
  return true; // request handled properly
}

bool onPowerLevel_Dim(const String &deviceId, int &powerLevel) {
  device_state.powerLevel = powerLevel;
  Serial.printf("Device %s power level changed to %d\r\n", deviceId.c_str(), device_state.powerLevel);
  analogWrite(DIMLIGHT_PIN, powerLevel );
  return true;
}

bool onAdjustPowerLevel_Dim(const String &deviceId, int &levelDelta) {
  device_state.powerLevel += levelDelta;
  Serial.printf("Device %s power level changed about %i to %d\r\n", deviceId.c_str(), levelDelta, device_state.powerLevel);
  levelDelta = device_state.powerLevel;
  return true;
}

//For On Off

bool onPowerState_OnOff(String deviceId, bool &state)
{
  Serial.printf("%s: %s\r\n", deviceId.c_str(), state ? "on" : "off");
  
  digitalWrite(ONOFFLIGHT_PIN, state? HIGH: LOW);             // set the new relay state
  onOffState = state;
  return true;
}
//For RGB
bool onPowerState_RGB(const String &deviceId, bool &state) {
  Serial.printf("[Device: %s]: Powerstate changed to %s\r\n", deviceId.c_str(), state ? "on" : "off");
  rgbState = state;
  return true; // request handled properly
}
bool onSetMode(const String& deviceId, const String& instance, String &mode) {
  Serial.printf("[Device: %s]: Modesetting for \"%s\" set to mode %s\r\n", deviceId.c_str(), instance.c_str(), mode.c_str());
  globalModes[instance] = mode;
  return true;
}

// PowerStateController
void updatePowerState(bool state) {
  rGBLED.sendPowerStateEvent(state);
}

// ColorController
bool onColor(const String &deviceId, byte &r, byte &g, byte &b) {
  Serial.printf("[Device: %s]: Color set to red=%d, green=%d, blue=%d\r\n", deviceId.c_str(), r, g, b);
  color.r = r;
  color.g = g;
  color.b = b;
  if(rgbState && globalModes["modeInstance1"] == "Solid") 
  {  
    AllOn(r, g, b); 
  }
  return true; // request handled properly
}

// ColorController
void updateColor(byte r, byte g, byte b) {
  rGBLED.sendColorEvent(r, g, b);
  //fill_solid(leds, NUM_LEDS, CRGB(r, g, b));
    //FastLED.show();
}

// ModeController
void updateMode(String instance, String mode) {
  rGBLED.sendModeEvent(instance, mode, "PHYSICAL_INTERACTION");
}


void setupWiFi() {
  Serial.printf("\r\n[Wifi]: Connecting");

  #if defined(ESP8266)
    WiFi.setSleepMode(WIFI_NONE_SLEEP); 
    WiFi.setAutoReconnect(true);
  #elif defined(ESP32)
    WiFi.setSleep(false); 
    WiFi.setAutoReconnect(true);
  #endif

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.printf(".");
    delay(250);
  }
  IPAddress localIP = WiFi.localIP();
  Serial.printf("connected!\r\n[WiFi]: IP-Address is %d.%d.%d.%d\r\n", localIP[0], localIP[1], localIP[2], localIP[3]);
}

void setupSinricPro() {
  //SinricProDimSwitch &myDimSwitch = SinricPro[DIMLIGHT_ID];

  // set callback function to device
  myDimSwitch.onPowerState(onPowerState_Dim);
  myDimSwitch.onPowerLevel(onPowerLevel_Dim);
  myDimSwitch.onAdjustPowerLevel(onAdjustPowerLevel_Dim);


  //SinricProSwitch& myOnOffSwitch = SinricPro[ONOFFLIGHT_ID];
  myOnOffSwitch.onPowerState(onPowerState_OnOff);

  rGBLED.onPowerState(onPowerState_RGB);

  // ModeController
  rGBLED.onSetMode("modeInstance1", onSetMode);
// ColorController
  rGBLED.onColor(onColor);

  // setup SinricPro
  SinricPro.onConnected([](){ Serial.printf("Connected to SinricPro\r\n"); }); 
  SinricPro.onDisconnected([](){ Serial.printf("Disconnected from SinricPro\r\n"); });

  SinricPro.restoreDeviceStates(true);
  
  SinricPro.begin(APP_KEY, APP_SECRET);


}

// main setup function
void setup() {
  Serial.begin(BAUD_RATE); Serial.printf("\r\n\r\n");
  setupPins();
  setupWiFi();
  setupSinricPro();
}

void loop() {
  
  SinricPro.handle();
  //handleButtonPress(SW_DIMLIGHT_PIN, dimState, DIMLIGHT_PIN, DIMLIGHT_ID);
  //handleButtonPress(SW_ONOFFLIGHT_PIN, onOffState, ONOFFLIGHT_PIN, ONOFFLIGHT_ID);
  handleDimSwitchPress() ;
  handleOnOffSwitchPress();
  handleRGBSwitchPress();
  if(rgbState)
  {
    if (globalModes["modeInstance1"] == "Solid") { AllOn(color.r, color.g, color.b); }
    if (globalModes["modeInstance1"] == "Twinkle") { TwinklePixels(random(256), 255, 20, 50, 50); }
    if (globalModes["modeInstance1"] == "ShootingStar") { shootingStarAnimation(255, 255, 255, random(10, 60), random(5, 40), random(2000, 8000), 1); }
    if (globalModes["modeInstance1"] == "RainbowCycle") { rainbowCycle(20); }
    if (globalModes["modeInstance1"] == "Fade") { fadeAnimation(255, 255, 255); }
  }
  else
  {
    FastLED.clear();
    FastLED.show();
  }
}

void setupPins()
{
  pinMode(DIMLIGHT_PIN, OUTPUT);
  pinMode(ONOFFLIGHT_PIN, OUTPUT);
  pinMode(RGBLIGHT_PIN, OUTPUT);

  pinMode(0, OUTPUT);//Set D3 to LOW
  digitalWrite(0, LOW);

  pinMode(SW_DIMLIGHT_PIN, INPUT);
  pinMode(SW_ONOFFLIGHT_PIN, INPUT);
  pinMode(SW_RGBLIGHT_PIN, INPUT);
  

  FastLED.addLeds<WS2812B, RGBLIGHT_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 1000);
  FastLED.clear();
  FastLED.show();
}
//void handleOnOffSwitchPress(int buttonPin, bool powerState, int controlPin, String deviceID) 
void handleOnOffSwitchPress() 
{
  unsigned long actualMillis = millis();
  if (digitalRead(SW_ONOFFLIGHT_PIN) == HIGH && actualMillis - lastBtnPress > 1000)  {
    if (onOffState) {
      onOffState = LOW;
    } else {
      onOffState = HIGH;
    }
	
    // get Switch device back
    //SinricProSwitch& mySwitch = SinricPro[ONOFFLIGHT_ID];
    
    
	// send powerstate event to server
    //mySwitch.sendPowerStateEvent(onOffState); // send the new powerState to SinricPro server
    //Serial.printf("Device %s turned %s (manually via button)\r\n", mySwitch.getDeviceId().c_str(), onOffState?"on":"off");

    myOnOffSwitch.sendPowerStateEvent(onOffState); // send the new powerState to SinricPro server
    Serial.printf("Device %s turned %s (manually via button)\r\n", myOnOffSwitch.getDeviceId().c_str(), onOffState?"on":"off");

    // set relay status
	  digitalWrite(ONOFFLIGHT_PIN, onOffState ? HIGH:LOW);
	 
    lastBtnPress = actualMillis;  // update last button press variable
  } 
}

void handleDimSwitchPress() 
{
  unsigned long actualMillis = millis();
  if (digitalRead(SW_DIMLIGHT_PIN) == HIGH && actualMillis - lastBtnPress > 1000)  {
    if (dimState) {
      dimState = LOW;
    } else {
      dimState = HIGH;
    }
	
    // get Switch device back
    //SinricProSwitch& mySwitch = SinricPro[ONOFFLIGHT_ID];
    
    
	// send powerstate event to server
    //mySwitch.sendPowerStateEvent(onOffState); // send the new powerState to SinricPro server
    //Serial.printf("Device %s turned %s (manually via button)\r\n", mySwitch.getDeviceId().c_str(), onOffState?"on":"off");

    myDimSwitch.sendPowerStateEvent(dimState); // send the new powerState to SinricPro server
    Serial.printf("Device %s turned %s (manually via button)\r\n", myDimSwitch.getDeviceId().c_str(), dimState?"on":"off");

    // set relay status
	  digitalWrite(DIMLIGHT_PIN, dimState ? HIGH:LOW);
	 
    lastBtnPress = actualMillis;  // update last button press variable
  } 
}

void handleRGBSwitchPress() 
{
  unsigned long actualMillis = millis();
  if (digitalRead(SW_RGBLIGHT_PIN) == HIGH && actualMillis - lastBtnPress > 1000)  {
    if (rgbState) {
      rgbState = LOW;
    } else {
      rgbState = HIGH;
    }
	
    // get Switch device back
    //SinricProSwitch& mySwitch = SinricPro[ONOFFLIGHT_ID];
    
    
	// send powerstate event to server
    //mySwitch.sendPowerStateEvent(onOffState); // send the new powerState to SinricPro server
    //Serial.printf("Device %s turned %s (manually via button)\r\n", mySwitch.getDeviceId().c_str(), onOffState?"on":"off");

    rGBLED.sendPowerStateEvent(rgbState); // send the new powerState to SinricPro server
    Serial.printf("Device %s turned %s (manually via button)\r\n", rGBLED.getDeviceId().c_str(), rgbState?"on":"off");

    // set relay status
	  //digitalWrite(RGBLIGHT_PIN, rgbState ? HIGH:LOW);
    if(!rgbState)
    {
      FastLED.clear();
      FastLED.show();
    }
    else if (globalModes["modeInstance1"] == "Solid")
    {
      AllOn(color.r, color.g, color.b);
    }

	 
    lastBtnPress = actualMillis;  // update last button press variable
  } 
}