#include "Config.h"

byte MAC[6] = { 0x8C, 0x4C, 0xAD, 0xF0, 0xBF, 0x30 };
// char* ServerMQTT = "evoluzn.org";
// char* ServerMQTT = "192.168.29.14";
char* ServerMQTT = "203.109.124.70";
// int MqttPort = 1883;
int MqttPort = 18889;
const char* mqttUserName = "evzin_led";
const char* mqttUserPassword = "63I9YhMaXpa49Eb";


String devNamePrefix = "tube";
String deviceID = "";
String subTopic = "/control";  // devName + 6 digits of MAC + /Control
String pubTopic = "/status";   // // devName + 6 digits of MAC + /Status
String globalTopic = "tubeGlobal";

const char _ledTrggringMOSFETPin_1 = PA10;
const char _ledTrggringMOSFETPin_2 = PA9;
const char _SSpin = PB10;
const char _ntcPin = PA2;
const char _ethernetResetPin = PB12;
const char _PIRSensorPin = PA1;
// const char _PIRSensorPin = PB3;

const byte ledIntensityMsgEEPROMAdd = 0;
const byte autobrightnessMsgEEPROMAdd = 1;
const byte userDefinedLuxIntEEPROMAdd = 2;
const byte userDefinedLuxFloatEEPROMAdd = 3;
// const byte twoTouchCounterEEPROMAdd = 4;
const byte tempOverThresholdEEPROMAdd = 5;
const byte globalTopicLenghtEEPROMAdd = 6;
const byte globalTopicEEPROMAdd = 7;  // Global topic max length should of 23 bits
const byte maxledPWMIntensityMsgEEPROM = 30;
const byte autoMotionDetectFlagMsgEEPROM = 31;
const byte motionCounterMsgEEPROM = 32;
const byte pirThresholdIntEEPROMAdd = 33;
const byte pirThresholdFloatEEPROMAdd = 34;

int ethernetResetCounter = 0;
byte analogPulseMaxDutyCycle = 204;
byte ledAnalogTriggerValue = analogPulseMaxDutyCycle;
bool responseOn200Request = false;
bool autoBrightnessFlag = true;
bool ethernetLinkFlag = true;

unsigned int previousTime = 0;
unsigned int userDefinedLux = 1400;
// byte twoTouchCounterStatus = 0;
byte tempOverThreshold = 80;  // This varibale should be upgradable by MQTT and should be stored in EEPROM

byte globalTopicLength = 0;

String MqttRecdMsg = "";
bool autoMotionDetect_Flag = true;
byte loadStatus = false;

int duration = 0;
unsigned int motionCounter = 25;
unsigned int motionConfirmedCounter = motionCounter;
int PIR_THRESH = 200;  // PIR threshold value upto 2000

float Power = 0.0;
String alertMsg = "";
unsigned int calibartedFloorLux = 0;

int mqttRestartCounter = 0;

bool ethernetConnectivityFlag = true;
unsigned int ethernetReconnectCounterTimeout = 0;
bool ethernetLinkCheckFlag = false;
