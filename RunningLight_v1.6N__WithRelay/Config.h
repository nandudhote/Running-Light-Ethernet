#ifndef CONFIG_H
#define CONFIG_H

#include "Arduino.h"

extern byte MAC[6];
extern char* ServerMQTT;
extern int MqttPort;
extern const char* mqttUserName;
extern const char* mqttUserPassword;

extern String devNamePrefix;
extern String deviceID;
extern String subTopic;  // devName + 6 digits of MAC + /Control
extern String pubTopic;  // // devName + 6 digits of MAC + /Status
extern String globalTopic;

extern byte analogPulseMaxDutyCycle;
extern const char _ledTrggringMOSFETPin_1;
extern const char _ledTrggringMOSFETPin_2;
extern const char _SSpin;
extern const char _ntcPin;
extern const char _ethernetResetPin;
extern const char _PIRSensorPin;

extern const byte ledIntensityMsgEEPROMAdd;
extern const byte autobrightnessMsgEEPROMAdd;
extern const byte userDefinedLuxIntEEPROMAdd;
extern const byte userDefinedLuxFloatEEPROMAdd;
// extern const byte twoTouchCounterEEPROMAdd;
extern const byte tempOverThresholdEEPROMAdd;
extern const byte globalTopicLenghtEEPROMAdd;
extern const byte globalTopicEEPROMAdd;
extern const byte maxledPWMIntensityMsgEEPROM;
extern const byte autoMotionDetectFlagMsgEEPROM;
extern const byte motionCounterMsgEEPROM;
extern const byte pirThresholdIntEEPROMAdd;
extern const byte pirThresholdFloatEEPROMAdd;
extern const byte scaleStepForAutoBrightnessEEPROMAdd;
extern const byte totalLightLuxIntEEPROMAdd;
extern const byte totalLightLuxFloatEEPROMAdd;
extern const byte additionalLuxInpercentageEEPROMAdd;

extern int ethernetResetCounter;
extern byte ledAnalogTriggerValue;
extern bool responseOn200Request;
extern bool autoBrightnessFlag;
extern bool ethernetLinkFlag;

extern unsigned int previousTime;
extern unsigned int userDefinedLux;
// extern byte twoTouchCounterStatus;
extern byte tempOverThreshold;

extern byte globalTopicLength;

extern String MqttRecdMsg;
extern bool autoMotionDetect_Flag;
extern byte loadStatus;

extern int duration;
extern unsigned int motionCounter;
extern unsigned int motionConfirmedCounter;
extern int PIR_THRESH;

extern float Power;
extern String alertMsg;
extern unsigned int calibartedFloorLux;

extern int mqttRestartCounter;

extern bool ethernetConnectivityFlag;
extern unsigned int ethernetReconnectCounterTimeout;
extern bool ethernetLinkCheckFlag;

extern byte luxConfirmationCounter;
extern float requiredintensityFromLight;
extern byte scaleStepForAutoBrightness;

extern unsigned int totalLuxOfLight;
extern byte additionalLuxInpercentage;

#endif
