#include "internalDrivers.h"
#include "Config.h"
#include "w5500.h"
#include <Wire.h>
#include <BH1750.h>

BH1750 lightMeter;

internalDrivers::internalDrivers() {
}

void internalDrivers::gpioInit() {
  pinMode(_ledTrggringMOSFETPin_1, OUTPUT);
  pinMode(_ledTrggringMOSFETPin_2, OUTPUT);
  pinMode(_ntcPin, INPUT);
  pinMode(_PIRSensorPin, INPUT);
  pinMode(_ethernetResetPin, OUTPUT);
}

void internalDrivers::BH1750Init() {
  Wire.begin();
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
}

void internalDrivers::ledAnalogTrigger(byte triggerValue) {
  analogWrite(_ledTrggringMOSFETPin_1, triggerValue);
  analogWrite(_ledTrggringMOSFETPin_2, triggerValue);
  if (triggerValue == 0) {
    loadStatus = false;
    Power = 0.0;
  } else {
    loadStatus = true;
    Power = readPower();
  }
  delay(100);  // delay  to reflect intensity at down
}

float internalDrivers::readPower() {
  return floatMap(ledAnalogTriggerValue, 0.0, float(analogPulseMaxDutyCycle), 0.0, 48.0);
}

float internalDrivers::readNTC() {
  unsigned int samples = 0, samplingRate = 5;
  for (byte i = 0; i < samplingRate; i++) {
    samples += analogRead(_ntcPin);
    delay(10);
  }
  int average = samples / samplingRate;
  float Temperature = (7.3149 * exp(0.0026 * average));  //  y = 7.3149e0.0026x
  return Temperature;
}

bool internalDrivers::checkIfTempOverThresAndGenrateAlarm(float temp) {
  return (temp > tempOverThreshold) ? true : false;
}

String internalDrivers::prepareDevID(byte mac[], String devPref) {
  char devID[15];
  snprintf(devID, sizeof(devID), "%s%02X%02X%02X", devPref.c_str(), mac[3], mac[4], mac[5]);
  return String(devID);
}

void internalDrivers::maintainAutoLuxAtFloor() {
  // if (autoBrightnessFlag) {
    float sensorReadingData = lightMeter.readLightLevel();
    calibartedFloorLux = 3.2596 * sensorReadingData - 20.288;  // y = 3.2596x - 20.288
    if (calibartedFloorLux < 0.0 || sensorReadingData < 0.0) {
      calibartedFloorLux = 0.0;
      Wire.begin();
      lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
    }

    static byte luxConfirmationCounter = 0;
    /*if floor lux less than expected lux then increase the brightness*/
    if (calibartedFloorLux < (userDefinedLux * 1.0)) {
      ledAnalogTriggerValue = constrain((ledAnalogTriggerValue + 2), 0, analogPulseMaxDutyCycle);
      /*if 25% more lux found at down then we will think about dim the light at max 20% of existing lux*/
    } else if (calibartedFloorLux >= (userDefinedLux * 1.15)) {
      luxConfirmationCounter++;
      if (luxConfirmationCounter > 1) {
        ledAnalogTriggerValue = constrain((ledAnalogTriggerValue - 2), 0, analogPulseMaxDutyCycle);
        luxConfirmationCounter = 0;
      }
    } else {
      delay(1);
    }
  // }   changes by Nilesh
}

void internalDrivers::writeOneByteInEEPROM(int Add, byte data) {
  EEPROM.write(Add, data);
  delay(10);
}

void internalDrivers::readDataFromEEPROM() {
  // twoTouchCounterStatus = EEPROM.read(twoTouchCounterEEPROMAdd);
  ledAnalogTriggerValue = EEPROM.read(ledIntensityMsgEEPROMAdd);
  autoBrightnessFlag = EEPROM.read(autobrightnessMsgEEPROMAdd);
  int tempIntLuxValue = EEPROM.read(userDefinedLuxIntEEPROMAdd);
  int tempFloatLuxValue = EEPROM.read(userDefinedLuxFloatEEPROMAdd);
  userDefinedLux = int((tempIntLuxValue + float(tempFloatLuxValue / 100.0)) * 8.0);
  Serial.print("ReadUserDefinedLux : ");
  Serial.println(userDefinedLux);
  globalTopicLength = EEPROM.read(globalTopicLenghtEEPROMAdd);
  String globalTopicTemp = loadStringFromEEPROM(globalTopicEEPROMAdd, globalTopicLength);
  if (globalTopicTemp != NULL) {
    globalTopic = globalTopicTemp;
  }
  byte analogPulseMaxDutyCycleTemp = EEPROM.read(maxledPWMIntensityMsgEEPROM);
  if (analogPulseMaxDutyCycleTemp != 0) {
    analogPulseMaxDutyCycle = analogPulseMaxDutyCycleTemp;
  }
  byte tempOverThresholdTemp = EEPROM.read(tempOverThresholdEEPROMAdd);
  if (tempOverThresholdTemp != 0) {
    tempOverThreshold = tempOverThresholdTemp;
  }
  byte motionCounterTemp = EEPROM.read(motionCounterMsgEEPROM);
  if (motionCounterTemp != 0) {
    motionCounter = motionCounterTemp;
  }
  int tempIntPIRThresholdValue = EEPROM.read(pirThresholdIntEEPROMAdd);
  int tempFloatPIRThresholdValue = EEPROM.read(pirThresholdFloatEEPROMAdd);
  PIR_THRESH = int((tempIntPIRThresholdValue + float(tempFloatPIRThresholdValue / 100.0)) * 20.0);
  autoMotionDetect_Flag = EEPROM.read(autoMotionDetectFlagMsgEEPROM);
}

float internalDrivers::floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void internalDrivers::storeStringInEEPROM(String data, byte Addr) {
  for (int i = 0; i < data.length(); i++) {
    EEPROM.write(Addr + i, data.charAt(i));
    delay(10);
  }
}

String internalDrivers::loadStringFromEEPROM(byte Addr, byte Length) {
  String readData = "";
  for (int i = Addr; i < (Addr + Length); i++) {
    readData += char(EEPROM.read(i));
    delay(10);
  }
  return readData;
}

void internalDrivers::motionDetectionUsingPIR() {
  // if (autoMotionDetect_Flag) { // change by Nilesh
  duration = pulseIn(_PIRSensorPin, HIGH);
  // +Serial.print("Duration : ");
  // +Serial.println(duration);
  motionConfirmedCounter += 1;
  // This is to omit the counter range overflow error
  if (motionConfirmedCounter >= 65500) {
    motionConfirmedCounter = motionCounter + 1;
  }
  float additionalMotionCount;
  if (ethernetLinkCheckFlag) {
    additionalMotionCount = 1.2;
  } else {
    additionalMotionCount = 1.0;
  }
  if ((duration > PIR_THRESH) || (motionConfirmedCounter <= (motionCounter * additionalMotionCount))) {
    if ((duration > PIR_THRESH)) {
      motionConfirmedCounter = 0;
    }
    ledAnalogTrigger(ledAnalogTriggerValue);
    // // +Serial.println("Motion Detected");
    delay(100);
  } else {
    ledAnalogTrigger(0);
    // // +Serial.println("No Motion Detected");
  }
  // } // change by Nilesh
}

SplitData internalDrivers::splitStringByColon(const String& data) {
  SplitData mqttMsg;
  int firstIndex = data.indexOf(':');
  if (firstIndex != -1) {
    mqttMsg.indexOneData = data.substring(0, firstIndex);
    int secondIndex = data.indexOf(':', firstIndex + 1);
    if (secondIndex != -1) {
      mqttMsg.indexTwoData = data.substring(firstIndex + 1, secondIndex);
      mqttMsg.indexThreeData = data.substring(secondIndex + 1);
      if (mqttMsg.indexThreeData.length() > 0) {
      }
    } else {
      mqttMsg.indexTwoData = data.substring(firstIndex + 1);
    }
  } else {
    mqttMsg.indexOneData = data.substring(firstIndex + 1);
  }
  return mqttMsg;
}

void internalDrivers::clearUsedEEPROM() {
  for (int i = 0; i < 512; i++) {
    if (i == 0) {
      EEPROM.write(i, ledAnalogTriggerValue);
    } else if (i == 71) {
      EEPROM.write(i, analogPulseMaxDutyCycle);
    } else {
      EEPROM.write(i, 0);
    }
    delay(10);
  }
}
