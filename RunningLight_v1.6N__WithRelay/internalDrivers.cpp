#include "internalDrivers.h"
#include "Config.h"
#include "w5500.h"
#include <Wire.h>
#include <BH1750.h>

BH1750 lightMeter;

internalDrivers::internalDrivers() {
}

/*
 * functionName : gpioInit();
 * details: This function, gpioInit, is a member function of the internalDrivers class and is responsible for initializing specific GPIO. 
 */
void internalDrivers::gpioInit() {
  pinMode(_ledTrggringMOSFETPin_1, OUTPUT);
  pinMode(_ledTrggringMOSFETPin_2, OUTPUT);
  pinMode(_ntcPin, INPUT);
  pinMode(_PIRSensorPin, INPUT);
  pinMode(_ethernetResetPin, OUTPUT);
}

/*
 * functionName : BH1750Init();
 * details: This function, BH1750Init, is a member function of the internalDrivers class and is responsible for initializing Lux Sensor BH1750. 
 */
void internalDrivers::BH1750Init() {
  Wire.begin();
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
}

/*
 * functionName : ledAnalogTrigger(byte triggerValue);
 * Input : Passing the intensity in analog
 * details: This function, ledAnalogTrigger, is a member function of the internalDrivers class and is responsible for controlling the PWM. 
 */
void internalDrivers::ledAnalogTrigger(byte triggerValue) {
  // analogWrite(_ledTrggringMOSFETPin_1, triggerValue);
  analogWrite(_ledTrggringMOSFETPin_2, triggerValue);
  if (triggerValue == 0) {
    digitalWrite(_ledTrggringMOSFETPin_1, HIGH);
    loadStatus = false;
    Power = 0.0;
  } else {
    digitalWrite(_ledTrggringMOSFETPin_1, LOW);
    loadStatus = true;
    Power = readPower();
  }
  delay(100);  // delay  to reflect intensity at down
}

/*
 * functionName : readPower();
 * return Type : float
 * return Value : power in float
 * details: This function, readPower, is a member function of the internalDrivers class and is responsible measuring the power of the device. 
 */
float internalDrivers::readPower() {
  return floatMap(ledAnalogTriggerValue, 0.0, float(analogPulseMaxDutyCycle), 0.0, 48.0);
}

/*
 * functionName : readNTC();
 * return Type : float
 * return Value : temprature in float
 * details: This function, readNTC, is a member function of the internalDrivers class and is responsible for measuring the temprature of the device. 
 */
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

/*
 * functionName : checkIfTempOverThresAndGenrateAlarm(float temp);
 * input : passing the temprature
 * return Type : bool
 * return Value : true/false
 * details: This function, checkIfTempOverThresAndGenrateAlarm, is a member function of the internalDrivers class and is responsible for generating alert for themprature is over from threshold data. 
 */
bool internalDrivers::checkIfTempOverThresAndGenrateAlarm(float temp) {
  return (temp > tempOverThreshold) ? true : false;
}

/*
 * functionName : prepareDevID(byte mac[], String devPref);
 * input : passing the macAddress and device prefix name
 * return Type : string
 * return Value : device id in String
 * details: This function, prepareDevID, is a member function of the internalDrivers class and is responsible for creating device id. 
 */
String internalDrivers::prepareDevID(byte mac[], String devPref) {
  char devID[15];
  snprintf(devID, sizeof(devID), "%s%02X%02X%02X", devPref.c_str(), mac[3], mac[4], mac[5]);
  return String(devID);
}

/*
 * functionName : maintainAutoLuxAtFloor;
 * details: This function, maintainAutoLuxAtFloor, is a member function of the internalDrivers class and is responsible for reading the Lux Sensor Also calling the maintainLuxDependsNaturalLight function by passing the lux. 
 */
void internalDrivers::maintainAutoLuxAtFloor() {
  if (autoBrightnessFlag) {
    float sensorReadingData = lightMeter.readLightLevel();
    calibartedFloorLux = 3.2596 * sensorReadingData - 20.288;  // y = 3.2596x - 20.288
    if (calibartedFloorLux < 0.0 || sensorReadingData < 0.0) {
      calibartedFloorLux = 0.0;
      Wire.begin();
      lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
    }
    maintainLuxDependsNaturalLight(calibartedFloorLux);
  }
}

/*
 * functionName : maintainLuxDependsNaturalLight(unsigned int naturalLux);
 * input : lux data
 * details: This function, maintainLuxDependsNaturalLight, is a member function of the internalDrivers class and is responsible for autoBrightness according to userdefined lux. 
 */
void internalDrivers::maintainLuxDependsNaturalLight(unsigned int naturalLux) {
  if (naturalLux < userDefinedLux) {
    requiredintensityFromLight = ((float(userDefinedLux - naturalLux) / totalLuxOfLight) * 100) + additionalLuxInpercentage;  //calculating required intensity From Light In percentage And adding additionalLux To full fill the userDefined Lux
    requiredintensityFromLight = (requiredintensityFromLight > 100.00) ? 100.00 : requiredintensityFromLight;                 //checking if required intensity is greater then 100% then make it 100% else required intensity
    float brightness = map(requiredintensityFromLight, 0.0, 100.0, 0.0, analogPulseMaxDutyCycle);                             //Mapping the required intensity into analog data
    float stepValue = (float(analogPulseMaxDutyCycle) / 100) * (float)scaleStepForAutoBrightness;                             //Calculating the step scale in analog duty cycle // Step size is dynamically controlled // calculate step Value in analog
    float targetValue = ceil(stepValue * ceil(brightness / stepValue));                                                       //Calculating the target intensity in analog duty cycle according to required intensity with the help of step scale of autoBrightness

    if (ledAnalogTriggerValue < targetValue) {
      ledAnalogTriggerValue = constrain(ledAnalogTriggerValue + 1, 0, analogPulseMaxDutyCycle);
      luxConfirmationCounter = 0;
    } else if (ledAnalogTriggerValue > targetValue) {
      if (++luxConfirmationCounter > 10) {
        ledAnalogTriggerValue = constrain(ledAnalogTriggerValue - 1, 0, analogPulseMaxDutyCycle);
      }
    } else {
      //do Nothing
    }
  } else {
    // if (++luxConfirmationCounter > 5) {
    requiredintensityFromLight = 0.0;
    ledAnalogTriggerValue = constrain(ledAnalogTriggerValue - 1, 0, analogPulseMaxDutyCycle);
    // }
  }
}

/*
 * functionName : writeOneByteInEEPROM(int Add, byte data);
 * input : passing address and data
 * details: This function, writeOneByteInEEPROM, is a member function of the internalDrivers class and is responsible for write data of specific address in EEPROM. 
 */
void internalDrivers::writeOneByteInEEPROM(int Add, byte data) {
  EEPROM.write(Add, data);
  delay(10);
}

/*
 * functionName : readDataFromEEPROM();
 * details: This function, readDataFromEEPROM, is a member function of the internalDrivers class and is responsible for read data of specific address from EEPROM. 
 */
void internalDrivers::readDataFromEEPROM() {
  // twoTouchCounterStatus = EEPROM.read(twoTouchCounterEEPROMAdd);
  ledAnalogTriggerValue = EEPROM.read(ledIntensityMsgEEPROMAdd);
  autoBrightnessFlag = EEPROM.read(autobrightnessMsgEEPROMAdd);
  int tempIntLuxValue = EEPROM.read(userDefinedLuxIntEEPROMAdd);
  int tempFloatLuxValue = EEPROM.read(userDefinedLuxFloatEEPROMAdd);
  unsigned int tempUserDefinedLux = int((tempIntLuxValue + float(tempFloatLuxValue / 100.0)) * 8.0);
  if (tempUserDefinedLux != 0) {
    userDefinedLux = tempUserDefinedLux;
  }
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
  unsigned int PIR_THRESHTemp = int((tempIntPIRThresholdValue + float(tempFloatPIRThresholdValue / 100.0)) * 20.0);
  if (PIR_THRESHTemp != 0) {
    PIR_THRESH = PIR_THRESHTemp;
  }
  autoMotionDetect_Flag = EEPROM.read(autoMotionDetectFlagMsgEEPROM);
  byte tempscaleStepForAutoBrightness = EEPROM.read(scaleStepForAutoBrightnessEEPROMAdd);
  if (tempscaleStepForAutoBrightness != 0) {
    scaleStepForAutoBrightness = tempscaleStepForAutoBrightness;
  }
  int tempInttotalLuxOfLight = EEPROM.read(totalLightLuxIntEEPROMAdd);
  int tempFloattotalLuxOfLight = EEPROM.read(totalLightLuxFloatEEPROMAdd);
  unsigned int totalLuxOfLightTemp = int((tempInttotalLuxOfLight + float(tempFloattotalLuxOfLight / 100.0)) * 8.0);
  if (totalLuxOfLightTemp != 0) {
    totalLuxOfLight = totalLuxOfLightTemp;
  }
  byte additionalLuxInpercentageTemp = EEPROM.read(additionalLuxInpercentageEEPROMAdd);
  if (additionalLuxInpercentageTemp != 0) {
    additionalLuxInpercentage = additionalLuxInpercentageTemp;
  }
}

/*
 * functionName : floatMap(float x, float in_min, float in_max, float out_min, float out_max);
 * input : float x, float in_min, float in_max, float out_min, float out_max
 * return type : float 
 * details: This function, floatMap, is a member function of the internalDrivers class and is responsible for mapping the floating data. 
 */
float internalDrivers::floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
 * functionName : storeStringInEEPROM(String data, byte Addr);
 * input : data in string, address in byte
 * details: This function, storeStringInEEPROM, is a member function of the internalDrivers class and is responsible for storing the string data on specific address. 
 */
void internalDrivers::storeStringInEEPROM(String data, byte Addr) {
  for (int i = 0; i < data.length(); i++) {
    EEPROM.write(Addr + i, data.charAt(i));
    delay(10);
  }
}

/*
 * functionName : loadStringFromEEPROM(byte Addr, byte Length);
 * input : length in byte, address in byte
 * return type : String
 * details: This function, loadStringFromEEPROM, is a member function of the internalDrivers class and is responsible for loading data from specific address and return it. 
 */
String internalDrivers::loadStringFromEEPROM(byte Addr, byte Length) {
  String readData = "";
  for (int i = Addr; i < (Addr + Length); i++) {
    readData += char(EEPROM.read(i));
    delay(10);
  }
  return readData;
}

/*
 * functionName : motionDetectionUsingPIR();
 * details: This function, motionDetectionUsingPIR, is a member function of the internalDrivers class and is responsible for performing auto motion operation. 
 */
void internalDrivers::motionDetectionUsingPIR() {
  if (autoMotionDetect_Flag) {
    duration = pulseIn(_PIRSensorPin, HIGH);
    motionConfirmedCounter += 1;
    // This is to omit the counter range overflow error
    if (motionConfirmedCounter >= 4294967296) {
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
      // +Serial.println("Motion Detected");
      delay(100);
    } else {
      ledAnalogTrigger(0);
      // +Serial.println("No Motion Detected");
    }
  }
}

/*
 * functionName : splitStringByColon(const String& data);
 * input : data in string
 * return type : return structure of spliting data
 * details: This function, splitStringByColon, is a member function of the internalDrivers class and is responsible for split the string data from ':' and return the structure. 
 */
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

/*
 * functionName : clearUsedEEPROM();
 * details: This function, clearUsedEEPROM, is a member function of the internalDrivers class and is responsible for clearing the EEPROM. 
 */
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
