#ifndef INTERNALDRIVERS_H
#define INTERNALDRIVERS_H

#include "Arduino.h"
#include <Wire.h>
#include <EEPROM.h>

struct SplitData {
  String indexOneData;
  String indexTwoData;
  String indexThreeData;
};

class internalDrivers {
public:
  internalDrivers();
  void gpioInit();                                               // initilization of GPIO
  void BH1750Init();                                             // initialization of Lux Sensor
  void ledAnalogTrigger(byte triggerValue);                      //function to change the value of PWM/intensity
  float readPower();                                             // Power calculation depending on the device
  float readNTC();                                               // reading temperature value from NTC
  bool checkIfTempOverThresAndGenrateAlarm(float temp);          // function to generate alarm in case threshold temperature is triggered
  String prepareDevID(byte mac[], String devPref);               // function to create device ID
  void maintainAutoLuxAtFloor();                                 // Only to get lux sensor readings
  void maintainLuxDependsNaturalLight(unsigned int naturalLux);  // Auto brightness function
  void writeOneByteInEEPROM(int Add, byte data);                 // Emprom writing function
  void readDataFromEEPROM();                                     // Reading data from eeprom
  // void twoTouchCounterUpdateAndStore(byte status);
  float floatMap(float x, float in_min, float in_max, float out_min, float out_max);  // function to map intensity
  void storeStringInEEPROM(String data, byte Addr);                                   // String values to store in eeprom such as global topics etc.
  String loadStringFromEEPROM(byte Addr, byte Length);                                // loading the string values from eeprom
  void motionDetectionUsingPIR();                                                     // function to detect motion from PIR Sensor
  SplitData splitStringByColon(const String& data);                                   // Split the values recevied from mqtt
  void clearUsedEEPROM();                                                             // function to clear eeprom
};

#endif
