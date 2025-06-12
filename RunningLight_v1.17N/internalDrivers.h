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
  void gpioInit();
  void BH1750Init();
  void ledAnalogTrigger(byte triggerValue);
  float readPower();
  float readNTC();
  bool checkIfTempOverThresAndGenrateAlarm(float temp);
  String prepareDevID(byte mac[], String devPref);
  void maintainAutoLuxAtFloor();
  void writeOneByteInEEPROM(int Add, byte data);
  void readDataFromEEPROM();
  // void twoTouchCounterUpdateAndStore(byte status);
  float floatMap(float x, float in_min, float in_max, float out_min, float out_max);
  void storeStringInEEPROM(String data, byte Addr);
  String loadStringFromEEPROM(byte Addr, byte Length);
  void motionDetectionUsingPIR();
  SplitData splitStringByColon(const String& data);
  void clearUsedEEPROM();
};

#endif
