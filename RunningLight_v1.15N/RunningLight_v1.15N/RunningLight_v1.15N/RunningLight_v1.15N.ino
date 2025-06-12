/*
   Name of Project: Running Light
   Author: Evoluzn India Pvt Ltd
   Developer: Nandakishor Dhote
   Controller Used: STM32
   Sensor Used: Lux Sensor, 10K Ntc, PIR Sensor
   Code Status: 
*/
#include "w5500.h"
#include "Config.h"
#include "internalDrivers.h"

W5500 w5500;
internalDrivers iDrivers;
subPubTopics Topic;

// unsigned long prevTime = 0;
// unsigned int DELAY = 900;

void setup() {
  // delay(3000);
  Serial.begin(9600);             // For debugging Purpose
  iDrivers.gpioInit();            // Initializing the GPIO
  iDrivers.readDataFromEEPROM();  // Reading The parameters From EEPROM
  iDrivers.BH1750Init();
  iDrivers.maintainAutoLuxAtFloor();
  /* reload the previus state of the load while automotion OFF*/
  if (!autoMotionDetect_Flag) {
    iDrivers.ledAnalogTrigger(ledAnalogTriggerValue);  // Initially trigger The LED in ON state
  } else {
    for (byte i = 0; i < 5; i++) {
      iDrivers.motionDetectionUsingPIR();
    }
  }
  // iDrivers.twoTouchCounterUpdateAndStore(twoTouchCounterStatus);
  w5500.Begin(MAC, ServerMQTT, MqttPort, _SSpin);
  deviceID = iDrivers.prepareDevID(MAC, devNamePrefix);
  Topic = w5500.createSubPubTopics(deviceID, subTopic, pubTopic, globalTopic);
  w5500.reconnectToMqtt(Topic.Subscribe, Topic.Global);  // establish the connections with Mqtt
  // +Serial.println(Topic.Subscribe);
}

void loop() {
  if (!autoMotionDetect_Flag) {
    iDrivers.ledAnalogTrigger(ledAnalogTriggerValue);
  } else {
    iDrivers.motionDetectionUsingPIR();
  }
  iDrivers.maintainAutoLuxAtFloor();
  float ntcTemperature = iDrivers.readNTC();
  bool overTempAlertFlag = iDrivers.checkIfTempOverThresAndGenrateAlarm(ntcTemperature);

  /* One Mints Millis loop */
  if (w5500.ethernetLinkCheck()) {
    ethernetReconnectCounterTimeout = 0;
    ethernetLinkCheckFlag = true;
    w5500.clientLoop();
    if (MqttRecdMsg != "") {
      w5500.decodeAndUpdateThresholdFromMqtt(MqttRecdMsg);
      MqttRecdMsg = "";
    }
    unsigned long currentTime = millis();
    if (currentTime - previousTime >= 60000 || responseOn200Request || alertMsg != "") {
      if (w5500.CheckMQTTConnection()) {
        mqttRestartCounter = 0;
        if (alertMsg != "") {
          w5500.publishMqttMsg_Alert(Topic.Publish, deviceID, alertMsg);
          alertMsg = "";
        } else if (responseOn200Request) {
          w5500.publishMqttMsg(Topic.Publish, "300", ledAnalogTriggerValue, loadStatus, Power, ntcTemperature, autoBrightnessFlag, autoMotionDetect_Flag, calibartedFloorLux, duration);
          responseOn200Request = false;
        } else {
          w5500.publishMqttMsg(Topic.Publish, deviceID, ledAnalogTriggerValue, loadStatus, Power, ntcTemperature, autoBrightnessFlag, autoMotionDetect_Flag, calibartedFloorLux, duration);
          if (overTempAlertFlag) {
            w5500.publishMqttMsg_Alert(Topic.Publish, deviceID, "Device Internal Temperature > " + String(tempOverThreshold));
            overTempAlertFlag = false;
          }
          previousTime = currentTime;
        }
      } else {
        w5500.reconnectToMqtt(Topic.Subscribe, Topic.Global);
        mqttRestartCounter++;
        if (mqttRestartCounter >= 200) {
          // // +Serial.println("mqttRestart");
          w5500.Begin(MAC, ServerMQTT, MqttPort, _SSpin);
          mqttRestartCounter = 0;
        }
      }
    }
  } else {
    ethernetLinkCheckFlag = false;
    ethernetReconnectCounterTimeout++;
    if (ethernetReconnectCounterTimeout >= 200) {  // Ethernet will reset after 10 Min of mqtt or ethernet failed
      digitalWrite(_ethernetResetPin, LOW);
      delay(500);  // Reset Pin should be hold low at least for 500Us
      digitalWrite(_ethernetResetPin, HIGH);
      delay(100);
      w5500.Begin(MAC, ServerMQTT, MqttPort, _SSpin);  // change by Nilesh
      ethernetReconnectCounterTimeout = 0;
      ethernetConnectivityFlag = true;
    }
  }
  delay(100);
}
