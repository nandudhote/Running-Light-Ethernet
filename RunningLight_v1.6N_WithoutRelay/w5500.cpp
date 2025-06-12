#include "W5500.h"
#include "Config.h"
#include "String.h"
#include "internalDrivers.h"

W5500 iw5500;
internalDrivers iiDrivers;

EthernetServer server(80);
EthernetClient WLS_Client;
PubSubClient client(WLS_Client);

W5500::W5500() {
}

/*
 * functionName : Begin(byte EthMAC[], const char* MqttSever, int mqttPort, const char _ssPin);
 * input : byte EthMAC[], const char* MqttSever, int mqttPort, const char _ssPin
 * details: This function, Begin, is a member function of the W5500 class and is responsible for initialize the ethernet module. 
 */
void W5500::Begin(byte EthMAC[], const char* MqttSever, int mqttPort, const char _ssPin) {
  digitalWrite(_ethernetResetPin, HIGH);  //Enable ethernet module via reset pin
  Ethernet.init(_ssPin);
  /* At initially if ethernet is connected and in device on condition if ethernet try to connect for that we need to do ethernet begin for STM32 else link will not create*/
  Ethernet.begin(EthMAC);
  MqttBegin(MqttSever, mqttPort);
}

/*
 * functionName : MqttBegin(const char* MqttSever, int mqttPort);
 * input : const char* MqttSever, int mqttPort
 * details: This function, Begin, is a member function of the W5500 class and is responsible for initialize the mqtt. 
 */
void W5500::MqttBegin(const char* MqttSever, int mqttPort) {
  client.setServer(MqttSever, mqttPort);
  client.setCallback(MQTT_Pull);
}

/*
 * functionName : ethernetLinkCheck();
 * details: This function, ethernetLinkCheck, is a member function of the W5500 class and is responsible for checking the link of ethernet. 
 */
bool W5500::ethernetLinkCheck() {
  if (Ethernet.hardwareStatus()) {
    if (Ethernet.linkStatus() == LinkON) {
      /* At initially if ethernet not connected and in device on condition if ethernet try to connect for that we need to do ethernet begin for STM32 else link will not create*/
      if (ethernetLinkFlag) {
        Ethernet.begin(MAC);
        ethernetLinkFlag = false;
      }
      // Serial.println("ethernet is ok");
      // ethernetConnectivityFlag = true;
      return true;
    } else {
      // Serial.println("link failed");
      return false;
    }
  } else {
    // Serial.println("hardware fault");
    return false;
  }
}

/*
 * functionName : reconnectToMqtt(String pubTopic, String subTopic, String globalTopic);
 * input : String pubTopic, String subTopic, String globalTopic
 * details: This function, reconnectToMqtt, is a member function of the W5500 class and is responsible for connecting the device to the mqtt server. 
 */
void W5500::reconnectToMqtt(String pubTopic, String subTopic, String globalTopic) {
  String willMessage = "{" + deviceID + ":offline}";  // Last Will and Testament message
  if (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // if (client.connect(deviceID.c_str(), mqttUserName, mqttUserPassword)) {
    // if (client.connect(deviceID.c_str(), mqttUserName, mqttUserPassword, pubTopic.c_str(), 1, true, willMessage.c_str())) {
    // if (client.connect(deviceID.c_str())) {
    if (client.connect(deviceID.c_str(), pubTopic.c_str(), 1, true, willMessage.c_str())) {
      String onlineMessage = "{" + deviceID + ":online}";
      client.publish(pubTopic.c_str(), onlineMessage.c_str(), true);
      Serial.println("connected");
      subscribeTopics(subTopic);
      subscribeTopics(globalTopic);
    } else {
      delay(100);
    }
  }
}

/*
 * functionName : MQTT_Pull(char* topic, byte* payload, unsigned int length);
 * input : char* topic, byte* payload, unsigned int length
 * details: This function, MQTT_Pull, is a member function of the W5500 class and is responsible for receive data from mqtt. 
 */
void W5500::MQTT_Pull(char* topic, byte* payload, unsigned int length) {
  MqttRecdMsg = "";
  for (int i = 0; i < length; i++) {
    if (((char)payload[i] != ' ') && ((char)payload[i] != '\n')) {
      MqttRecdMsg += (char)payload[i];
    }
  }
}

/*
 * functionName : decodeAndUpdateThresholdFromMqtt(String mqttMsg)
 * input : String mqttMsg
 * details: This function, decodeAndUpdateThresholdFromMqtt, is a member function of the W5500 class and is responsible for updating the data from mqtt. 
 */
void W5500::decodeAndUpdateThresholdFromMqtt(String mqttMsg) {
  SplitData mqttDecodedMsg = iiDrivers.splitStringByColon(mqttMsg);
  if (mqttDecodedMsg.indexOneData == "autoBrightness") {
    autoBrightnessFlag = (mqttDecodedMsg.indexTwoData).toInt();                      // its a flag
    unsigned int userDefindluxOrBrightness = mqttDecodedMsg.indexThreeData.toInt();  // get actual value

    iiDrivers.writeOneByteInEEPROM(autobrightnessMsgEEPROMAdd, autoBrightnessFlag);
    if (autoBrightnessFlag) {
      if (userDefindluxOrBrightness != userDefinedLux) {
        userDefinedLux = userDefindluxOrBrightness;
        float tempIntLuxValue = userDefinedLux / 8.0;
        int tempFloatLuxValue = (tempIntLuxValue - int(tempIntLuxValue)) * 100;
        iiDrivers.writeOneByteInEEPROM(userDefinedLuxIntEEPROMAdd, int(tempIntLuxValue));
        iiDrivers.writeOneByteInEEPROM(userDefinedLuxFloatEEPROMAdd, tempFloatLuxValue);
      }
    } else {
      userDefindluxOrBrightness = (userDefindluxOrBrightness > 100) ? 100 : userDefindluxOrBrightness;
      ledAnalogTriggerValue = map(userDefindluxOrBrightness, 0, 100, 0, analogPulseMaxDutyCycle);  // % to byte range conversion
      iiDrivers.writeOneByteInEEPROM(ledIntensityMsgEEPROMAdd, ledAnalogTriggerValue);
    }
  }
  if (mqttDecodedMsg.indexOneData == "maxAnalogPulseDutyCycle") {
    byte brightnessInPer = ceil(iiDrivers.floatMap(ledAnalogTriggerValue, 0.0, float(analogPulseMaxDutyCycle), 0.0, 100.0));  // For Proper Conversion of analogData to percentage
    analogPulseMaxDutyCycle = ((mqttDecodedMsg.indexTwoData).toInt() > 255) ? 255 : (mqttDecodedMsg.indexTwoData).toInt();
    ledAnalogTriggerValue = map(brightnessInPer, 0, 100, 0, analogPulseMaxDutyCycle);
    iiDrivers.writeOneByteInEEPROM(maxledPWMIntensityMsgEEPROM, analogPulseMaxDutyCycle);
    iiDrivers.writeOneByteInEEPROM(ledIntensityMsgEEPROMAdd, ledAnalogTriggerValue);
    alertMsg = "maxAnalogPulseDutyCycleUpdated: " + String(analogPulseMaxDutyCycle);
  }
  if (mqttDecodedMsg.indexOneData == "updateMqttTopic") {
    if (mqttDecodedMsg.indexThreeData.length() < 20) {
      if (mqttDecodedMsg.indexTwoData == "globalTopic") {
        iw5500.unSubscribe(globalTopic + subTopic);
        globalTopic = mqttDecodedMsg.indexThreeData;
        iw5500.subscribeTopics(globalTopic + subTopic);
        iiDrivers.writeOneByteInEEPROM(globalTopicLenghtEEPROMAdd, globalTopic.length());
        iiDrivers.storeStringInEEPROM(globalTopic, globalTopicEEPROMAdd);
        alertMsg = "GlobalTopicUpdated: " + globalTopic;
      }
    } else {
      alertMsg = "TopicLenghtShouldBeUpto20Chars";
    }
  }
  if (mqttDecodedMsg.indexOneData == "tempOverThreshold") {
    tempOverThreshold = (mqttDecodedMsg.indexTwoData).toInt();
    iiDrivers.writeOneByteInEEPROM(tempOverThresholdEEPROMAdd, tempOverThreshold);
    alertMsg = "tempOverThresholdValueUpdated: " + String(tempOverThreshold);
  }
  if (mqttDecodedMsg.indexOneData == "autoMotionDetection") {
    if (mqttDecodedMsg.indexTwoData == "100") {
      autoMotionDetect_Flag = true;
      iiDrivers.writeOneByteInEEPROM(autoMotionDetectFlagMsgEEPROM, autoMotionDetect_Flag);
      alertMsg = "autoMotionActivate";
    } else if (mqttDecodedMsg.indexTwoData == "0") {
      autoMotionDetect_Flag = false;
      iiDrivers.writeOneByteInEEPROM(autoMotionDetectFlagMsgEEPROM, autoMotionDetect_Flag);
      alertMsg = "autoMotionDeactivate";
    }
  }
  if (mqttDecodedMsg.indexOneData == "motionCounter") {
    motionCounter = (mqttDecodedMsg.indexTwoData).toInt();
    iiDrivers.writeOneByteInEEPROM(motionCounterMsgEEPROM, motionCounter);
    alertMsg = "motionCounterUpdated: " + String(motionCounter);
  }
  if (mqttDecodedMsg.indexOneData == "PIRThreshold") {
    PIR_THRESH = (mqttDecodedMsg.indexTwoData).toInt();
    float tempIntPIRThresholdValue = PIR_THRESH / 20.0;
    int tempFloatPIRThresholdValue = (tempIntPIRThresholdValue - int(tempIntPIRThresholdValue)) * 100;
    iiDrivers.writeOneByteInEEPROM(pirThresholdIntEEPROMAdd, int(tempIntPIRThresholdValue));
    iiDrivers.writeOneByteInEEPROM(pirThresholdFloatEEPROMAdd, tempFloatPIRThresholdValue);
    alertMsg = "PIRThresholdUpdated: " + String(PIR_THRESH);
  }
  if (mqttDecodedMsg.indexOneData == "autoBrightnessStep") {
    scaleStepForAutoBrightness = (mqttDecodedMsg.indexTwoData).toInt();
    iiDrivers.writeOneByteInEEPROM(scaleStepForAutoBrightnessEEPROMAdd, scaleStepForAutoBrightness);
    alertMsg = "autoBrightnessStepUpdated: " + String(scaleStepForAutoBrightness);
  }
  if (mqttDecodedMsg.indexOneData == "totalLuxOfLight") {
    totalLuxOfLight = (mqttDecodedMsg.indexTwoData).toInt();
    float tempInttotalLuxOfLight = totalLuxOfLight / 8.0;
    int tempFloattotalLuxOfLight = (tempInttotalLuxOfLight - int(tempInttotalLuxOfLight)) * 100;
    iiDrivers.writeOneByteInEEPROM(totalLightLuxIntEEPROMAdd, int(tempInttotalLuxOfLight));
    iiDrivers.writeOneByteInEEPROM(totalLightLuxFloatEEPROMAdd, tempFloattotalLuxOfLight);
    alertMsg = "totalLuxOfLightUpdated: " + String(totalLuxOfLight);
  }
  if (mqttDecodedMsg.indexOneData == "AdditionalLux") {
    unsigned int addingLuxInUserDefinedLux = (mqttDecodedMsg.indexTwoData).toInt();
    additionalLuxInpercentage = ((float)addingLuxInUserDefinedLux / totalLuxOfLight) * 100;
    iiDrivers.writeOneByteInEEPROM(additionalLuxInpercentageEEPROMAdd, additionalLuxInpercentage);
    alertMsg = "AdditionalLuxUpdated: " + String(addingLuxInUserDefinedLux);
  }
  if (mqttDecodedMsg.indexOneData == "storeStatus") {
    alertMsg = globalTopic + ":" + String(tempOverThreshold) + ":" + String(analogPulseMaxDutyCycle) + ":" + String(motionCounter) + ":" + String(PIR_THRESH) + ":" + String(userDefinedLux) + ":" + String(scaleStepForAutoBrightness) + ":" + String(totalLuxOfLight) + ":" + String(additionalLuxInpercentage) + ":" + String(requiredintensityFromLight);
  }
  if (mqttDecodedMsg.indexOneData == "200") {
    responseOn200Request = true;
  }
  if (mqttDecodedMsg.indexOneData == "restart") {
    NVIC_SystemReset();
  }
  if (mqttDecodedMsg.indexOneData == "EEPROMClear") {
    iiDrivers.clearUsedEEPROM();
  }
}

/*
 * functionName : CheckMQTTConnection();
 * return type : bool
 * details: This function, CheckMQTTConnection, is a member function of the W5500 class and is responsible for checking the mqtt connection. 
 */
bool W5500::CheckMQTTConnection() {
  if (client.connected()) {
    return true;
  }
  return false;
}

/*
 * functionName : clientLoop();
 * details: This function, CheckMQTTConnection, is a member function of the W5500 class and is responsible for establish the communication between device and mqtt. 
 */
void W5500::clientLoop() {
  client.loop();
}

/*
 * functionName : publishMqttMsg(String pubTopic, String devID, byte Intensity, bool devicestatus, float power, float temp, bool autoBrightness, bool autoMotionStatus, unsigned int lux, int pirSensorData)
 * input : String pubTopic, String devID, byte Intensity, bool devicestatus, float power, float temp, bool autoBrightness, bool autoMotionStatus, unsigned int lux, int pirSensorData
 * details: This function, publishMqttMsg, is a member function of the W5500 class and is responsible for publishing data on mqtt. 
 */
void W5500::publishMqttMsg(String pubTopic, String devID, byte Intensity, bool devicestatus, float power, float temp, bool autoBrightness, bool autoMotionStatus, unsigned int lux, int pirSensorData) {
  byte brightness = ceil(iiDrivers.floatMap(Intensity, 0.0, float(analogPulseMaxDutyCycle), 0.0, 100.0));
  String Final = "{device_id:" + devID + ":" + String(brightness) + ":" + String(devicestatus) + ":" + String(power) + ":" + String(temp) + ":" + String(autoBrightness) + ":" + String(autoMotionStatus) + ":" + String(lux) + ":" + String(pirSensorData) + "}";
  client.publish(pubTopic.c_str(), Final.c_str());
}

/*
 * functionName : publishMqttMsg_Alert(String pubTopic, String devID, String Alert)
 * input : String pubTopic, String devID, String Alert
 * details: This function, publishMqttMsg_Alert, is a member function of the W5500 class and is responsible for publishing alert Msg on mqtt. 
 */
void W5500::publishMqttMsg_Alert(String pubTopic, String devID, String Alert) {
  String Final = "{device_id:" + devID + ":" + Alert + "}";
  client.publish(pubTopic.c_str(), Final.c_str());
}

/*
 * functionName : createSubPubTopics(String devID, String SubTopic, String PubTopic, String globTopic)
 * input : String devID, String SubTopic, String PubTopic, String globTopic
 * return type : return the structure 
 * details: This function, createSubPubTopics, is a member function of the W5500 class and is responsible for creating publish, subscribe and global topic. 
 */
subPubTopics W5500::createSubPubTopics(String devID, String SubTopic, String PubTopic, String globTopic) {
  subPubTopics Topics;
  Topics.Subscribe = devID + SubTopic;
  Topics.Publish = devID + PubTopic;
  Topics.Global = globalTopic + SubTopic;
  return Topics;
}

/*
 * functionName : subscribeTopics(String subsTopic)
 * input : String subsTopic 
 * details: This function, subscribeTopics, is a member function of the W5500 class and is responsible for creating subscribing the topic. 
 */
void W5500::subscribeTopics(String subsTopic) {
  client.subscribe(subsTopic.c_str());
}

/*
 * functionName : unSubscribe(String subPubTopic)
 * input : String subPubTopic 
 * details: This function, unSubscribe, is a member function of the W5500 class and is responsible for creating unSubscribing the topic. 
 */
void W5500::unSubscribe(String subPubTopic) {
  client.unsubscribe(subPubTopic.c_str());
}
