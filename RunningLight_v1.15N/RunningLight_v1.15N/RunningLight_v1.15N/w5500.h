#ifndef W5500_H_
#define W5500_H_

#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

struct subPubTopics {
  String Subscribe;
  String Publish;
  String Global;
};

class W5500 {

public:
  W5500();
  void Begin(byte EthMAC[], const char* MqttSever, int MqttPort, const char _ssPin);
  void MqttBegin(const char* MqttSever, int mqttPort);
  bool ethernetLinkCheck();
  void reconnectToMqtt(String subTopic, String globalTopic);
  static void MQTT_Pull(char* topic, byte* payload, unsigned int length);
  void decodeAndUpdateThresholdFromMqtt(String mqttMsg);
  bool CheckMQTTConnection();
  void clientLoop();
  void publishMqttMsg(String pubTopic, String devID, byte Intensity, bool devicestatus, float power, float temp, bool autoBrightness, bool autoMotionStatus, unsigned int lux, int pirSensorData);
  void publishMqttMsg_Alert(String pubTopic, String devID, String Alert);
  subPubTopics createSubPubTopics(String devID, String SubTopic, String PubTopic, String globTopic);
  void subscribeTopics(String subsTopic);
  void unSubscribe(String subPubTopic);
};

#endif
