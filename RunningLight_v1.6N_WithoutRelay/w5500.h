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
  void Begin(byte EthMAC[], const char* MqttSever, int MqttPort, const char _ssPin);                                                                                                                // Initialize the Ethernet
  void MqttBegin(const char* MqttSever, int mqttPort);                                                                                                                                              // Initialize the mqtt
  bool ethernetLinkCheck();                                                                                                                                                                         // Checking the Ethernet Connection
  void reconnectToMqtt(String pubTopic, String subTopic, String globalTopic);                                                                                                                       //Trying to connect Mqtt Connection
  static void MQTT_Pull(char* topic, byte* payload, unsigned int length);                                                                                                                           // Mqtt Callback Function
  void decodeAndUpdateThresholdFromMqtt(String mqttMsg);                                                                                                                                            //Decoding Msg received from Mqtt And Update the data
  bool CheckMQTTConnection();                                                                                                                                                                       //Checking Mqtt Connection
  void clientLoop();                                                                                                                                                                                //Establish connection between device and mqtt, Also for publish and recive msg from mqtt
  void publishMqttMsg(String pubTopic, String devID, byte Intensity, bool devicestatus, float power, float temp, bool autoBrightness, bool autoMotionStatus, unsigned int lux, int pirSensorData);  //Publishing msg On Mqtt
  void publishMqttMsg_Alert(String pubTopic, String devID, String Alert);                                                                                                                           //Publishing Alert Msg On Mqtt
  subPubTopics createSubPubTopics(String devID, String SubTopic, String PubTopic, String globTopic);                                                                                                // Creating publish, subscribe and global Topic
  void subscribeTopics(String subsTopic);                                                                                                                                                           // Subscring the device Topic
  void unSubscribe(String subPubTopic);                                                                                                                                                             // UnSubsribe the device Topic
};

#endif
