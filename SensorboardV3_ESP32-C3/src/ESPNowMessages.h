#ifndef ESP_NOW_MESSAGES
#define ESP_NOW_MESSAGES

#include <Arduino.h>

#include <ESPNowBuffer.h>
#include <SanityChecks.h>

enum BoardMessageType
{
  REED_BOARD_MESSAGE = 2,
  SENSORBOARD_V2_MESSAGE = 3,
  KINETIC_SWITCH = 200,
  TESTBOARD = 255,
};

class BaseMessage
{
public:
  BaseMessage(uint8_t type)
  {
    _type = type;
  }

  uint8_t parseData(ESPNowBuffer *buffer);
  void writeData(ESPNowBuffer *buffer);
  bool sanityCheck();
  void printDebug(Print* print);

  uint8_t size()
  {
    sizeof(BaseMessage);
    return sizeof(_type) + sizeof(id);
  }

  uint8_t getType()
  {
    return _type;
  }

  uint8_t id;

private:
  uint8_t _type;
};

class BatteryBoardMessage : public BaseMessage
{
public:
  BatteryBoardMessage(uint8_t type) : BaseMessage(type) {}

  uint8_t parseData(ESPNowBuffer *buffer);
  void writeData(ESPNowBuffer *buffer);
  bool sanityCheck();
  void printDebug(Print* print);

  uint8_t size()
  {
    return BaseMessage::size() + sizeof(charging) + sizeof(batteryVoltage);
  }

  uint8_t charging;
  float batteryVoltage;
};

class SensorboardV2Message : public BatteryBoardMessage
{
public:
  SensorboardV2Message() : BatteryBoardMessage(BoardMessageType::SENSORBOARD_V2_MESSAGE) {}

  uint8_t parseData(ESPNowBuffer *buffer);
  void writeData(ESPNowBuffer *buffer);
  bool sanityCheck();
  void printDebug(Print* print);

  uint8_t size()
  {
    return BatteryBoardMessage::size() + sizeof(temperature) + sizeof(humidity);
  }

  float temperature;
  uint8_t humidity;
};

class ReedBoardMessage : public BatteryBoardMessage
{
public:
  ReedBoardMessage() : BatteryBoardMessage(BoardMessageType::REED_BOARD_MESSAGE) {}

  uint8_t parseData(ESPNowBuffer *buffer);
  void writeData(ESPNowBuffer *buffer);
  bool sanityCheck();
  void printDebug(Print* print);

  uint8_t size()
  {
    return BatteryBoardMessage::size() + sizeof(state);
  }

  uint8_t state;
};

bool ESPNowDoSanityCheck(ESPNowBuffer *buffer);

void ESPNowPrintDebug(ESPNowBuffer* buffer, Print* print);

#endif

/*
class SensorBoardMessage : public BatteryBoardMessage
{
public:
  SensorBoardMessage() : BatteryBoardMessage(BoardMessageType::SENSOR_BOARD_MESSAGE) {}

  uint8_t parseData(ESPNowBuffer *buffer);
  void writeData(ESPNowBuffer *buffer);

  uint8_t size()
  {
    return BatteryBoardMessage::size() + sizeof(lux) + sizeof(temperature) + sizeof(humidity);
  }

  float lux;
  float temperature;
  float humidity;
};
*/