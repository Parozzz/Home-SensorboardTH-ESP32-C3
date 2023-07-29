#ifndef ESP_NOW_MESSAGES
#define ESP_NOW_MESSAGES

#include <Arduino.h>

#include <ESPNowBuffer.h>
#include <SanityChecks.h>

enum BoardType
{
  INVALID_BOARD = 0,
  REED_BOARD = 2,
  SENSOR_BOARD = 3,
  KINETIC_SWITCH_BOARD = 200,
  TEST_BOARD = 255,
};

enum MessageType
{
  SENSOR_READING_MESSAGE = 1,
  ERROR_MESSAGE = 99,
};

class BaseMessage
{
public:
  BaseMessage(uint8_t boardType, uint8_t messageType)
  {
    this->boardType = boardType;
    this->_messageType = messageType;
  }

  uint8_t parseData(ESPNowBuffer *buffer);
  void writeData(ESPNowBuffer *buffer);
  bool sanityCheck();
  void printDebug(Print *print);

  uint8_t size()
  {
    sizeof(BaseMessage);
    return  sizeof(_messageType) +  sizeof(boardType) + sizeof(id);
  }

  uint8_t getMessageType()
  {
    return _messageType;
  }

  uint8_t id;
  uint8_t boardType;

private:
  uint8_t _messageType;
};

class BatteryBoardMessage : public BaseMessage
{
public:
  BatteryBoardMessage(uint8_t boardType, uint8_t messageType) : BaseMessage(boardType, messageType) {}

  uint8_t parseData(ESPNowBuffer *buffer);
  void writeData(ESPNowBuffer *buffer);
  bool sanityCheck();
  void printDebug(Print *print);

  uint8_t size()
  {
    return BaseMessage::size() + sizeof(charging) + sizeof(batteryVoltage);
  }

  uint8_t charging;
  float batteryVoltage;
};

class ErrorMessage : public BatteryBoardMessage
{
public:
  ErrorMessage(uint8_t boardType) : BatteryBoardMessage(boardType, MessageType::ERROR_MESSAGE) {}

  uint8_t parseData(ESPNowBuffer *buffer);
  void writeData(ESPNowBuffer *buffer);
  bool sanityCheck();
  void printDebug(Print *print);

  uint8_t size()
  {
    return BaseMessage::size() + sizeof(code1) + sizeof(code2) + sizeof(code2);
  }

  uint16_t code1;
  uint16_t code2;
  uint16_t code3;
};


class SensorboardV2Message : public BatteryBoardMessage
{
public:
  SensorboardV2Message() : BatteryBoardMessage(BoardType::SENSOR_BOARD, MessageType::SENSOR_READING_MESSAGE) {}

  uint8_t parseData(ESPNowBuffer *buffer);
  void writeData(ESPNowBuffer *buffer);
  bool sanityCheck();
  void printDebug(Print *print);

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
  ReedBoardMessage() : BatteryBoardMessage(BoardType::REED_BOARD, MessageType::SENSOR_READING_MESSAGE) {}

  uint8_t parseData(ESPNowBuffer *buffer);
  void writeData(ESPNowBuffer *buffer);
  bool sanityCheck();
  void printDebug(Print *print);

  uint8_t size()
  {
    return BatteryBoardMessage::size() + sizeof(state);
  }

  uint8_t state;
};
/*
bool ESPNowDoSanityCheck(ESPNowBuffer *buffer);

void ESPNowPrintDebug(ESPNowBuffer *buffer, Print *print);
*/
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