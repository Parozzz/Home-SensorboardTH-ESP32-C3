#include <ESPNowMessages.h>

// ==============================
//  BASE MESSAGE
uint8_t BaseMessage::parseData(ESPNowBuffer *buffer)
{
    uint8_t index = 1; // Start at 1 because the Byte0=MessageType and it is required at creation.

    boardType = buffer->getByte(index);
    index += 1;

    id = buffer->getByte(index);
    index += 1;

    return index;
}

void BaseMessage::writeData(ESPNowBuffer *buffer)
{
    buffer->addByte(_messageType);
    buffer->addByte(boardType);
    buffer->addByte(id);
}

bool BaseMessage::sanityCheck()
{
    return boardType > 0 && _messageType > 0;
}

void BaseMessage::printDebug(Print *print)
{
    print->printf("Base- Board= %d, Message= %d, ID = %d \n", boardType, _messageType, id);
}
// ==============================

// ==============================
//  BATTERY BOARD MESSAGE
uint8_t BatteryBoardMessage::parseData(ESPNowBuffer *buffer)
{
    uint8_t index = BaseMessage::parseData(buffer);

    charging = buffer->getByte(index);
    index += 1;

    batteryVoltage = buffer->getDWord(index) / 100.0f;
    index += 4;

    return index;
}

void BatteryBoardMessage::writeData(ESPNowBuffer *buffer)
{
    BaseMessage::writeData(buffer);
    buffer->addByte(charging);
    buffer->addDWord((uint32_t)(batteryVoltage * 100.0));
}

bool BatteryBoardMessage::sanityCheck()
{
    bool sanity = BaseMessage::sanityCheck();
    sanity = sanity && batteryVoltage < SANITY_MAX_BATTERY_VOLTAGE && batteryVoltage > SANITY_MIN_BATTERY_VOLTAGE;
    return sanity;
}

void BatteryBoardMessage::printDebug(Print *print)
{
    BaseMessage::printDebug(print);
    print->print("Battery- Battery= ");
    print->print(batteryVoltage);
    print->printf(", Charging= %d \n", charging);
}
// ==============================

// ==============================
//  ERROR MESSAGE
uint8_t ErrorMessage::parseData(ESPNowBuffer *buffer)
{
    uint8_t index = BatteryBoardMessage::parseData(buffer);

    code1 = buffer->getWord(index);
    index += 2;

    code2 = buffer->getWord(index);
    index += 2;

    code3 = buffer->getWord(index);
    index += 2;

    return index;
}

void ErrorMessage::writeData(ESPNowBuffer *buffer)
{
    BatteryBoardMessage::writeData(buffer);
    buffer->addWord(code1);
    buffer->addWord(code2);
    buffer->addWord(code3);
}

bool ErrorMessage::sanityCheck()
{
    return BatteryBoardMessage::sanityCheck();
}

void ErrorMessage::printDebug(Print *print)
{
    BatteryBoardMessage::printDebug(print);
    print->printf("Error- code1= %d, code2= %d, code3= %d\n", code1, code2, code3);
}
// ==============================

// ==============================
//  SENSORBOARD V2 MESSAGE
uint8_t SensorboardV2Message::parseData(ESPNowBuffer *buffer)
{
    uint8_t index = BatteryBoardMessage::parseData(buffer);

    temperature = buffer->getDWord(index) / 100.0f;
    index += 4;

    humidity = buffer->getByte(index);
    index += 1;

    return index;
}

void SensorboardV2Message::writeData(ESPNowBuffer *buffer)
{
    BatteryBoardMessage::writeData(buffer);
    buffer->addDWord((uint32_t)(temperature * 100.0));
    buffer->addByte(humidity);
}

bool SensorboardV2Message::sanityCheck()
{
    bool sanity = BatteryBoardMessage::sanityCheck();
    sanity = sanity && temperature < SANITY_MAX_TEMPERATURE && temperature > SANITY_MIN_TEMPERATURE;
    sanity = sanity && humidity < SANITY_MAX_HUMIDITY && humidity > SANITY_MIN_HUMIDITY;
    return sanity;
}

void SensorboardV2Message::printDebug(Print *print)
{
    BatteryBoardMessage::printDebug(print);
    print->print("SensorboardV2- Temp= ");
    print->print(temperature);
    print->printf(", Humidity= %d \n", humidity);
}
// ==============================

// ==============================
//  REED BOARD MESSAGE
uint8_t ReedBoardMessage::parseData(ESPNowBuffer *buffer)
{
    uint8_t index = BatteryBoardMessage::parseData(buffer);

    state = buffer->getByte(index);
    index += 1;

    return index;
}

void ReedBoardMessage::writeData(ESPNowBuffer *buffer)
{
    BatteryBoardMessage::writeData(buffer);
    buffer->addByte(state);
}

bool ReedBoardMessage::sanityCheck()
{
    bool sanity = BatteryBoardMessage::sanityCheck();
    return sanity;
}

void ReedBoardMessage::printDebug(Print *print)
{
    BatteryBoardMessage::printDebug(print);
    print->printf("ReedBoard- State= %d \n", state);
}
// ==============================

// ==============================
//  OTHERS
/*
bool ESPNowDoSanityCheck(ESPNowBuffer *buffer)
{
    uint8_t type = buffer->getByte(0); // First byte is always the type. Second byte is the ID.
    if (type == BoardType::REED_BOARD_MESSAGE)
    {
        ReedBoardMessage msg;
        msg.parseData(buffer);
        return msg.sanityCheck();
    }
    else if (type == BoardType::SENSORBOARD_V2_MESSAGE)
    {
        SensorboardV2Message msg;
        msg.parseData(buffer);
        return msg.sanityCheck();
    }
    else if (type == BoardType::ERROR_MESSAGE)
    {
        ErrorMessage msg;
        msg.parseData(buffer);
        return msg.sanityCheck();
    }
    else
    {
        return false;
    }
}

void ESPNowPrintDebug(ESPNowBuffer *buffer, Print *print)
{
    uint8_t type = buffer->getByte(0); // First byte is always the type. Second byte is the ID.
    if (type == BoardType::REED_BOARD_MESSAGE)
    {
        ReedBoardMessage msg;
        msg.parseData(buffer);
        msg.printDebug(print);
    }
    else if (type == BoardType::SENSORBOARD_V2_MESSAGE)
    {
        SensorboardV2Message msg;
        msg.parseData(buffer);
        msg.printDebug(print);
    }
    else if (type == BoardType::ERROR_MESSAGE)
    {
        ErrorMessage msg;
        msg.parseData(buffer);
        msg.printDebug(print);
    }
    else
    {
        print->printf("Message not found with type= %d \n", type);
    }
}*/
// ==============================
