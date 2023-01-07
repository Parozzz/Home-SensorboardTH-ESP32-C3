#include <ESPNowMessages.h>

// ==============================
//  READ DATA TO ESPNOW BUFFER
// ==============================

uint8_t BaseMessage::parseData(ESPNowBuffer *buffer)
{
    uint8_t index = 1; // Start at 1 because the byte 0 is the board type the should be parsed before creating the class

    id = buffer->getByte(index);
    index += 1;

    return index;
}

#define BOARD_TYPE_COUNT 4
uint8_t boardTypes[BOARD_TYPE_COUNT] = {REED_BOARD_MESSAGE, SENSORBOARD_V2_MESSAGE, KINETIC_SWITCH, TESTBOARD};

bool BaseMessage::sanityCheck()
{
    bool typeFound = false;
    for (int x = 0; x < BOARD_TYPE_COUNT; x++)
    {
        if (_type == boardTypes[x])
        {
            typeFound = true;
        }
    }
    return typeFound;
}

void BaseMessage::printDebug(Print *print)
{
    print->printf("Base- Type= %d, ID = %d \n", _type, id);
}

uint8_t BatteryBoardMessage::parseData(ESPNowBuffer *buffer)
{
    uint8_t index = BaseMessage::parseData(buffer);

    charging = buffer->getByte(index);
    index += 1;

    batteryVoltage = buffer->getDWord(index) / 100.0f;
    index += 4;

    return index;
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

uint8_t SensorboardV2Message::parseData(ESPNowBuffer *buffer)
{
    uint8_t index = BatteryBoardMessage::parseData(buffer);

    temperature = ((float)buffer->getDWord(index)) / 100.0f;
    index += 4;

    humidity = buffer->getByte(index);
    index += 1;

    return index;
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

uint8_t ReedBoardMessage::parseData(ESPNowBuffer *buffer)
{
    uint8_t index = BatteryBoardMessage::parseData(buffer);

    state = buffer->getByte(index);
    index += 1;

    return index;
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
//  READ DATA TO ESPNOW BUFFER
// ==============================

// ==============================
//  WRITE DATA TO ESPNOW BUFFER
// ==============================
void BaseMessage::writeData(ESPNowBuffer *buffer)
{
    buffer->addByte(_type);
    buffer->addByte(id);
}

void BatteryBoardMessage::writeData(ESPNowBuffer *buffer)
{
    BaseMessage::writeData(buffer);
    buffer->addByte(charging);
    buffer->addDWord((uint32_t)(batteryVoltage * 100.0));
}

void SensorboardV2Message::writeData(ESPNowBuffer *buffer)
{
    BatteryBoardMessage::writeData(buffer);
    buffer->addDWord((uint32_t)(temperature * 100.0));
    buffer->addByte(humidity);
}

void ReedBoardMessage::writeData(ESPNowBuffer *buffer)
{
    BatteryBoardMessage::writeData(buffer);
    buffer->addByte(state);
}
// ==============================
//  WRITE DATA TO ESPNOW BUFFER
// ==============================

bool ESPNowDoSanityCheck(ESPNowBuffer *buffer)
{
    uint8_t type = buffer->getByte(0); // First byte is always the type. Second byte is the ID.
    if (type == BoardMessageType::REED_BOARD_MESSAGE)
    {
        ReedBoardMessage msg;
        msg.parseData(buffer);
        return msg.sanityCheck();
    }
    else if (type == BoardMessageType::SENSORBOARD_V2_MESSAGE)
    {
        SensorboardV2Message msg;
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
    if (type == BoardMessageType::REED_BOARD_MESSAGE)
    {
        ReedBoardMessage msg;
        msg.parseData(buffer);
        msg.printDebug(print);
    }
    else if (type == BoardMessageType::SENSORBOARD_V2_MESSAGE)
    {
        SensorboardV2Message msg;
        msg.parseData(buffer);
        msg.printDebug(print);
    }
}
