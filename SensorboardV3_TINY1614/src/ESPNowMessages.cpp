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

uint8_t BatteryBoardMessage::parseData(ESPNowBuffer *buffer)
{
    uint8_t index = BaseMessage::parseData(buffer);

    charging = buffer->getByte(index);
    index += 1;

    batteryVoltage = buffer->getDWord(index) / 100.0f;
    index += 4;

    return index;
}

uint8_t SensorBoardMessage::parseData(ESPNowBuffer *buffer)
{
    uint8_t index = BatteryBoardMessage::parseData(buffer);

    lux = buffer->getDWord(index) / 100.0f;
    index += 4;

    humidity = buffer->getDWord(index) / 100.0f;
    index += 4;

    temperature = buffer->getDWord(index) / 100.0f;
    index += 4;

    return index;
}

uint8_t SensorboardV2Message::parseData(ESPNowBuffer *buffer)
{
    uint8_t index = BatteryBoardMessage::parseData(buffer);

    temperature = ((float) buffer->getDWord(index)) / 100.0f;
    index += 4;

    humidity = buffer->getByte(index);
    index += 1;

    return index;
}

uint8_t ReedBoardMessage::parseData(ESPNowBuffer *buffer)
{
    uint8_t index = BatteryBoardMessage::parseData(buffer);

    state = buffer->getByte(index);
    index += 1;

    return index;
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

void SensorBoardMessage::writeData(ESPNowBuffer *buffer)
{
    BatteryBoardMessage::writeData(buffer);
    buffer->addDWord((uint32_t)(lux * 100.0));
    buffer->addDWord((uint32_t)(humidity * 100.0));
    buffer->addDWord((uint32_t)(temperature * 100.0));
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