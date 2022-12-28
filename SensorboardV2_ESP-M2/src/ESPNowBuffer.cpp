#include <ESPNowBuffer.h>

#include <Arduino.h>

void ESPNowBuffer::setDataAmount(int len)
{
  _len = len + ESPNOW_BUFFER_CHECKSUM_BYTES;
}

void ESPNowBuffer::copyData(const uint8_t* data, int len)
{
  _len = len;
  memcpy(_buffer, data, _len);
}

uint32_t ESPNowBuffer::calculateChecksum()
{
  uint32_t checksum = 0;
  for (uint8_t x = 0; x < _len - ESPNOW_BUFFER_CHECKSUM_BYTES; x++)
  {
    checksum += _buffer[x];
  }

  uint32_t mask = 0;
  for (int x = 0; x < ESPNOW_BUFFER_CHECKSUM_BYTES; x++)
  {
    uint32_t shift = 8 * x;
    mask |= ((uint32_t)0xFF) << shift;
  }
  return checksum & mask;
}

void ESPNowBuffer::createAndSetChecksum()
{
  uint32_t checksum = calculateChecksum();

  for (int x = 0; x < ESPNOW_BUFFER_CHECKSUM_BYTES; x++)
  {
    uint8_t shift = 8 * x;
    uint32_t mask = ((uint32_t)0xFF) << shift;

    uint8_t byte = (uint8_t)((checksum & mask) >> shift);
    _buffer[_len - 1 - x] = byte; // MSB to LSB. B3 - B2 - B1 - B0
  }
}

uint32_t ESPNowBuffer::getChecksum()
{
  uint32_t checksum = 0;
  for (int x = 0; x < ESPNOW_BUFFER_CHECKSUM_BYTES; x++)
  {
    uint8_t shift = 8 * x;

    uint8_t byte = _buffer[_len - 1 - x];
    checksum |= ((uint32_t)byte) << shift;
  }
  return checksum;
}

bool ESPNowBuffer::isChecksumValid()
{
  return getChecksum() == calculateChecksum();
}

uint8_t ESPNowBuffer::getByte(uint16_t index)
{
  return _buffer[index];
}

uint16_t ESPNowBuffer::getWord(uint16_t index)
{
  uint16_t data = 0;
  data |= (_buffer[0] & 0xFF) << 8;
  data |= (_buffer[1] & 0xFF);
  return data;
}

uint32_t ESPNowBuffer::getDWord(uint16_t index)
{
  uint32_t data = 0;
  data |= (_buffer[0] & 0xFF) << 24;
  data |= (_buffer[1] & 0xFF) << 16;
  data |= (_buffer[2] & 0xFF) << 8;
  data |= (_buffer[3] & 0xFF);
  return data;
}

void ESPNowBuffer::setByte(uint16_t index, uint8_t data)
{
  _buffer[index] = data;
}

void ESPNowBuffer::setWord(uint16_t index, uint16_t data)
{
  _buffer[index] = (data >> 8) & 0xFF;
  _buffer[index + 1] = (data)&0xFF;
}

void ESPNowBuffer::setDWord(uint16_t index, uint32_t data)
{
  _buffer[index] = (data >> 24) & 0xFF;
  _buffer[index + 1] = (data >> 16) & 0xFF;
  _buffer[index + 2] = (data >> 8) & 0xFF;
  _buffer[index + 3] = (data)&0xFF;
}