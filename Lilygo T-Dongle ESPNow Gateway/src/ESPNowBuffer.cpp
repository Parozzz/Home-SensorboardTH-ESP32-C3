#include <ESPNowBuffer.h>
#include <Arduino.h>

void ESPNowBuffer::clear()
{
  _len = 0;
  _checksumAdded = false;
  _inUse = false;
}

void ESPNowBuffer::copyData(const uint8_t *data, int len)
{
  _len = len;
  memcpy(_buffer, data, _len);

  _checksumAdded = true; 
}

uint8_t ESPNowBuffer::createChecksum()
{
  uint8_t checksum = 0;
  for (uint8_t x = 0; x < (_checksumAdded ? _len -1 : _len); x++)
  {
    checksum = checksum ^ _buffer[x];
  }
  checksum = (checksum ^ 0xFF) + 1;
  return checksum;
}

uint8_t ESPNowBuffer::updateChecksum()
{
  uint8_t checksum = createChecksum();
  if (_checksumAdded)
  {
    this->setByte(_len - 1, checksum);
  }
  else
  {
    _checksumAdded = true;
    this->addByte(checksum);
  }

  return checksum;
}

uint8_t ESPNowBuffer::getChecksum()
{
  return _checksumAdded ? this->getByte(_len - 1) : 0x00;
}

bool ESPNowBuffer::isChecksumValid()
{
  return getChecksum() == createChecksum();
}

uint8_t ESPNowBuffer::getByte(uint16_t index)
{
  return _buffer[index];
}

uint16_t ESPNowBuffer::getWord(uint16_t index)
{
  uint16_t data = 0;
  data |= (_buffer[index + 0] & 0xFF) << 8;
  data |= (_buffer[index + 1] & 0xFF);
  return data;
}

uint32_t ESPNowBuffer::getDWord(uint16_t index)
{
  uint32_t data = 0;
  data |= (_buffer[index + 0] & 0xFF) << 24;
  data |= (_buffer[index + 1] & 0xFF) << 16;
  data |= (_buffer[index + 2] & 0xFF) << 8;
  data |= (_buffer[index + 3] & 0xFF);
  return data;
}

uint8_t *ESPNowBuffer::getData(uint8_t offset, uint8_t len)
{
  uint8_t data[len];
  for (int x = 0; x < len; x++)
  {
    data[x] = _buffer[offset + x];
  }
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
  _buffer[index + 3] = (data) & 0xFF;
}

void ESPNowBuffer::setData(uint8_t *data, uint8_t offset, uint8_t dataLen)
{
  for (int x = 0; x < dataLen; x++)
  {
    _buffer[offset + x] = data[x];
  }
}

void ESPNowBuffer::addByte(uint8_t data)
{
  this->setByte(_len, data);
  _len += 1;
}

void ESPNowBuffer::addWord(uint16_t data)
{
  this->setWord(_len, data);
  _len += 2;
}

void ESPNowBuffer::addDWord(uint32_t data)
{
  this->setDWord(_len, data);
  _len += 4;
}

void ESPNowBuffer::addData(uint8_t *data, uint8_t dataLen)
{
  this->setData(data, _len, dataLen);
  _len += dataLen;
}