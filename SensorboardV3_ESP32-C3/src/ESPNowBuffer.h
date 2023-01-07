#ifndef ESPNOW_RECV_BUFFER
#define ESPNOW_RECV_BUFFER

#include <Arduino.h>

#define ESPNOW_BUFFER_MAX 250
class ESPNowBuffer
{
public:
    // This does not include the checksum (It is added by the function)
    void clear();
    void copyData(const uint8_t *data, int len);

    uint8_t updateChecksum();
    uint8_t createChecksum();
    uint8_t getChecksum();
    bool isChecksumValid();

    uint8_t getByte(uint16_t index);
    uint16_t getWord(uint16_t index);
    uint32_t getDWord(uint16_t index);
    void getData(uint8_t offset, uint8_t len, uint8_t* data);

    void setByte(uint16_t index, uint8_t data);
    void setWord(uint16_t index, uint16_t data);
    void setDWord(uint16_t index, uint32_t data);
    void setData(uint8_t *data, uint8_t offset, uint8_t dataLen);

    void addByte(uint8_t data);
    void addWord(uint16_t data);
    void addDWord(uint32_t data);
    void addData(uint8_t *data, uint8_t dataLen);

    void setLen(uint16_t len)
    {
        _len = len;
    }

    uint16_t getLen()
    {
        return _len;
    }

    uint8_t *getBuffer()
    {
        return _buffer;
    }

    void setInUse(bool inUse)
    {
        _inUse = inUse;
    }

    bool isInUse()
    {
        return _inUse;
    }

    uint8_t operator[](int i) const
    {
        return _buffer[i];
    }

    uint8_t &operator[](int i)
    {
        return _buffer[i];
    }

private:
    uint8_t _buffer[ESPNOW_BUFFER_MAX];
    int _len;

    bool _inUse;

    bool _checksumAdded = false;
};

#endif