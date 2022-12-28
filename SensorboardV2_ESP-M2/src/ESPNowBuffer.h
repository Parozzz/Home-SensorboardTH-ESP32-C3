#ifndef ESPNOW_RECV_BUFFER
#define ESPNOW_RECV_BUFFER

#include <Arduino.h>

#define ESPNOW_BUFFER_MAX 250

#define ESPNOW_BUFFER_CHECKSUM_BYTES 1 // Max 4 bytes

class ESPNowBuffer
{
public:
    // ESPNowBuffer(const uint8_t *incomingData, int dataCount);
    // ESPNowBuffer(uint16_t dataCount);

    void setDataAmount(int len); //Set amount of byte to send. Checksum is set apart.
    void copyData(const uint8_t *data, int len);

    uint32_t calculateChecksum();
    void createAndSetChecksum();
    uint32_t getChecksum();
    bool isChecksumValid();

    uint8_t getByte(uint16_t index);
    uint16_t getWord(uint16_t index);
    uint32_t getDWord(uint16_t index);

    void setByte(uint16_t index, uint8_t data);
    void setWord(uint16_t index, uint16_t data);
    void setDWord(uint16_t index, uint32_t data);

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

    uint8_t& operator[](int i)
    {
        return _buffer[i];
    }

private:
    uint8_t _buffer[ESPNOW_BUFFER_MAX];
    int _len;

    bool _inUse;
};

#endif