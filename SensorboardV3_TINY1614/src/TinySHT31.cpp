/*!
 *  @file Adafruit_SHT31.cpp
 *
 *  @mainpage Adafruit SHT31 Digital Humidity & Temp Sensor
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the SHT31 Digital Humidity & Temp Sensor
 *
 *  Designed specifically to work with the SHT31 Digital sensor from Adafruit
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/2857
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  @section license License
 *
 *  BSD license, all text above must be included in any redistribution
 */

#include <TinySHT31.h>

void SHT31::setAddress(uint8_t i2caddr)
{
    _i2cAddress = i2caddr;
}

bool SHT31::softReset()
{
    writeCommand(SHT31_SOFTRESET);
    delay(10);

    return readStatus() != 0xFFFF;
}

void SHT31::setRepeatability(SHT31_MEASURE_REPEATABILITY repeatability)
{
    _repeatability = repeatability;
}

uint16_t SHT31::readStatus(void)
{
    writeCommand(SHT31_READSTATUS);

    Wire.requestFrom(_i2cAddress, 3);
    uint16_t stat = Wire.read();
    stat <<= 8;
    stat |= Wire.read();
    return stat;
}

bool SHT31::heaterStatus()
{
    uint16_t status = readStatus();
    return (status >> 13 & 0x01) == 1;
}

void SHT31::heater(bool h)
{
    if (h)
        writeCommand(SHT31_HEATEREN);
    else
        writeCommand(SHT31_HEATERDIS);
}

SHT31Reading SHT31::readBoth(void)
{
    return readTempHum();
}

float SHT31::readTemperature(void)
{
    return readTempHum().humidity;
}

float SHT31::readHumidity(void)
{
    return readTempHum().humidity;
}

/**
 * Performs a CRC8 calculation on the supplied values.
 *
 * @param data  Pointer to the data to use when calculating the CRC8.
 * @param len   The number of bytes in 'data'.
 *
 * @return The computed CRC8 value.
 */
static uint8_t crc8(const uint8_t *data, int len)
{
    /*
   *
   * CRC-8 formula from page 14 of SHT spec pdf
   *
   * Test data 0xBE, 0xEF should yield 0x92
   *
   * Initialization data 0xFF
   * Polynomial 0x31 (x8 + x5 +x4 +1)
   * Final XOR 0x00
   */

    const uint8_t POLYNOMIAL(0x31);
    uint8_t crc(0xFF);

    for (int j = len; j; --j)
    {
        crc ^= *data++;

        for (int i = 8; i; --i)
        {
            crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
        }
    }
    return crc;
}

SHT31Reading SHT31::readTempHum(void)
{
    SHT31Reading readValues = SHT31Reading();

    uint8_t readbuffer[6];

    writeCommand(_repeatability);
    delay(20);

    Wire.requestFrom(_i2cAddress, (int) sizeof(readbuffer));
    if (Wire.available() != sizeof(readbuffer))
    {
        readValues.operationResult = 1;
        return readValues;
    }

    for (size_t i = 0; i < sizeof(readbuffer); i++)
    {
        readbuffer[i] = Wire.read();
    }

    if (readbuffer[2] != crc8(readbuffer, 2) || readbuffer[5] != crc8(readbuffer + 3, 2))
    {
        readValues.operationResult = 2;
        return readValues;
    }

    int32_t stemp = (int32_t)(((uint32_t)readbuffer[0] << 8) | readbuffer[1]);
    // simplified (65536 instead of 65535) integer version of:
    // temp = (stemp * 175.0f) / 65535.0f - 45.0f;
    stemp = ((4375 * stemp) >> 14) - 4500;
    readValues.temperature = (float)stemp / 100.0f;

    uint32_t shum = ((uint32_t)readbuffer[3] << 8) | readbuffer[4];
    // simplified (65536 instead of 65535) integer version of:
    // humidity = (shum * 100.0f) / 65535.0f;
    shum = (625 * shum) >> 12;
    readValues.humidity = (float)shum / 100.0f;

    readValues.valid = true;
    
    return readValues;
}

void SHT31::writeCommand(uint16_t cmd)
{
    Wire.beginTransmission(_i2cAddress);
    Wire.write(cmd >> 8);
    Wire.write(cmd & 0xFF);
    Wire.endTransmission();
}
