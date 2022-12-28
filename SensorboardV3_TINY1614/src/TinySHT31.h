/*!
 *  @file Adafruit_SHT31.h
 *
 *  This is a library for the SHT31 Digital Humidity & Temp Sensor
 *
 *  Designed specifically to work with the  Digital Humidity & Temp Sensor
 *  -----> https://www.adafruit.com/product/2857
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  BSD license, all text above must be included in any redistribution
 */

#ifndef ADAFRUIT_SHT31_H
#define ADAFRUIT_SHT31_H

#include "Arduino.h"
#include <Wire.h>

#define SHT31_DEFAULT_ADDRESS 0x44 /**< SHT31 Default Address if ADDR pin connected to GND */
#define SHT31_VCC_ADDRESS 0x45 /**< SHT31 address if ADDR pin connected to VCC */

#define SHT31_MEAS_HIGHREP_STRETCH 0x2C06 /**< Measurement High Repeatability with Clock Stretch Enabled */
#define SHT31_MEAS_MEDREP_STRETCH 0x2C0D  /**< Measurement Medium Repeatability with Clock Stretch Enabled */
#define SHT31_MEAS_LOWREP_STRETCH 0x2C10  /**< Measurement Low Repeatability with Clock Stretch Enabled*/

#define SHT31_MEAS_HIGHREP 0x2400 /**< Measurement High Repeatability with Clock Stretch Disabled */
#define SHT31_MEAS_MEDREP 0x240B  /**< Measurement Medium Repeatability with Clock Stretch Disabled */
#define SHT31_MEAS_LOWREP 0x2416  /**< Measurement Low Repeatability with Clock Stretch Disabled */
#define SHT31_READSTATUS 0xF32D   /**< Read Out of Status Register */
#define SHT31_CLEARSTATUS 0x3041  /**< Clear Status */
#define SHT31_SOFTRESET 0x30A2    /**< Soft Reset */
#define SHT31_HEATEREN 0x306D     /**< Heater Enable */
#define SHT31_HEATERDIS 0x3066    /**< Heater Disable */

struct SHT31Reading
{
  float humidity;
  float temperature;
  uint8_t operationResult;
  bool valid;

  SHT31Reading()
  {
    humidity = -1;
    temperature = -1;
    operationResult = 0;
    valid = false;
  }
};

enum SHT31_MEASURE_REPEATABILITY
{
  LOWREP = SHT31_MEAS_LOWREP,
  MEDIUMREP = SHT31_MEAS_MEDREP,
  HIGHREP = SHT31_MEAS_HIGHREP
};

/**
 * Driver for the Adafruit SHT31-D Temperature and Humidity breakout board.
 */
class SHT31
{
public:
  void setAddress(uint8_t i2cAddress);

  bool softReset();

  /**
   * Set the repeatability of the measurements. Lower repeatability means lower power consumptions.
   * 
   * @param repeatability The new repeatability
   */
  void setRepeatability(SHT31_MEASURE_REPEATABILITY repeatability);

  /**
   * Gets a single temperature reading from the sensor.
   *
   * @return A float value indicating the temperature.
   */
  float readTemperature(void);

  /**
   * Gets a single relative humidity reading from the sensor.
   *
   * @return A float value representing relative humidity.
   */
  float readHumidity(void);

  /**
   * Get a reading of both values
   *
   * @return A float value representing relative humidity.
   */
  SHT31Reading readBoth(void);

  /**
   * Gets the current status register contents.
   *
   * @return The 16-bit status register.
   */
  uint16_t readStatus(void);

  /**
   * Read the status of the heater
   * 
   * @return The status of the heater. Return True if active.
   */
  bool heaterStatus();

  /**
   * Enables or disabled the heating element.
   *
   * @param h True to enable the heater, False to disable it.
   */
  void heater(bool h);

private:
  uint8_t _i2cAddress = SHT31_DEFAULT_ADDRESS;
  SHT31_MEASURE_REPEATABILITY _repeatability = SHT31_MEASURE_REPEATABILITY::HIGHREP;

  /**
   * Internal function to perform a temp + humidity read.
   *
   * @return True if successful, otherwise false.
   */
  SHT31Reading readTempHum(void);

  /**
   * Internal function to perform and I2C write.
   *
   * @param cmd   The 16-bit command ID to send.
   */
  void writeCommand(uint16_t cmd);

  /**
   * Internal function to read data over the I2C bus.
   *
   * @return True if successful, otherwise False.
   */
  bool readData(void);
};

#endif
