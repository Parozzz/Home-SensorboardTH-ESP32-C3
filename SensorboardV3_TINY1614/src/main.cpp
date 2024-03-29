#include <Arduino.h>

#include <Wire.h>
#include <TinySHT31.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <EEPROM.h>
#include <ESPNowMessages.h>
#include <ESPNowBuffer.h>
#include <ErrorCodes.h>

#define TIME_MINUTE_SENDS 5             // Minutes between times (Not precise, maybe +-30s)
#define TIME_HOURS_LOW_BATTERY_SENDS 12 // Hours between sleeps when low battery is detected

#define TEMPERATURE_MIN_DIFF 0.2 // Temperature difference to send values to ESP
#define HUMIDITY_MIN_DIFF 1.35   // Temperature difference to send values to ESP

#define SHT31_READING_TRIALS 3
#define ESP32_WRITING_TRIALS 3

#define SDA_PIN PIN_PB1
#define SCL_PIN PIN_PB0
#define RX_PIN PIN_PB3
#define TX_PIN PIN_PB2

#define USB_IN_PIN PIN_PA1

#define ESPRDY_DONE_IMPULSES 5
#define ESP_READY_PIN PIN_PA2
#define ESP_PROG_PIN PIN_PA3
#define ESP_RESET_PIN PIN_PA7 // This is on a transistor. HIGH = Reset esp.

#define BATT_VOLT_CAPACITOR_DISCHARGE_TIME 50 // Milliseconds for the capacitor di to dischard to reach battery divider voltage
#define BATT_VOLT_ADC_GND_PIN PIN_PA5

#define SHT31_RST_PIN PIN_PA4

// Divide the voltage measured by the ADC0 Percentage result.
#define BATT_MAX_FOR_PERCENTAGE 4.2 // Max voltage for battery % calculation
#define BATT_VOLTAGE_VERY_LOW 3.5   // Min battery voltage

#define TERMINATOR_BYTE 0xFE
#define TERMINATOR_BYTE_COUNT 3

#define DEBUG
#define DEBUG_TIME

SHT31 sht31;
float batteryADC0FullScale = 1.0;

ESPNowBuffer recvESPNowBuffer;

void setup()
{
  // put your setup code here, to run once:
  pinMode(USB_IN_PIN, INPUT);
  pinMode(ESP_PROG_PIN, INPUT_PULLUP);
  pinMode(ESP_READY_PIN, INPUT_PULLUP);

  pinMode(BATT_VOLT_ADC_GND_PIN, OUTPUT);
  digitalWrite(BATT_VOLT_ADC_GND_PIN, HIGH); // This start slow to allow the capacitor to adjust after a startup.

  pinMode(ESP_RESET_PIN, OUTPUT);
  digitalWrite(ESP_RESET_PIN, LOW);

  pinMode(SHT31_RST_PIN, OUTPUT);
  digitalWrite(SHT31_RST_PIN, HIGH);

  //==================== RTC SETUP ====================
  RTC_CLKSEL = RTC_CLKSEL_INT1K_gc; // Select the 1024Hz oscillator as main source
  RTC_CTRLA = RTC_RTCEN_bm;         // Enable RTC

  RTC_PITCTRLA = RTC_PERIOD_CYC32768_gc | RTC_PITEN_bm; // 32k cicli = ~32 secondi

  while (RTC_STATUS > 0 | RTC_PITSTATUS > 0)
  {
  } // Wait for all register to be synchronized
  //==================== RTC SETUP ====================

  //==================== ADC SETUP ====================
  // ADC by default is set to 10 bit. NOT to run in standby mode
  VREF_CTRLA = VREF_ADC0REFSEL_0V55_gc; // Set reference for ADC0 to 1.5V
  // VREF_CTRLB = VREF_ADC0REFEN_bm; // Writing a '1' to this bit forces the voltage reference for the ADC0 to be running, even if it is not requested.

  ADC0_CTRLA = ADC_RESSEL_10BIT_gc; // 10 bit
  ADC0_CTRLB = ADC_SAMPNUM_ACC8_gc; // Number of accumulation

  // REFSEL = Set reference for ADC0 to INTERNAL_REFERENCE (1.1V). PRESCALER to 64 divisions
  // PRESC_DIV = The higher the prescaler, the more time the ADC takes and the more precise the values are. For 10 bit must be <1.5MHz. 10Mhz / 32 = 312KHz
  ADC0_CTRLC = ADC_REFSEL_INTREF_gc | ADC_PRESC_DIV32_gc;
  // INITDLY_DLY = Wait x number of cycles before starting the ADC for all the stuff to be initialized (Like VRef).
  // ASDV = randomize start reading of the ADC
  // SAMPDLY0 = Add a delay between accumulation to reduce periodic noise.
  ADC0_CTRLD = ADC_INITDLY_DLY16_gc | ADC_ASDV_bp | ADC_SAMPDLY0_bp | ADC_SAMPDLY1_bp;

  // TotalSampleTime = (2 + SAMPDLY + SAMPLEN) / CLK_ADC
  /*
    Using SAMPCTRL.SAMPLEN at the same time as CTRLD.SAMPDLY or CTRLD.ASDV will cause an unpredictable sampling length.
    Work Around = When setting SAMPCTRL.SAMPLEN greater than 0x0, the CTRLD.SAMPDLY and CTRLD.ASDV must be cleared.
  */
  // ADC0_SAMPCTRL = ADC_SAMPLEN0_bm | ADC_SAMPLEN1_bm; // Aggiunge delay tra un accumulazione e l'altra
  ADC0_MUXPOS = ADC_MUXPOS_AIN6_gc;           // Set ADC MUX Pin at AIN2 (PA6)
  PORTA_PIN6CTRL = PORT_ISC_INPUT_DISABLE_gc; // Disable input buffer for ADC pin
  //==================== ADC SETUP ====================

  sht31.setAddress(SHT31_DEFAULT_ADDRESS);

  Wire.pins(SDA_PIN, SCL_PIN);
  Wire.begin();

  Serial.setTimeout(750); // To shorten time when writing a command.
  Serial.pins(TX_PIN, RX_PIN);
  Serial.begin(115200);

#ifdef DEBUG
  Serial.println("Booted.");
#endif

  EEPROM.begin();
  EEPROM.get(0, batteryADC0FullScale);
  EEPROM.end();

  delay(2000);
}

void sleep(bool lowBatterySleep)
{
#ifdef DEBUG
  Serial.println("Sleep - Setting up.");
#endif

  RTC_PITINTCTRL = RTC_PI_bm; // Enable interrupt. Enable before serial otherwise it gets stuck there.
  while (RTC_PITSTATUS > 0)
  {
  } // Wait for all register to be synchronized

  // 1 cycles is around 32seconds
  // 10 = ~5 minute sleep
  // uint16_t sleepCycles = 5;
  uint16_t sleepCycles = (TIME_MINUTE_SENDS * 60) / 32;
  if (lowBatterySleep)
  {
    sleepCycles = (TIME_HOURS_LOW_BATTERY_SENDS * 3600) / 32; //~12h sleep
  }

#ifdef DEBUG
  uint16_t sleepTime = sleepCycles * 32;
  Serial.printf("Sleep - Start for %d:%d \n", sleepTime / 60, sleepTime % 60);
  Serial.println("================");
  delay(15);
#endif

  for (uint16_t x = 0; x < sleepCycles; x++)
  {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_mode();

    RTC_PITINTFLAGS = 1; // Clear interrutp flag.
    while (RTC_PITSTATUS > 0)
    {
    } // Wait for all register to be synchronized
  }

  RTC_PITINTCTRL = 0; // Disable interrupt

#ifdef DEBUG
  Serial.println("Sleep - Wakeup");
#endif
}

SHT31Reading readSensorData()
{
#ifdef DEBUG
  Serial.println("SHT31 - Start reading");
#endif

  SHT31Reading sensorData = sht31.readBoth();

#ifdef DEBUG
  if (!sensorData.valid)
  {
    Serial.println("SHT31 - Invalid Data.");
  }
  else
  {
    Serial.print("SHT31 - T= ");
    Serial.print(sensorData.temperature);
    Serial.print(", RH= ");
    Serial.println(sensorData.humidity);
  }
#endif

  return sensorData;
}

float lastTemp;
float lastHum;
bool validateSensorData(SHT31Reading sensorData, bool force)
{
  bool enoughDiff = fabs(sensorData.temperature - lastTemp) > TEMPERATURE_MIN_DIFF || fabs(sensorData.humidity - lastHum) > HUMIDITY_MIN_DIFF;
  if (enoughDiff || force)
  {
#ifdef DEBUG
    Serial.println("Validate - OK");
#endif
    lastTemp = sensorData.temperature;
    lastHum = sensorData.humidity;
    return true;
  }

#ifdef DEBUG
  Serial.println("Validate - Not enough difference");
#endif
  return false;
}

/*
===================================================
========= BATTERY READING AND CALIBRATION =========
===================================================
*/

#define ADC_10BIT
#ifdef ADC_10BIT
#define ADC_MAX_VALUE 0x3FF
#else
#define ADC_MAX_VALUE 0xFF
#endif

#define BATT_READING_MIN_DECIMALS_READING 3 // How many centisemal step for sending battery voltage. (4.03 - 4.00 - 3.97)
#define BATT_READING_IGNORE_FIRST 3
#define BATT_READINGS_BUFFER_SIZE 25 // Since the battery depletes REALLY slowly, this should only increase reading stability.
uint16_t oldBatteryReadings[BATT_READINGS_BUFFER_SIZE];
uint8_t batteryReadingsCount = 0;

uint16_t readADC0()
{
  digitalWrite(BATT_VOLT_ADC_GND_PIN, LOW);
  delay(BATT_VOLT_CAPACITOR_DISCHARGE_TIME); // The capicitor need time to discharge and reach the actual voltage to read the battery voltage. This is done to reduce power consumption.

  ADC0_CTRLA |= ADC_ENABLE_bm;                // Enable ADC
  ADC0_COMMAND = ADC_STCONV_bm;               // Start conversion
  while ((ADC0_COMMAND & ADC_STCONV_bm) != 0) // Wait for HW to clear the STCONV bit
  {
  }
  uint16_t adc0Value = (ADC0_RES / 8); // Voltage read (/64 perchè ho 64 accumulazioni impostate sui registri)
  ADC0_CTRLA &= ~ADC_ENABLE_bm;        // Disable ADC

  digitalWrite(BATT_VOLT_ADC_GND_PIN, HIGH);

  return adc0Value;
}

float calibrateBatteryFullScale(float measuredVoltage)
{
  uint16_t adc0Value = readADC0();

  float adc0Percentage = adc0Value / (float)ADC_MAX_VALUE;
  // adc0 : measured = MAX_VALUE : x
  // adc0Perecentage[%] : measuredVoltage[V] = 1.0[%] : x[V]

  return measuredVoltage / adc0Percentage;
}

// With 64 accumulation + 5 readings this takes ~120ms.
float readBatteryVoltage()
{
#ifdef DEBUG
  Serial.println("Battery - Start reading ADC");
#endif

  uint16_t adc0Value = readADC0();

  float adc0Percentage = adc0Value / (float)ADC_MAX_VALUE;
  float adc0Voltage = adc0Percentage * batteryADC0FullScale;

  uint16_t voltageInt = (uint16_t)floor(adc0Voltage * 100.0);

  // This discrimate if the reading is in the high part of the remainder or not.
  // If is not, i select the higher part of the voltage. If in the middle, it will select the lower value just to be safe.
  uint16_t remainder = (voltageInt % BATT_READING_MIN_DECIMALS_READING);
  voltageInt = voltageInt - remainder;
  if (remainder > (BATT_READING_MIN_DECIMALS_READING / 2))
  {
    voltageInt += BATT_READING_MIN_DECIMALS_READING;
  }

  for (int x = BATT_READINGS_BUFFER_SIZE - 2; x >= 0; x--)
  {
    oldBatteryReadings[x + 1] = oldBatteryReadings[x];
  }
  oldBatteryReadings[0] = voltageInt;
  batteryReadingsCount = min(batteryReadingsCount + 1, BATT_READINGS_BUFFER_SIZE);

  // This is used to find the more frequent value inside the old readings.
  // Using this wait it should eliminate reading spike eliminating less frequent values and keep only the more frequent.
  uint16_t valueMatrix[10][2]; //[][0]:Voltage [][1]:Counts
  memset(valueMatrix, 0, sizeof(valueMatrix));
  for (int x = 0; x < batteryReadingsCount; x++)
  {
    uint16_t loopVoltage = oldBatteryReadings[x];

    for (int y = 0; y < 10; y++)
    {
      // If i have found the value or if i find a zero (It means i reached the end!) it will set the voltage inside the loop and increase the count.
      if (valueMatrix[y][0] == loopVoltage || valueMatrix[y][0] == 0)
      {
        valueMatrix[y][0] = loopVoltage;
        valueMatrix[y][1]++;
        break;
      }
    }
  }

  uint16_t finalVoltageInt = 0;
  uint16_t maxCount = 0;
  for (int x = 0; x < 10; x++)
  {
    uint16_t loopVoltage = valueMatrix[x][0];
    uint16_t loopCount = valueMatrix[x][1];
    if (loopVoltage == 0 || loopCount == 0)
    {
      break;
    }

    if (maxCount == 0 || loopCount > maxCount)
    {
      maxCount = loopCount;
      finalVoltageInt = loopVoltage;
    }
  }

  float voltage = finalVoltageInt / 100.0;

#ifdef DEBUG
  Serial.print("Battery - ADC0 FullScale= ");
  Serial.print(batteryADC0FullScale, 3);
  Serial.print(", ADC0[%]= ");
  Serial.print(adc0Percentage, 4);
  Serial.print(", Vadc0[V]= ");
  Serial.print(adc0Voltage, 3);
  Serial.printf(", Vconv= %d, R= %d", voltageInt, remainder);
  Serial.println();

  for (int x = 0; x < 10; x++)
  {
    uint16_t loopVoltage = valueMatrix[x][0];
    uint16_t loopCount = valueMatrix[x][1];
    if (loopVoltage == 0 || loopCount == 0)
    {
      break;
    }

    Serial.printf("Battery - Count= %d, Vint= %d\n", loopCount, loopVoltage);
  }

  Serial.print("Battery - V= ");
  Serial.print(voltage, 2);
  Serial.print("[V]");
  Serial.println();
#endif

  return voltage;
}

bool waitESPRDYImpulses(uint32_t timeoutTime)
{
  uint32_t timeoutTimestamp = millis();

  uint8_t readyImpulseCount = 0;
  uint8_t oldImpulse = true; // ESP START WITH PIN HIGH!
  while (readyImpulseCount < ESPRDY_DONE_IMPULSES)
  {
    if (millis() - timeoutTimestamp > timeoutTime)
    {
#ifdef DEBUG
      Serial.printf("\nESPSend - Pulse timeout. Impulse recv: %d \n", readyImpulseCount);
#endif
      return false;
    }

    uint8_t impulse = digitalRead(ESP_READY_PIN);
    if (!impulse && oldImpulse) // Only care about falling edge
    {
      readyImpulseCount++;
    }

    oldImpulse = impulse;
  }

  return true;
}

bool ESPSend()
{
#ifdef DEBUG
  Serial.println("ESPSend - Wake ESP");
#endif

  digitalWrite(ESP_RESET_PIN, HIGH);
  delay(1);
  digitalWrite(ESP_RESET_PIN, LOW);

  delay(50); // The ESP32-C3 seems to have ~200ms of bootup time where pins are not reliable. So i wait a bit before waiting.

  uint32_t wakeupTimestamp = millis();
  while (digitalRead(ESP_READY_PIN)) // Wait for the ESP to put the pin LOW
  {
    if (millis() - wakeupTimestamp > 500)
    {
#ifdef DEBUG
      Serial.println("ESPSend - Wake FAILED.");
#endif
      return false;
    }
  }

  delay(5); // Wait some time before sending data.
  for (int x = 0; x < recvESPNowBuffer.getLen(); x++)
  {
    Serial.write(recvESPNowBuffer[x]);
  }

  for (int x = 0; x < TERMINATOR_BYTE_COUNT; x++)
  {
    Serial.write(TERMINATOR_BYTE);
  }
  Serial.flush();

  if (!waitESPRDYImpulses(150)) // Wait for the ESP to send me impulses to indicate serial reading OK
  {
#ifdef DEBUG
    Serial.println("ESPSend - Serial failed");
#endif
    return false;
  }

  if (!waitESPRDYImpulses(1000))
  {
#ifdef DEBUG
    Serial.println("ESPSend - ESPNow fail");
#endif
    return false;
  }

#ifdef DEBUG
  Serial.println("\nESPSend - OK");
#endif

  return true;
}

bool firstBoot = false;
bool batteryCalibration = false;
bool stopReadContinously = false;
uint16_t errorCode1, errorCode2, errorCode3 = 0;

uint32_t usbPluggedTimestamp = 0;

uint32_t espProgTimestamp = 0;
bool espProgrammingMode = false;

bool lowBatterySent = false;

bool isCharging()
{
  return digitalRead(USB_IN_PIN);
}

SensorboardV2Message message;
bool prepareReadingESPNowBuffer(float temperature, float humidity, float batteryVoltage)
{
  message.temperature = temperature;
  message.humidity = lround(ceil(humidity));
  message.batteryVoltage = batteryVoltage;
  message.charging = isCharging();

#ifdef DEBUG
  message.printDebug(&Serial);
#endif

  if (message.sanityCheck())
  {
#ifdef DEBUG
    Serial.println("ESPNowBuffer - Reading Sanity Check OK");
#endif
    recvESPNowBuffer.clear();
    message.writeData(&recvESPNowBuffer);
    recvESPNowBuffer.updateChecksum();
    return true;
  }

#ifdef DEBUG
  Serial.println("ESPNowBuffer - Reading Sanity Check FAIL");
#endif
  return false;
}

ErrorMessage errorMessage = ErrorMessage(BoardType::SENSOR_BOARD);
bool prepareErrorESPNowBuffer(uint16_t errorCode, float batteryVoltage)
{
  errorMessage.code1 = errorCode;
  errorMessage.batteryVoltage = batteryVoltage;

#ifdef DEBUG
  errorMessage.printDebug(&Serial);
#endif

  if (errorMessage.sanityCheck())
  {
#ifdef DEBUG
    Serial.println("ESPNowBuffer - Error Sanity Check OK");
#endif
    recvESPNowBuffer.clear();
    errorMessage.writeData(&recvESPNowBuffer);
    recvESPNowBuffer.updateChecksum();
    return true;
  }

#ifdef DEBUG
  Serial.println("ESPNowBuffer - Error Sanity Check FAIL");
#endif
  return false;
}

void loop()
{
#pragma region Calibrations and Setup
  if (isCharging())
  {
    if (!firstBoot)
    {
      firstBoot = true;

      bool calibrated = batteryADC0FullScale != NAN && batteryADC0FullScale > 0; // NAN = EEPROM HAS NOT READ IT PROPERLY.
      if (!calibrated)
      {
        Serial.println("Calibration required.");
      }

      Serial.println("Waiting for command.");
      Serial.println("1 or BTT = Calibrate battery");
      Serial.println("2 or SAVE = Save & Quit calibration");
      Serial.println("3 or START = Restart continous reading.");
      Serial.println("4 or STOP = Stop continous reading.");
      Serial.println("======");
      delay(2);
    }

    if (Serial.available() > 0)
    {
      String read = Serial.readString();

      Serial.print("Recv= ");
      Serial.println(read);

      if (read == "1" || read == "BTT")
      {
        batteryCalibration = true;

        Serial.println("Write actual battery voltage for calibration in format x.xxx");
        Serial.println();
      }
      else if (read == "2" || read == "SAVE")
      {
        batteryCalibration = false;

        Serial.println("Battery factor saved.");
        Serial.println();
      }
      else if (read == "3" || read == "START")
      {
        stopReadContinously = false;
        Serial.println("Readings restarted.");
      }
      else if (read == "4" || read == "STOP")
      {
        stopReadContinously = true;
        Serial.println("Readings stopped.");
      }
      else if (batteryCalibration)
      {
        float voltageMeasured = read.toFloat();
        if (voltageMeasured > 0)
        {
          batteryADC0FullScale = calibrateBatteryFullScale(voltageMeasured);

          EEPROM.begin();
          EEPROM.put(0, batteryADC0FullScale);
          EEPROM.end();

          Serial.print("New FullScale= ");
          Serial.println(batteryADC0FullScale, 4);
          Serial.println("SAVE to save and exit calibration input");
        }
      }
    }

    // Do this after the serial stuff so it can be done
    // If the prog button is pressed, any operation is halted to avoid disturbing the esp.
    if (!digitalRead(ESP_PROG_PIN))
    {
      Serial.println("ESPProg - Button pressed.");
      while (!digitalRead(ESP_PROG_PIN)) // Wait for the button to be released.
      {
        _NOP();
      }

      Serial.println("ESPProg - Programming mode active. Waiting for ESP Wakeup.");
      delay(2000);

      espProgrammingMode = true;
    }

    // Once the programming is active, i wait for the esp to put the ready pin low (Meaning is has been woke up after programming).
    if (espProgrammingMode && !digitalRead(ESP_READY_PIN))
    {
      Serial.println("ESPProg - Programming mode disabled.");
      espProgrammingMode = false;
    }

    if (espProgrammingMode || stopReadContinously || batteryCalibration || (millis() - usbPluggedTimestamp) < 5000)
    {
      return;
    }
  }
  else
  {
    // If is not charging, disable all this calibration stuff to avoid any issue.
    espProgrammingMode = false;
    stopReadContinously = false;
    batteryCalibration = false;
  }
#pragma endregion

#ifdef DEBUG_TIME
  uint32_t totalTimestamp = millis();
#endif

  float batteryVoltage = readBatteryVoltage();
  if (!isCharging() && batteryVoltage < BATT_VOLTAGE_VERY_LOW)
  {
    if (!lowBatterySent && prepareErrorESPNowBuffer(ERROR_BATTERY_LOW, batteryVoltage))
    {
      lowBatterySent = ESPSend();
    }

    sleep(true);
    return;
  }

  lowBatterySent = false;
  // If i am updating the battery factor, i do not want all the stuff for the ESP or the sensor.
  SHT31Reading sensorData = readSensorData();
  if (!sensorData.valid)
  {
    prepareErrorESPNowBuffer(ERROR_READING_TH_SENSOR, batteryVoltage);
    ESPSend();

    sleep(true);
    return;
  }
  else if (validateSensorData(sensorData, isCharging()) && prepareReadingESPNowBuffer(sensorData.temperature, sensorData.humidity, batteryVoltage))
  {
#ifdef DEBUG_TIME
    uint32_t espTimeTaken = millis();
#endif
    for (int x = 0; x < ESP32_WRITING_TRIALS; x++)
    {
      // Inside the loop in case is pressed after one trial is done to avoid waiting timeouts.
      if (isCharging() && !digitalRead(ESP_PROG_PIN))
      {
        Serial.println("Stopping ESPSend for programming.");
        return;
      }
#ifdef DEBUG
      Serial.printf("ESPSend - Start try #%d \n", x);
#endif
      if (ESPSend())
      {
        break;
      }
    }
#ifdef DEBUG_TIME
#ifndef DEBUG
    Serial.println(); // Used to separe the time from the data sent.
#endif
    Serial.printf("ESPSend - Time: %d ms \n", (millis() - espTimeTaken));
#endif
  }

#ifdef DEBUG_TIME
  Serial.printf("Total operation time %d ms \n", (millis() - totalTimestamp));
  Serial.println("======== END LOOP ========");
  delay(10);
#endif

  if (isCharging())
  {
    usbPluggedTimestamp = millis();
    return;
  }

  sleep(false);
}

ISR(RTC_PIT_vect)
{
}