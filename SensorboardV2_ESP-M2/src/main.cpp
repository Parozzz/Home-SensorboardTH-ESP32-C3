#include <Arduino.h>

#include <Esp.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <ESPNowBuffer.h>

#define SENSORBOARD_ID 1
#define REEDBOARD_ID 2
#define SENSORBOARDV2_ID 3

#define BOARD_ID_DIPSWITCH_BIT0 14
#define BOARD_ID_DIPSWITCH_BIT1 12
#define BOARD_ID_DIPSWITCH_BIT2 13

#define ESP_OK 0
#define WIFI_CHANNEL 7

#define ACK 0x69
#define NACK 0xBB
#define ACK_COUNT 8

//#define DEBUG

//{0x24, 0x6F, 0x28, 0x9E, 0x68, 0x58}; //OF MINI32 WITH STA
//{0x24, 0x6F, 0x28, 0x96, 0xDD 0x24}; //OF Heltec BOARD

#define MAC_ESP32_S2_CUCUMBER          \
  {                                    \
    0x7C, 0xDF, 0xA1, 0x00, 0xBA, 0x62 \
  }

ESPNowBuffer sendBuffer;
ESPNowBuffer espnow_buffer;

uint8_t recvMAC[6];
uint8_t slaveMACAddress[] = MAC_ESP32_S2_CUCUMBER;

void sleep()
{
  delay(20);

  ESP.deepSleepInstant(0, WAKE_NO_RFCAL);
  delay(5000);
}

volatile int16_t espNowDataSentResult; // 0 = OK, >0 = NOK
static void OnEspNowDataSent(unsigned char *mac_addr, uint8_t status)
{
  espNowDataSentResult = status;
}

void OnESPNowDataRecv(uint8_t *mac_addr, uint8_t *data, uint8_t len)
{
  if (len > 3)
  {
    memcpy(recvMAC, mac_addr, 6);

    espnow_buffer.copyData(data, len);
    espnow_buffer.setInUse(true);
  }
}

bool initWifi()
{
#ifdef DEBUG
  Serial.print("INIT WiFi");
#endif

  WiFi.forceSleepWake();
  WiFi.persistent(false);
  if (WiFi.mode(WIFI_STA))
  {
    WiFi.setSleepMode(WiFiSleepType_t::WIFI_NONE_SLEEP);
    WiFi.setOutputPower(20.5);
    WiFi.setAutoConnect(false);
    WiFi.setAutoReconnect(false);

    WiFi.begin("dummy_ssid", "dummy_ssid", WIFI_CHANNEL, NULL, false);
    WiFi.disconnect();
    return true;
  }

  return false;
}

bool initESPNow()
{
#ifdef DEBUG
  Serial.print("INIT ESPNow");
#endif

  if (esp_now_init() == ESP_OK)
  {
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_add_peer(slaveMACAddress, ESP_NOW_ROLE_CONTROLLER, WIFI_CHANNEL, NULL, 0);

    esp_now_register_send_cb(OnEspNowDataSent);
    esp_now_register_recv_cb(OnESPNowDataRecv);

    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);

    return true;
  }

  return false;
}

void stopESPNow()
{
  esp_now_deinit();
}

bool sendESPNow()
{
#ifdef DEBUG
  Serial.println("ESPNow - Send Data");
#endif

  espNowDataSentResult = -1;

  uint8_t *buffer = sendBuffer.getBuffer();
  uint16_t len = sendBuffer.getLen();
  if (esp_now_send(slaveMACAddress, buffer, len) == ESP_OK)
  {
    uint32_t timestamp = millis();
    while (espNowDataSentResult == -1)
    {
      yield(); // This is required otherwise the CPU is completely STUCK
      if (millis() - timestamp > 500)
      {
        break;
      }
    }
  }

#ifdef DEBUG
  Serial.print("ESPNow - Result = ");
  Serial.println(espNowDataSentResult == ESP_OK ? "OK" : "FAIL");
#endif

  return espNowDataSentResult == ESP_OK;
}

void sendACK()
{
#ifdef DEBUG
  Serial.println("Serial - Sending ACK");
#endif

  for (int x = 0; x < ACK_COUNT; x++)
  {
    Serial.write(ACK);
  }
}

void sendNACK()
{
#ifdef DEBUG
  Serial.println("Serial - Sending NACK");
#endif

  for (int x = 0; x < ACK_COUNT; x++)
  {
    Serial.write(NACK);
  }
}

struct
{
  uint16_t temperature;
  uint8_t humidity;
  uint16_t batteryVoltage;
} serialData;

bool waitSerialData()
{
#ifdef DEBUG
  Serial.println("SerialData - Waiting data");
#endif

  uint32_t timestamp = millis();
  while (Serial.available() < 8)
  {
    yield();

    if (millis() - timestamp > 200)
    {

#ifdef DEBUG
      Serial.println("SerialData - Timeout recv");
#endif

      return false;
    }
  }

  uint16_t temperature = (Serial.read() << 8) | Serial.read();
  uint8_t humidity = Serial.read();
  uint16_t batteryVoltage = (Serial.read() << 8) | Serial.read();
  uint16_t checksumRecv = (Serial.read() << 8) | Serial.read();

  uint16_t checksumCalc = temperature + humidity + batteryVoltage;
  if (checksumCalc != checksumRecv)
  {

#ifdef DEBUG
    Serial.println("SerialData - Invalid checksum");
    Serial.println(String() + "T[°C]=" + temperature + ", RH[%]=" + rh + ", B[V]=" + batteryPercentage + ", Checksum=" + checksumCalc);
#endif

    return false;
  }

  serialData.temperature = temperature;
  serialData.humidity = humidity;
  serialData.batteryVoltage = batteryVoltage;

#ifdef DEBUG

  float fTemperature = temperature / 100.0;
  float fBatteryVoltage = batteryVoltage / 100.0;
  Serial.println(String() + "BoardID= " + boardID + ", T[°C]= " + fTemperature + ", RH[%]= " + humidity + ", B[V]= " + fBatteryVoltage);
#endif

  return true;
}

bool fillESPNowBuffer(uint8_t boardID)
{
#ifdef DEBUG
  Serial.println("Filling ESPNow Send Buffer.");
#endif

  sendBuffer.setDataAmount(7);

  sendBuffer.setByte(0, SENSORBOARDV2_ID);
  sendBuffer.setByte(1, boardID);
  sendBuffer.setWord(2, serialData.temperature);
  sendBuffer.setByte(5, serialData.humidity);
  sendBuffer.setWord(6, serialData.batteryVoltage);

  sendBuffer.createAndSetChecksum();

  return true;
}

uint8_t readBoardID()
{
  pinMode(BOARD_ID_DIPSWITCH_BIT0, INPUT_PULLUP);
  pinMode(BOARD_ID_DIPSWITCH_BIT1, INPUT_PULLUP);
  pinMode(BOARD_ID_DIPSWITCH_BIT2, INPUT_PULLUP);

  uint8_t bit0 = !digitalRead(BOARD_ID_DIPSWITCH_BIT0);
  uint8_t bit1 = !digitalRead(BOARD_ID_DIPSWITCH_BIT1);
  uint8_t bit2 = !digitalRead(BOARD_ID_DIPSWITCH_BIT2);
  uint8_t boardID = bit0 | (bit1 << 1) | (bit2 << 2);

#ifdef DEBUG
  Serial.print("Board ID= ");
  Serial.print(boardID, HEX);
  Serial.print(", Bit0=");
  Serial.print(bit0, HEX);
  Serial.print(", Bit1=");
  Serial.print(bit1, HEX);
  Serial.print(", Bit2=");
  Serial.println(bit2, HEX);
#endif

  return boardID;
}

void setup()
{
  if (system_get_os_print())
  {
    system_set_os_print(0);
  }

  Serial.begin(38400);
  Serial.println();

  uint8_t boardID = readBoardID();
  if (boardID == 0 || !initWifi() || !initESPNow())
  {
    sendNACK();
    sleep();
  }

  sendACK(); // Write wake up ok.

  if (!waitSerialData() || !fillESPNowBuffer(boardID) || !sendESPNow())
  {
    sendNACK();
    sleep();
  }

  sendACK();
}

void loop()
{

#ifdef DEBUG
  Serial.print("======================");
#endif

  sleep();
}
