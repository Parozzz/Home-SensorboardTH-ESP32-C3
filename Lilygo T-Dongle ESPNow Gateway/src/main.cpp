/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

/*
#define LILYGO_TDONGLE_MAC {0x58, 0xCF, 0x79, 0x32, 0xE1, 0xC2}
*/

#include <WiFi.h>
#include <esp_now.h>
#include <USB.h>
#include <USBCDC.h>
#include <ESPNowBuffer.h>
#include <ESPNowMessages.h>
#include <ESPNowJsonMessages.h>
#include <TDongleScreen.h>

//#define SERIAL_DEBUG

#define WIFI_CHANNEL 7

#define LED 39

#define HEADER_LEN 2
#define FOOTER_LEN 1

// ESP_OK = 0
// ESP_FAIL = -1
#define ESP_WAIT -2

USBCDC usbSerial;
TDongleScreen screen;

struct RecvData
{
  uint8_t macAddress[6];
  ESPNowBuffer espnowBuffer;
  bool inUse = false;
};

#define RECV_DATA_AMOUNT 5
RecvData recvData[RECV_DATA_AMOUNT] = {
    RecvData(),
    RecvData(),
    RecvData(),
    RecvData(),
    RecvData()
};
SensorboardV2Message sensorboardMessage;

int8_t sendESPNowStatus;
void OnESPNowSend(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  sendESPNowStatus = status == esp_now_send_status_t::ESP_NOW_SEND_SUCCESS ? ESP_OK : ESP_FAIL;
}

void OnESPNowRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  if (len > 3)
  {
    for (int x = 0; x < RECV_DATA_AMOUNT; x++)
    {
      RecvData *data = &recvData[x];
      if (!data->inUse)
      {
        memcpy(data->macAddress, mac, 6);

        data->espnowBuffer.copyData(incomingData, len);
        data->inUse = true;

        break;
      }
    }
  }
}

bool initWiFi()
{
  WiFi.softAP("dummy_ssid", "dummy_ssid", WIFI_CHANNEL, 0, 1, false);

  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);

  if (WiFi.enableSTA(true))
  {
    WiFi.setSleep(wifi_ps_type_t::WIFI_PS_NONE);
    WiFi.disconnect();
    return true;
  }

  return false;
}

bool initESPNow()
{

  if (esp_now_init() == ESP_OK)
  {
    esp_now_register_send_cb(OnESPNowSend);
    esp_now_register_recv_cb(OnESPNowRecv);
    return true;
  }

  return false;
}

void yieldLoop()
{
  while (true)
  {
    yield();
  }
}

void setup()
{
  screen.init();

  USB.usbClass(0);
  USB.usbSubClass(0);
  USB.usbProtocol(0);
  USB.manufacturerName("PAROZZZ");
  USB.productName("PAROZZZ_USB");

  usbSerial.begin(115200);
  USB.begin();

  delay(500);

  bool wifiInitOK = initWiFi();
  bool espNowInitOK = initESPNow();

#ifdef SERIAL_DEBUG
  usbSerial.println(screen.printf("INIT - WiFi= %s, ESPNow= %s",
                                  wifiInitOK ? "OK" : "FAIL",
                                  espNowInitOK ? "OK" : "FAIL"));
#endif

  if (!wifiInitOK || !espNowInitOK)
  {
    yieldLoop();
  }

  pinMode(LED, OUTPUT);
}

uint32_t ledFlashTimestamp = 0;
bool ledState = false;

uint32_t sendFeedbackTimestamp;
bool sendFirstValues = false;

uint8_t displayCounter = 0;

void parseReceivedData(RecvData *data)
{
#ifdef SERIAL_DEBUG
  usbSerial.printf("ESPNow - Type=%d, ID=%02x, Len=%d, MAC=", data->espnowBuffer[0], data->espnowBuffer[1], data->espnowBuffer.getLen());
  for (int x = 0; x < 6; x++)
  {
    usbSerial.print(data->macAddress[x], HEX);
    usbSerial.print(":");
  }

  usbSerial.print(", Recv Hex=");
  for (int x = 0; x < data->espnowBuffer.getLen(); x++)
  {
    usbSerial.printf("%02x", data->espnowBuffer[x]);
  }
  usbSerial.println();
#endif

  if (!data->espnowBuffer.isChecksumValid())
  {
    screen.printf("%d-Checksum err. R=%d C=%d",
                  displayCounter++,
                  data->espnowBuffer.getChecksum(),
                  data->espnowBuffer.createChecksum());

    return;
  }

  switch (data->espnowBuffer[0])
  {
  case BoardMessageType::SENSORBOARD_V2_MESSAGE:
    sensorboardMessage.parseData(&data->espnowBuffer);
    screen.printf("%d-SBV2 ID%d B%.2f C%d T%.2f H%d",
                  displayCounter++,
                  sensorboardMessage.id,
                  sensorboardMessage.batteryVoltage,
                  sensorboardMessage.charging,
                  sensorboardMessage.temperature,
                  sensorboardMessage.humidity);
    printSensorboardJSON(&usbSerial, sensorboardMessage);
    break;
  default:
    screen.printf("%d-Type err. R=%d",
                  displayCounter++,
                  data->espnowBuffer[0]);
    return;
  }

  // RE-SEND THE RECEIVED MESSAGE BACK
  // This is done for specific boards that want to be sure that the message it has been received correctly.
  // Not all boards uses it (Since it increase power consumptions) but it is kept for compatibility.
  esp_now_peer_info peerInfo;
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.ifidx = wifi_interface_t::WIFI_IF_STA;
  peerInfo.encrypt = false;
  memcpy(peerInfo.peer_addr, data->macAddress, 6);
  esp_now_add_peer(&peerInfo);

  sendESPNowStatus = ESP_WAIT;
  esp_now_send(data->macAddress, data->espnowBuffer.getBuffer(), data->espnowBuffer.getLen());

  sendFeedbackTimestamp = millis();
  while (sendESPNowStatus == ESP_WAIT)
  {
    yield();
    if ((millis() - sendFeedbackTimestamp) > 300)
    {
      break;
    }
  }

  esp_now_del_peer(peerInfo.peer_addr);

#ifdef SERIAL_DEBUG
  usbSerial.printf("Send Status= %s\n", sendESPNowStatus == ESP_OK ? "OK" : sendESPNowStatus == ESP_WAIT ? "TIMEOUT"
                                                                                                         : "FAIL");
#endif
}

void loop()
{
#ifdef SERIAL_DEBUG
  if (!sendFirstValues && millis() > 5000)
  {
    sendFirstValues = true;
    usbSerial.println(screen.printf("MAC= %s, Channel= %d",
                                    WiFi.macAddress().c_str(),
                                    WiFi.channel()));
  }
#endif

  if ((millis() - ledFlashTimestamp) > 500)
  {
    ledFlashTimestamp = millis();

    ledState = !ledState;
    digitalWrite(LED, ledState);
  }

  for (int x = 0; x < RECV_DATA_AMOUNT; x++)
  {
    RecvData *data = &recvData[x];
    if (data->inUse)
    {
      parseReceivedData(data);
      data->inUse = false;
    }
  }
}
