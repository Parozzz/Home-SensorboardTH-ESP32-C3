#include <Arduino.h>

#include <esp_wifi.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESPNowBuffer.h>
#include <ESPNowMessages.h>
#include <esp_efuse.h>
#include <esp_efuse_table.h>

#define ESP32_C3_MAC                   \
  {                                    \
    0xA0, 0x76, 0x4E, 0x4B, 0xC0, 0x70 \
  }

#define LILYGO_TDONGLE_MAC             \
  {                                    \
    0x58, 0xCF, 0x79, 0x32, 0xE1, 0xC2 \
  }

#define USB_IN_PIN GPIO_NUM_1

#define ESP_READY_PIN GPIO_NUM_3
#define ESPRDY_DONE_IMPULSES 5

#define WIFI_CHANNEL 7

#define TERMINATOR_BYTE 0xFE
#define TERMINATOR_BYTE_COUNT 3

#define ID_SW0 GPIO_NUM_6
#define ID_SW1 GPIO_NUM_5
#define ID_SW2 GPIO_NUM_4

//#define DEBUG

#define DEBUG_WITH_UART
#ifdef DEBUG
#ifdef DEBUG_WITH_UART
Print *debug = &Serial0;
#else
Print *debug = &Serial;
#endif
#endif

// ESP_OK = 0
// ESP_FAIL = -1
#define ESP_WAIT -2

const uint8_t slaveMAC[6] = LILYGO_TDONGLE_MAC;

uint8_t recvMacAddress[6];
ESPNowBuffer recvESPNowBuffer;

int8_t sendESPNowStatus;
ESPNowBuffer sendESPNowBuffer;

void OnESPNowSend(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  sendESPNowStatus = status == esp_now_send_status_t::ESP_NOW_SEND_SUCCESS ? ESP_OK : ESP_FAIL;
}

void OnESPNowRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  if (len > 3 && !recvESPNowBuffer.isInUse())
  {
    memcpy(recvMacAddress, mac, 6);

    recvESPNowBuffer.copyData(incomingData, len);
    recvESPNowBuffer.setInUse(true);
  }
}

bool initWiFi()
{
  WiFi.softAP("dummy_ssid", "dummy_ssid", WIFI_CHANNEL);

  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);

  if (WiFi.enableSTA(true))
  {
    WiFi.setSleep(wifi_ps_type_t::WIFI_PS_NONE);
    WiFi.setTxPower(wifi_power_t::WIFI_POWER_18_5dBm);
    WiFi.disconnect();
    return true;
  }

  return true;
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

/*
When the value of eFuse field EFUSE_UART_PRINT_CONTROL is
0 (default), print is enabled and not controlled by GPIO8.
1, if GPIO8 is 0, print is enabled; if GPIO8 is 1, it is disabled.
2, if GPIO8 is 0, print is disabled; if GPIO8 is 1, it is enabled.
3, print is disabled and not controlled by GPIO8.
*/
void setSerialDebugEFuse()
{
  int size = esp_efuse_get_field_size(ESP_EFUSE_UART_PRINT_CONTROL);

  uint8_t efuse = 0;
  esp_efuse_read_field_blob(ESP_EFUSE_UART_PRINT_CONTROL, &efuse, size);

  if (efuse != 3)
  {
    efuse = 3;
    esp_efuse_write_field_blob(ESP_EFUSE_UART_PRINT_CONTROL, &efuse, size);
  }
}

void sleep()
{
  esp_wifi_stop();

  ESP.deepSleep(0xFFFFFFFF);
  delay(0xFFFF);
}

void setup()
{
  // The ATTINY has a pull_up on this pin, so it should always start HIGH
  digitalWrite(ESP_READY_PIN, HIGH); // I Write the pin first otherwise it will create a very small (Like 100us) impulse low. This way it does not.
  pinMode(ESP_READY_PIN, OUTPUT);

  setSerialDebugEFuse();

  pinMode(ID_SW0, INPUT_PULLUP);
  pinMode(ID_SW1, INPUT_PULLUP);
  pinMode(ID_SW2, INPUT_PULLUP);

  pinMode(USB_IN_PIN, INPUT);

#ifdef DEBUG
  Serial.begin(115200); // USB Serial
#endif

  Serial0.begin(115200); // UART

  bool initOK = true;

#ifdef DEBUG
  debug->println("Init WiFi.");
#endif
  initOK &= initWiFi();

#ifdef DEBUG
  debug->println("Init ESP_NOW.");
#endif
  initOK &= initESPNow();

#ifdef DEBUG
  debug->printf("BEGIN %s \n", initOK ? "OK" : "FAIL");
#endif
  if (!initOK)
  {
    sleep();
  }
}

uint8_t readID()
{
  uint8_t bit0 = digitalRead(ID_SW0) ^ 0x01;
  uint8_t bit1 = digitalRead(ID_SW1) ^ 0x01;
  uint8_t bit2 = digitalRead(ID_SW2) ^ 0x01;

  uint8_t id = bit0 | bit1 << 1 | bit2 << 2;
#ifdef DEBUG
  debug->printf("ID - Value=%d\n", id);
#endif
  return id;
}

// Generate some impulses to signal the TINY for wAKEUP / SEND_OK
void generateESPRDYImpulses()
{
  for (int x = 0; x < ESPRDY_DONE_IMPULSES; x++)
  {
    digitalWrite(ESP_READY_PIN, HIGH);
    delay(1);
    digitalWrite(ESP_READY_PIN, LOW);
    delay(1);
  }

  digitalWrite(ESP_READY_PIN, HIGH);
}

int8_t waitDataFromTINY()
{
#ifdef DEBUG
  debug->println("TINY - Waiting data");
#endif

  digitalWrite(ESP_READY_PIN, LOW);

  uint8_t buffer[ESPNOW_BUFFER_MAX];
  uint16_t bufferCount = 0;
  uint8_t terminatorByteCount = 0;

  uint32_t timestamp = millis();
  while (terminatorByteCount < TERMINATOR_BYTE_COUNT)
  {
    yield();

    // If the usb is plugged in, i wait indefinately until i receive data.
    if ((millis() - timestamp) > 1000) // Max 2 seconds of waiting.
    {
#ifdef DEBUG
      debug->println("TINY - Data TIMEOUT");
#endif
      return ESP_FAIL;
    }

    if (Serial0.available())
    {
      timestamp = millis(); // If i receive via serial, i update the timestamp to avoid triggering it while receiving data.

      uint8_t read = Serial0.read();
      buffer[bufferCount++] = read;
      if (read == TERMINATOR_BYTE)
      {
        terminatorByteCount++;
      }
    }
  }

  sendESPNowBuffer.copyData(buffer, bufferCount - TERMINATOR_BYTE_COUNT); // Remove the terminator bytes because i don't want them to be sent to ESPNOW
  sendESPNowBuffer.setInUse(true);

#ifdef DEBUG
    debug->print("HEX DUMP = ");
    for (int x = 0; x < sendESPNowBuffer.getLen(); x++)
    {
      debug->printf("%.2x,", sendESPNowBuffer[x]);
    }
    debug->println();
    ESPNowPrintDebug(&sendESPNowBuffer, debug);
#endif

  if (!sendESPNowBuffer.isChecksumValid() || !ESPNowDoSanityCheck(&sendESPNowBuffer))
  {
#ifdef DEBUG
    bool checksumOK = sendESPNowBuffer.isChecksumValid();
    bool sanityOK = ESPNowDoSanityCheck(&sendESPNowBuffer);
    debug->printf("TINY - Invalid received data. Checksum=%s, Sanity=%s \n", checksumOK ? "OK" : "FAIL", sanityOK  ? "OK" : "FAIL");
#endif
    return ESP_FAIL;
  }

  // This needs to be AFTER copying data, otherwise is useless.
  sendESPNowBuffer.setByte(1, readID()); // Byte 1 of a message is ALWAYS the id. Overriding it.
  sendESPNowBuffer.updateChecksum();

#ifdef DEBUG
  debug->printf("TINY - Data Received. Len= %d, Checksum. Recv=%d Calc=%d %s, Data= ",
                sendESPNowBuffer.getLen(),
                sendESPNowBuffer.getChecksum(),
                sendESPNowBuffer.createChecksum(),
                sendESPNowBuffer.isChecksumValid() ? "VALID" : "INVALID");
#endif

  delay(5);
  generateESPRDYImpulses(); // Generate RDY Impulses to indicate that the receiving is done correctly.

  return ESP_OK;
}

int8_t sendESPNowData()
{
#ifdef DEBUG
  debug->println("ESPNow - Adding peer");
#endif

  esp_now_peer_info peerInfo;
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.ifidx = wifi_interface_t::WIFI_IF_STA;
  peerInfo.encrypt = false;
  memcpy(peerInfo.peer_addr, slaveMAC, 6);
  esp_now_add_peer(&peerInfo);

  int8_t espnowResult = ESP_FAIL;
  for (int x = 0; x < 3; x++)
  {
#ifdef DEBUG
    debug->printf("ESPNow - Sending data n.%d \n", (x + 1));
#endif
    esp_now_send(slaveMAC, (uint8_t *)sendESPNowBuffer.getBuffer(), sendESPNowBuffer.getLen());

    uint32_t recvFeedbackTimestamp = millis();
    while (!recvESPNowBuffer.isInUse())
    {
      yield();
      if (millis() - recvFeedbackTimestamp > 750)
      {
#ifdef DEBUG
        debug->printf("ESPNow - Response timeout n.%D \n", (x + 1));
#endif
        break;
      }
    }

    if (!recvESPNowBuffer.isInUse())
    {
      continue;
    }

    espnowResult = ESP_OK;
    break;
  }

  esp_now_del_peer(peerInfo.peer_addr);

#ifdef DEBUG
  debug->printf("ESPNow - Exchange Result=%s, RecvMac=", espnowResult == ESP_OK ? "OK" : "FAIL");
  for (int x = 0; x < 6; x++)
  {
    debug->print(recvMacAddress[x], HEX);
    if (x != 5)
    {
      debug->print(":");
    }
  }
  debug->println();
#endif

  return espnowResult;
}

void loop()
{
  if (waitDataFromTINY() == ESP_OK && sendESPNowData() == ESP_OK)
  {
#ifdef DEBUG
    debug->println("TINY - Sending DONE Impulses.");
#endif
    generateESPRDYImpulses();
  }

#ifdef DEBUG
  debug->println("==============");
  delay(50);
#endif

  sleep();
}
/*
sendESPNowStatus = ESP_WAIT;

uint32_t sendFeedbackTimestamp = millis();
while (sendESPNowStatus == ESP_WAIT)
{
  yield();
  if (millis() - sendFeedbackTimestamp > 500)
  {
    break;
  }
}

if (sendESPNowStatus != ESP_OK)
{
  continue;
}
*/