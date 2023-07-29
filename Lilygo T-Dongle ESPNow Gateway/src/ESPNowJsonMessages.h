#ifndef ESPNOW_JSON_MESSAGED
#define ESPNOW_JSON_MESSAGED

#include <Arduino.h>
#include <cJSON.h>
#include <cJSON_Utils.h>
#include <ESPNowMessages.h>
#include <Constants.h>

cJSON *createJsonRoundedValue(float value)
{
  return cJSON_CreateNumber(round(value * 100));
}

void printUnformatted(Print *serialPrint, cJSON* json)
{
    char *jsonString = cJSON_PrintUnformatted(json);
    serialPrint->println(jsonString);
    serialPrint->flush();
    free(jsonString);
}

void printSensorboardJSON(Print *serialPrint, SensorboardV2Message message)
{
    cJSON *mainJson = cJSON_CreateObject();
    cJSON_AddItemToObject(mainJson, "MessageType", cJSON_CreateNumber(message.getMessageType()));
    cJSON_AddItemToObject(mainJson, "BoardType", cJSON_CreateNumber(message.boardType));
    cJSON_AddItemToObject(mainJson, "BoardID", cJSON_CreateNumber(message.id));

    cJSON *dataJson = cJSON_CreateObject();
    cJSON_AddItemToObject(mainJson, "Data", dataJson);

    if(message.temperature > SENSORBOARD_MIN_TEMP && message.temperature < SENSORBOARD_MAX_TEMP)
    {
      cJSON_AddItemToObject(dataJson, "temperature", createJsonRoundedValue(message.temperature));
    }
    
    if(message.humidity > HUMIDITY_MIN_TEMP && message.humidity < HUMIDITY_MAX_TEMP)
    {
      cJSON_AddItemToObject(dataJson, "humidity", cJSON_CreateNumber(message.humidity));
    }
    
    cJSON_AddItemToObject(dataJson, "charging", cJSON_CreateNumber(message.charging));
    cJSON_AddItemToObject(dataJson, "battery_voltage", createJsonRoundedValue(message.batteryVoltage));

    printUnformatted(serialPrint, mainJson);
    cJSON_Delete(mainJson);
}

void printErrorJSON(Print *serialPrint, ErrorMessage message)
{
    cJSON *mainJson = cJSON_CreateObject();
    cJSON_AddItemToObject(mainJson, "MessageType", cJSON_CreateNumber(message.getMessageType()));
    cJSON_AddItemToObject(mainJson, "BoardType", cJSON_CreateNumber(message.boardType));
    cJSON_AddItemToObject(mainJson, "BoardID", cJSON_CreateNumber(message.id));

    cJSON *dataJson = cJSON_CreateObject();
    cJSON_AddItemToObject(mainJson, "Data", dataJson);

    cJSON_AddItemToObject(dataJson, "code1", createJsonRoundedValue(message.code1));
    cJSON_AddItemToObject(dataJson, "code2", createJsonRoundedValue(message.code2));
    cJSON_AddItemToObject(dataJson, "code3", createJsonRoundedValue(message.code3));
    cJSON_AddItemToObject(dataJson, "battery_voltage", createJsonRoundedValue(message.batteryVoltage));

    printUnformatted(serialPrint, mainJson);
    cJSON_Delete(mainJson);
}




#endif