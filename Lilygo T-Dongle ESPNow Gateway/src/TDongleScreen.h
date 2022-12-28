#ifndef TDONGLE_SCREEN_H
#define TDONGLE_SCREEN_H

#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>

#define DATA_COUNT_MAX 7
#define DATA_STR_MAX_LEN 40
#define TFT_BACKGROUND TFT_SKYBLUE


class TDongleScreen
{
public:
    TDongleScreen();

    void init();

    const char* printf(const char* format, ...);

    const char* addLine(const char* str);
    const char* addLine(String str)
    {
        return addLine(str.c_str());
    }

private:
    TFT_eSPI _tft; // Invoke library, pins defined in User_Setup.h

    char _dataRegister[DATA_COUNT_MAX][DATA_STR_MAX_LEN];
    uint8_t _dataCount = 0;

    void shiftDataRegister();
    void updateScreen();
};

#endif