#include <TDongleScreen.h>

TDongleScreen::TDongleScreen()
{
}

void TDongleScreen::init()
{
    _tft.init();
    _tft.setRotation(3);
    _tft.setSwapBytes(true);
    // tft.loadFont(FreeMono9pt7bBitmaps);
    _tft.setTextFont(2);
    // tft.invertDisplay(false);
    _tft.fillScreen(TFT_BACKGROUND);
    //_tft.pushImage(0, 0, 240, 135, Lilygo1);
}

const char* TDongleScreen::printf(const char *format, ...)
{
    shiftDataRegister();

    //Clear the data before writing to it to avoid last string to remain.
    memset(_dataRegister[0], 0, DATA_STR_MAX_LEN);

    va_list args;
    va_start(args, format);
    vsprintf(_dataRegister[0], format, args);
    va_end(args);

    _dataCount = min(_dataCount + 1, DATA_COUNT_MAX);
    updateScreen();

    return _dataRegister[0];
}

const char* TDongleScreen::addLine(const char *str)
{
    shiftDataRegister();

    uint16_t moveCount = min(DATA_STR_MAX_LEN, (int)strlen(str));
    strncpy(_dataRegister[0], str, moveCount); // DST, SRC, NUM

    _dataCount = min(_dataCount + 1, DATA_COUNT_MAX);
    updateScreen();

    return _dataRegister[0];
}

void TDongleScreen::shiftDataRegister()
{
    for (int x = (DATA_COUNT_MAX - 1); x > 0; x--)
    {
        strcpy(_dataRegister[x], _dataRegister[x - 1]); // DST , SRC
    }
}

void TDongleScreen::updateScreen()
{
    _tft.fillScreen(TFT_BACKGROUND);

    for (int x = 0; x < _dataCount; x++)
    {
        const char *str = _dataRegister[x];
        if (str != nullptr)
        {
            _tft.setTextColor(TFT_BLACK);
            _tft.drawString(_dataRegister[x], 0, 20 * x);
        }
    }
}