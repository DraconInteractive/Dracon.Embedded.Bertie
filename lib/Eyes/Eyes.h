#pragma once

#include <GxEPD2_BW.h>
#include <U8g2_for_Adafruit_GFX.h>

using Display290 = GxEPD2_BW<GxEPD2_290_BS, GxEPD2_290_BS::HEIGHT>;


// One-time wiring
void EyesInit(Display290& display, U8G2_FOR_ADAFRUIT_GFX& fonts);

// Optional theme
void EyesSetColors(uint16_t background, uint16_t foreground);

// Drawing API (no more display/fonts args)
void displayEyes(int emote, bool refreshScreen = false);
void displayEyesSymbol(const char* symbol, bool refreshScreen = false);
void displayText0(const char* inputText, bool refreshScreen = false);
void displayTextAdv(const char* inputText,
                    int fontSize = 4,
                    uint16_t color = GxEPD_WHITE,
                    int cX = 120, int cY = 80,
                    bool refreshScreen = false);
void displayServerState(bool displayOnScreen,
                        bool refreshScreen,
                        bool hasClient,
                        String lastMessage);

// Internal standardization (now parameterless)
void setDisplayStandard();