#include "Eyes.h"

// Module-scope storage set by EyesInit()
static Display290* sDisp   = nullptr;
static U8G2_FOR_ADAFRUIT_GFX* sFonts = nullptr;
static uint16_t sBG = GxEPD_WHITE;
static uint16_t sFG = GxEPD_BLACK;

static inline void ensureInit() {
  if (!sDisp || !sFonts) {
    // Hard fail helps catch misuse during development
    // You can replace with `return;` if you prefer silent no-ops.
    Serial.println("Eyes not initialized. Call EyesInit() first.");
    abort();
  }
}


void EyesInit(Display290& display, U8G2_FOR_ADAFRUIT_GFX& fonts) {
  sDisp = &display;
  sFonts = &fonts;
}

void EyesSetColors(uint16_t background, uint16_t foreground) {
  sBG = background;
  sFG = foreground;
}

void setDisplayStandard() {
  ensureInit();
  sDisp->setRotation(3);
  sFonts->setFontMode(1);
  sFonts->setFontDirection(0);
  sFonts->setForegroundColor(sFG);
  sFonts->setBackgroundColor(sBG);
}

void displayEyes(int emote, bool refreshScreen) {
  ensureInit();
  sDisp->setRotation(3);
  sDisp->setPartialWindow(0, 0, sDisp->width(), sDisp->height());
  sDisp->firstPage();
  do {
    if (refreshScreen) sDisp->fillScreen(sBG);

    // Left & right eye bases
    sDisp->fillCircle(90, 60, 35, GxEPD_BLACK);
    sDisp->fillCircle(210, 60, 35, GxEPD_BLACK);

    switch (emote) {
      case -1: break;
      case 0:
        sDisp->fillCircle(90, 65, 8, GxEPD_WHITE);
        sDisp->fillCircle(210, 65, 8, GxEPD_WHITE);
        break;
      case 2:
        sDisp->fillCircle(90, 65, 14, GxEPD_WHITE);
        sDisp->fillCircle(210, 65, 14, GxEPD_WHITE);
        Serial.println("Showing wide");
        break;
      case 6:
        sDisp->fillCircle(90, 65, 14, GxEPD_WHITE);
        sDisp->fillCircle(210, 65, 14, GxEPD_WHITE);
        sDisp->fillCircle(90, 75, 14, GxEPD_BLACK);
        sDisp->fillCircle(210, 75, 14, GxEPD_BLACK);
        Serial.println("Showing happy");
        break;
      case 7:
        sDisp->fillCircle(90, 65, 14, GxEPD_WHITE);
        sDisp->fillCircle(210, 65, 14, GxEPD_WHITE);
        sDisp->fillCircle(90, 55, 14, GxEPD_BLACK);
        sDisp->fillCircle(210, 55, 14, GxEPD_BLACK);
        Serial.println("Showing sad");
        break;
    }
  } while (sDisp->nextPage());
}

void displayEyesSymbol(const char* symbol, bool refreshScreen) {
  ensureInit();
  sDisp->setRotation(3);
  sDisp->setPartialWindow(0, 0, sDisp->width(), sDisp->height());
  sDisp->firstPage();
  do {
    if (refreshScreen) sDisp->fillScreen(sBG);

    sDisp->fillCircle(90, 60, 35, GxEPD_BLACK);
    sDisp->fillCircle(210, 60, 35, GxEPD_BLACK);

    sDisp->setTextColor(GxEPD_WHITE);
    sDisp->setTextSize(4);
    sDisp->setCursor(80, 50);
    sDisp->print(symbol);
    sDisp->setCursor(200, 50);
    sDisp->print(symbol);
  } while (sDisp->nextPage());
}

void displayText0(const char* inputText, bool refreshScreen) {
  ensureInit();
  setDisplayStandard();
  sFonts->setFont(u8g2_font_logisoso32_tr);
  uint16_t y = 75;

  sDisp->setPartialWindow(0, 0, sDisp->width(), 296);
  sDisp->firstPage();
  do {
    if (refreshScreen) sDisp->fillScreen(sBG);
    sFonts->setCursor(10, y);
    sFonts->print(inputText);
  } while (sDisp->nextPage());
}

void displayTextAdv(const char* inputText, int fontSize, uint16_t color,
                    int cX, int cY, bool refreshScreen) {
  ensureInit();
  sDisp->setRotation(3);
  sDisp->setPartialWindow(0, 0, sDisp->width(), sDisp->height());
  sDisp->firstPage();
  do {
    if (refreshScreen) sDisp->fillScreen(sBG);
    sDisp->setTextColor(color);
    sDisp->setTextSize(fontSize);
    sDisp->setCursor(cX, cY);
    sDisp->print(inputText);
  } while (sDisp->nextPage());
}

void displayServerState(bool displayOnScreen, bool refreshScreen,
                        bool hasClient, String lastMessage) {
  ensureInit();
  Serial.println("---");
  Serial.println("Server State");
  Serial.print("Connected: ");
  Serial.println(hasClient ? "True" : "False");
  Serial.print("Last: ");
  Serial.println(lastMessage);
  if (!displayOnScreen) return;

  sFonts->setFont(u8g2_font_t0_16_tf);
  uint16_t x = 40, y = 25;

  sDisp->setPartialWindow(0, 0, sDisp->width(), 296);
  sDisp->firstPage();
  do {
    if (refreshScreen) sDisp->fillScreen(sBG);
    sFonts->setCursor(x, y + 25);     sFonts->print("Connected:");
    sFonts->setCursor(x + 100, y + 25); sFonts->print(hasClient ? "True" : "False");
    sFonts->setCursor(x, y + 75);     sFonts->print("Last:");
    sFonts->setCursor(x + 100, y + 75); sFonts->print(lastMessage);
  } while (sDisp->nextPage());
}