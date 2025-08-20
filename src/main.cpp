#define ENABLE_GxEPD2_GFX 1

#include <WiFi.h>
#include <map>
#include "driver/i2s.h"
#include <vector>
#include <Wire.h>

#include "Eyes.h"
#include "Motors.h"

// Mic
#define I2S_PORT         I2S_NUM_0
#define I2S_SAMPLE_RATE  16000
#define I2S_BITS         I2S_BITS_PER_SAMPLE_16BIT
#define I2S_DMA_BUF_LEN  256
#define I2S_DMA_BUF_CNT  6

#define BTN_PIN 32
#define BTN_THRESHOLD    1000

#define MIC_PIN 34

enum RecState { Idle, Recording };
RecState recState = Idle;

bool readingMic = false;
int micDelay = 5;

// IR
#define IR_ADDR 0x58   // 7-bit address
static uint8_t ir_buf[16];
static int16_t IRx[4], IRy[4];
static uint32_t ir_last_ms = 0;
static const uint16_t IR_PERIOD_MS = 500; // 15 = ~66 Hz

// Other peripherals
#define SLIDE_PIN 0 // TODO SET PIN
#define ROT_PIN 0 // TODO SET PIN

// E-Ink Display (Waveshare 2.9inch)
#define CS_PIN 5
#define DC_PIN 17
#define RST_PIN 16
#define BUSY_PIN 4

#define MAX_DISPLAY_BUFFER_SIZE 800 
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8) ? EPD::HEIGHT : MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8))
Display290 display(GxEPD2_290_BS(/*CS=D8*/ SS, /*DC=D3*/ DC_PIN, /*RST=D4*/ RST_PIN, /*BUSY=D2*/ BUSY_PIN)); // DEPG0290BS 128x296, SSD1680
uint16_t bg = GxEPD_WHITE;
uint16_t fg = GxEPD_BLACK;
U8G2_FOR_ADAFRUIT_GFX u8g2Fonts;

// WIFI / TCP
const char* ssid = "DraconE";
const char* password = "Rarceth1996!";

IPAddress static_ip(192,168,4,51);
IPAddress dns(192,168,4,1);
IPAddress gateway(192,168,4,1);
IPAddress subnet(255, 255, 255, 0);

int port = 8888;  //Port number
WiFiServer server(port);
WiFiClient activeClient;
String lastMessage = "";

// State flags
bool hasClient = false;

// Wiring: 
// Servo orange wires, base to 25, l1 to 26
// battery power -> rail 2
// esp32 power via usb
// esp32 3.3, gnd -> rail 1
// esp32 gnd -> rail 2
// rail 1 gnd -> rail 2 gnd

// eink rail 1
// eink y18 o5 g17 w16 p4 b23
// y->p are in order, b23 is the outlier at the far end

// Mic -> rail 1
// Mic -> 34

// Button -> rail 1
// Button -> 32

// IR -> rail 1
// SDA -> 21
// SCL -> 22

void ir_init();
bool ir_read_frame();

void blinkEvent(); // Not in use

// Utilities
float inverseLerp(float a, float b, float x);
float lerp(float start_val, float end_val, float fraction);

// TCP / MIC
void parseLastMessage();
void setupMic();
void processMic();
void debug_network();

// Flush policy
static uint32_t lastFlushMs = 0;
static uint16_t chunksSinceFlush = 0;
static const uint16_t FLUSH_EVERY_N_CHUNKS = 8;  // tune: 4–16 are typical
static const uint32_t FLUSH_EVERY_MS      = 50;  // tune: 20–100 ms

void setup()
{
  Serial.begin(115200);

  analogSetWidth(12); // 16 bit
  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(MIC_PIN, ADC_11db);

  randomSeed(analogRead(0));

  display.init();
  u8g2Fonts.begin(display); // connect u8g2 procedures to Adafruit GFX

  delay(100);

  EyesInit(display, u8g2Fonts);
  EyesSetColors(GxEPD_WHITE, GxEPD_BLACK);

  delay(500);

  ServoInit();

  //pinMode(SLIDE_PIN, INPUT);
  //pinMode(ROT_PIN, INPUT);

  pinMode(BTN_PIN, INPUT_PULLUP);

  displayEyesSymbol("|", true);

  WiFi.mode(WIFI_STA);
  WiFi.config(static_ip, gateway, subnet, dns);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
  }

  g_base();
  displayEyesSymbol("-", true);

  delay(350);

  Serial.print("W: "); Serial.println(display.width());
  Serial.print("H: "); Serial.println(display.height());

  displayServerState(false, true, hasClient, lastMessage);
  debug_network();

  server.begin(port);

  setupMic();

  ir_init();

  displayEyes(0);
  delay(500);
}

void loop()
{
  WiFiClient newClient = server.available();
  if (newClient && newClient.connected())
  {
    activeClient = newClient;
    hasClient = true;
    Serial.println("Client connected");
    activeClient.flush();
    displayServerState(false, true, hasClient, lastMessage);
  }

  if (hasClient && !activeClient.connected())
  {
    hasClient = false;
    activeClient.stop();
    displayServerState(false, true, hasClient, lastMessage);
  }

  bool btnHeld = digitalRead(BTN_PIN) == LOW;

  if (activeClient && hasClient) {
    switch (recState) {
      case Idle: 
        if (btnHeld)
        {
          Serial.println("Starting stream");
          // Begin stream header
          activeClient.println("MIC_STREAM_BEGIN");
          activeClient.print("sr=");
          activeClient.println(I2S_SAMPLE_RATE);
          activeClient.println("fmt=ADC12LJ");
          activeClient.flush();

          // Start I2S
          i2s_adc_enable(I2S_PORT);
          recState = Recording;
          
          lastFlushMs = millis();
          chunksSinceFlush = 0;
        }
        break;
      case Recording:
        if (!btnHeld || !hasClient)
        {
          i2s_adc_disable(I2S_PORT);
          if (hasClient)
          {
            Serial.println("Ending stream");
            activeClient.println("MIC_STREAM_END");
            activeClient.flush();
          }
          else {
            Serial.println("Lost client, ending stream");
          }
          recState = Idle;
        }
        else {
          processMic();
        }
        break;
    }
    
    if (recState == Idle && activeClient.available() > 0) 
    {
      String incoming = activeClient.readStringUntil('\n');
      incoming.trim();
      lastMessage = incoming;

      Serial.println("---");
      Serial.print("Data received: ");
      Serial.print(lastMessage);
      Serial.println();

      displayServerState(false, true, hasClient, lastMessage);

      displayEyesSymbol("!", true);
      delay(500);
      
      parseLastMessage();

      displayEyes(0);
    }
  }

  if (recState == Idle) {
    // IR cam poll at ~66 Hz
    if (millis() - ir_last_ms >= IR_PERIOD_MS) {
      ir_last_ms = millis();
      if (ir_read_frame()) {
        // Optional: quick serial debug
        // Serial.printf("IR: (%d,%d), (%d,%d), (%d,%d), (%d,%d)\n",
          //IRx[0],IRy[0], IRx[1],IRy[1], IRx[2],IRy[2], IRx[3],IRy[3]);
          if (IRx[0] != 1023 || IRy[0] != 1023) {
            Serial.printf("IR: (%d,%d)\n", IRx[0],IRy[0]);
          }
      }
    }
  }

  /*
  int d = analogRead(SLIDE_PIN);
  float n = inverseLerp(0, 4095, d); // map slide to float
  int m = (int)lerp(0, 180, n); // map float to angle
  Serial.print(" M: ");
  Serial.println(m);

  if (abs(m - baseServoC) > 3)
  {
    baseServo.write(m);
    baseServoC = m;
  }
  */

  switch (recState)
  {
    case Idle:
      delay(100);
      break;
    case Recording:
      delay(10);
      break;
  }
}

void setupMic() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    //.dma_buf_count = I2S_DMA_BUF_CNT,
    //.dma_buf_len = I2S_DMA_BUF_LEN,
    .dma_buf_count = 8,
    .dma_buf_len = 512,
    .use_apll = true,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  
  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_0); // matches ~3.3V full-scale
  i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_6 /* GPIO34 on ESP32 */);
  i2s_set_sample_rates(I2S_PORT, I2S_SAMPLE_RATE);
}

void processMic() {
  static uint8_t chunkBuf[1024];
  size_t bytes_read = 0;
  i2s_read(I2S_PORT, chunkBuf, sizeof(chunkBuf), &bytes_read, 20 / portTICK_PERIOD_MS);

  if (bytes_read > 0 && hasClient) {
    // Sanity print
    if (bytes_read > 0) {
        int16_t *p = (int16_t*)chunkBuf;
        int n = bytes_read / 2;
        uint16_t minw=0xFFFF, maxw=0, minu=4095, maxu=0;
        for (int i=0; i<n; ++i) {
          uint16_t w = (uint16_t)p[i];
          uint16_t u = (w >> 4) & 0x0FFF;
          if (w < minw) minw = w;
          if (w > maxw) maxw = w;
          if (u < minu) minu = u;
          if (u > maxu) maxu = u;
        }
      static bool once=false;
      if (!once) {
        once = true;
        Serial.printf("I2S raw min/max: 0x%04X / 0x%04X  | u12 min/max: %u / %u\n", minw, maxw, minu, maxu);
      }
    }
    activeClient.print("CHUNK ");
    activeClient.println((unsigned)bytes_read);
    activeClient.write(chunkBuf, bytes_read);

    // Flush policy: every N chunks OR after X ms, whichever comes first
    chunksSinceFlush++;
    uint32_t now = millis();
    if (chunksSinceFlush >= FLUSH_EVERY_N_CHUNKS || (now - lastFlushMs) >= FLUSH_EVERY_MS) {
      activeClient.flush();
      chunksSinceFlush = 0;
      lastFlushMs = now;
    }
  }
}

void blinkEvent() {
    std::map<int,int> rMap;
    rMap[0] = 0;
    rMap[1] = 1;
    rMap[2] = 2;
    rMap[3] = 6;
    rMap[4] = 7;

    displayEyes(-1);
    delay(250);
    int r = random(0,4);
    displayEyes(rMap[r]);
    delay(750);
    displayEyes(-1);
    delay(250);
    displayEyes(0);
}

void parseLastMessage() {
    if (lastMessage == "dbi") // debug eyes
    {
      displayEyesSymbol(">", true);
      delay(500);
    }
    else if (lastMessage == "dbsrange") // do full range servo test
    {
      servoTest();
      delay(100);
    }
    else if (lastMessage == "rsts") // reset servo
    {
      servoReset();
    }
    else if (lastMessage == "dbcon") // debug connection
    {
      displayServerState(false, true, hasClient, lastMessage);
      delay(100);
    }
    else if (lastMessage == "dbtxt") // debug sub text
    {
      const char* txt = "Hello!";
      displayTextAdv(txt, 2, GxEPD_BLACK, 128 - (strlen(txt) * 5), 90, true);
      delay(1000);
      const char* txt2 = "Hello friend!";
      displayTextAdv(txt2, 2, GxEPD_BLACK, 128 - (strlen(txt2) * 5), 90, true);
      delay(1000);
      const char* txt3 = "Hello friend! Welcome!";
      displayTextAdv(txt3, 2, GxEPD_BLACK, 128 - (strlen(txt3) * 5), 90, true);
      delay(1000);
    }
    else if (lastMessage == "nod")
    {
      g_yes();
    }
    else if (lastMessage == "shake")
    {
      g_no();
    }
    else if (lastMessage == "search")
    {
      g_search();
    }
    else if (lastMessage == "full")
    {
      g_full();
    }
    else if (lastMessage == "restart")
    {
      ESP.restart();
    }
    else {
      displayEyesSymbol("?");
      delay(500);
    }
}

// Helpers
void ir_write2(uint8_t a, uint8_t b) {
  Wire.beginTransmission(IR_ADDR);
  Wire.write(a);
  Wire.write(b);
  Wire.endTransmission();
}

void ir_init() {
  Wire.begin();
  Wire.setClock(100000);
  delay(5);

  // Init sequence from DFRobot example
  ir_write2(0x30, 0x01); delay(10);
  ir_write2(0x30, 0x08); delay(10);
  ir_write2(0x06, 0x90); delay(10);
  ir_write2(0x08, 0xC0); delay(10);
  ir_write2(0x1A, 0x40); delay(10);
  ir_write2(0x33, 0x33); delay(100);

  // Quick probe
  Wire.beginTransmission(IR_ADDR);
  uint8_t err = Wire.endTransmission();
  Serial.printf("IR cam init: %s\n", err == 0 ? "OK" : "NOT FOUND");
}

// read one frame; returns true if 4 points parsed
bool ir_read_frame() {
  // request data starting at 0x36
  Wire.beginTransmission(IR_ADDR);
  Wire.write(0x36);
  Wire.endTransmission(true);
  uint8_t n = Wire.requestFrom(IR_ADDR, 16, true);
  for (int i = 0; i < 16; i++){
    ir_buf[i] = 0;
  }

  int count = 0;
  while (Wire.available() && count < 16) {
    ir_buf[count] = Wire.read();
    count++;
  }

  // Decode 4 points: x = buf[1|4|7|10] + ((buf[3|6|9|12]&0x30)<<4)
  //                  y = buf[2|5|8|11] + ((buf[3|6|9|12]&0xC0)<<2)
  const uint8_t idx[4][3] = {{1,2,3},{4,5,6},{7,8,9},{10,11,12}};
  for (int i=0;i<4;i++) {
    uint8_t lx = ir_buf[idx[i][0]];
    uint8_t ly = ir_buf[idx[i][1]];
    uint8_t s  = ir_buf[idx[i][2]];
    IRx[i] = (int16_t)lx + ((int16_t)(s & 0x30) << 4);
    IRy[i] = (int16_t)ly + ((int16_t)(s & 0xC0) << 2);
  }
  return true;
}

float inverseLerp(float a, float b, float x) {
  if (a == b) { // Handle the degenerate case where the range is zero
    return (x >= a) ? 1.0 : 0.0; // Or handle as an error, depending on desired behavior
  }
  return (x - a) / (b - a);
}

float lerp(float start_val, float end_val, float fraction) {
  return start_val + (end_val - start_val) * fraction;
}

void debug_network(){
  if(WiFi.status() == WL_CONNECTED) {
      Serial.print("[*] Network information for ");
      Serial.println(ssid);

      Serial.println("[+] BSSID : " + WiFi.BSSIDstr());
      Serial.print("[+] Gateway IP : ");
      Serial.println(WiFi.gatewayIP());
      Serial.print("[+] Subnet Mask : ");
      Serial.println(WiFi.subnetMask());
      Serial.println((String)"[+] RSSI : " + WiFi.RSSI() + " dB");
      Serial.print("[+] ESP32 IP : ");
      Serial.println(WiFi.localIP());
  }
}