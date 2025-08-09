#define ENABLE_GxEPD2_GFX 1
#include <GxEPD2_BW.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <WiFi.h>
#include <map>
#include <Servo.h>
#include <vector>

#define SERVO_BASE_PIN 25
#define SERVO_LINK_1_PIN 26

#define SLIDE_PIN 0 // TODO SET PIN
#define ROT_PIN 0 // TODO SET PIN
#define MIC_PIN 34
#define BTN_PIN 39

#define CS_PIN 5
#define DC_PIN 17
#define RST_PIN 16
#define BUSY_PIN 4

#define MAX_DISPLAY_BUFFER_SIZE 800 
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8) ? EPD::HEIGHT : MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8))
GxEPD2_BW<GxEPD2_290_BS, GxEPD2_290_BS::HEIGHT> display(GxEPD2_290_BS(/*CS=D8*/ SS, /*DC=D3*/ DC_PIN, /*RST=D4*/ RST_PIN, /*BUSY=D2*/ BUSY_PIN)); // DEPG0290BS 128x296, SSD1680
uint16_t bg = GxEPD_WHITE;
uint16_t fg = GxEPD_BLACK;
U8G2_FOR_ADAFRUIT_GFX u8g2Fonts;

const char* ssid = "DraconE";
const char* password = "Rarceth1996!";

IPAddress static_ip(192,168,4,51);
IPAddress dns(192,168,4,1);
IPAddress gateway(192,168,4,1);
IPAddress subnet(255, 255, 255, 0);

int port = 8888;  //Port number
WiFiServer server(port);

unsigned long lastEventTime = 0;
unsigned long nextInterval = 0;

bool hasClient = false;
bool alreadyConnected = false;
String lastMessage = "";

Servo baseServo;
Servo linkOneServo;

int baseServoC;
int linkOneServoC;

bool readingMic = false;
int micDelay = 5;
std::vector<int> micBuffer;

// Notes

// Wiring: 
// Servo orange wires, base to 25, l1 to 26
// battery to pos/neg
// servos to pos/neg
// esp32 ground to neg
// esp32 power via usb
// eink to pos/neg
// eink y18 o5 g17 w16 p4 b23
// y->p are in order, b23 is the outlier at the far end
// Button signal to 39
// Mic signal to 34

void displayEyes(int emote, bool refreshScreen = false);
void displayEyesSymbol(const char* symbol, bool refreshScreen = false);
void displayText0(const char* inputText, bool refreshScreen = false);
void displayServerState(bool displayOnScreen = false, bool refreshScreen = false);
void displayTextAdv(const char* inputText, int fontSize = 4, uint16_t color = GxEPD_WHITE, int cX = 120, int cY = 80, bool refreshScreen = false);
void setDisplayStandard();
void scheduleInterval();
void debug_network();
void blinkEvent();
void servoTest();
void servoReset();
void setBaseServo (int angle);
void setLinkOneServo (int angle);
float inverseLerp(float a, float b, float x);
float lerp(float start_val, float end_val, float fraction);
void g_base();
void g_yes();
void g_no();
void g_search();
void g_full();
void parseLastMessage();
bool processMic();
void uploadMic(WiFiClient& client);

void setup()
{
  Serial.begin(115200);

  analogSetAttenuation(ADC_11db);
  micBuffer.reserve(1024);

  randomSeed(analogRead(0));

  display.init();
  u8g2Fonts.begin(display); // connect u8g2 procedures to Adafruit GFX

  delay(500);

  baseServo.attach(SERVO_BASE_PIN);
  linkOneServo.attach(SERVO_LINK_1_PIN);

  //pinMode(SLIDE_PIN, INPUT);
  //pinMode(ROT_PIN, INPUT);

  displayEyesSymbol("|", true);

  WiFi.mode(WIFI_STA);
  WiFi.config(static_ip, gateway, subnet, dns);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
  }

  g_base();
  delay(200);
  displayEyes(7);
  delay(200);
  displayEyes(0);

  Serial.print("W: "); Serial.println(display.width());
  Serial.print("H: "); Serial.println(display.height());

  displayServerState();
  debug_network();

  server.begin(port);

  scheduleInterval();
}

void loop()
{
  WiFiClient client = server.available();
  if (client) {
    if (client.connected())
    {
      if (!alreadyConnected)
      {
        Serial.println("New client connected");
        client.flush();
        alreadyConnected = true;
        scheduleInterval();
      }
    }

    hasClient = true;

    displayServerState();

    while (client.connected())
    {
      bool uploadMicFlag = processMic();
      
      if (uploadMicFlag)
      {
        uploadMic(client);
      }
      int length = client.available();
      if (client.available() > 0)
      {
        String incoming = client.readStringUntil('\n');
        incoming.trim();
        lastMessage = incoming;

        Serial.println("---");
        Serial.print("Data received: ");
        Serial.print(lastMessage);
        Serial.println();

        displayServerState();

        displayEyesSymbol("!", true);
        delay(500);
        
        parseLastMessage();

        displayEyes(0);

        scheduleInterval();
      }
    }
    client.stop();
    Serial.println("Client disconnected");
    hasClient = false;
    displayServerState();
  }

  /*
  if (millis() - lastEventTime > nextInterval)
  {
    blinkEvent();
    scheduleInterval();
  }
  */

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

  delay(100);  
}

bool processMic() {
  bool btnValue = analogRead(BTN_PIN) < 1000;
  bool uploadMicFlag = false;
  
  if (btnValue && !readingMic)
  {
    Serial.println("Button pressed");
    readingMic = true;
    micBuffer.clear();
  } else if (!btnValue && readingMic)
  {
    Serial.println("Button released");
    readingMic = false;
    uploadMicFlag = true;

    for (int i = 0; i < micBuffer.size(); i++)
    {
      Serial.println(micBuffer[i]);
    }
    Serial.flush();
  }
  
  if (readingMic)
  {
    int m = analogRead(MIC_PIN);
    int barLength = map(m, 0, 4095, 0, 20);  // Adjust for ESP32's 12-bit ADC
    
    //micBuffer.push_back(barLength);
    micBuffer.push_back(m);
    delay(micDelay);
  }

  return uploadMicFlag;
}

void uploadMic(WiFiClient& client) {
  String data = "MIC||";

  Serial.print("Sending ");
  Serial.print(micBuffer.size());
  Serial.println(" entries to client");

  for (int i = 0; i < micBuffer.size(); ++i) {
    data += micBuffer[i];
    if (i < micBuffer.size() - 1) {
      data += ",";
    }
  }

  data += "||";
  client.print(data);
  Serial.println("Sent");
  delay(100);
  client.flush();
}

void displayEyes(int emote, bool refreshScreen) {
  display.setRotation(3);
  display.setPartialWindow(0, 0, display.width(), display.height());
  display.firstPage();
  do {
    if (refreshScreen)
    {
      display.fillScreen(bg);
    }

    // Draw left eye
    display.fillCircle(90, 60, 35, GxEPD_BLACK);

    // Draw right eye
    display.fillCircle(210, 60, 35, GxEPD_BLACK);

    switch (emote) {
      case -1: // No eyes
        break;
      case 0: // Standard
        display.fillCircle(90, 65, 8, GxEPD_WHITE);
        display.fillCircle(210, 65, 8, GxEPD_WHITE);
        break;
      case 2: // Wide
        display.fillCircle(90, 65, 14, GxEPD_WHITE);
        display.fillCircle(210, 65, 14, GxEPD_WHITE);
        Serial.println("Showing wide");
        break;
      case 6: // Happy
        display.fillCircle(90, 65, 14, GxEPD_WHITE);
        display.fillCircle(210, 65, 14, GxEPD_WHITE);
        display.fillCircle(90, 75, 14, GxEPD_BLACK);
        display.fillCircle(210, 75, 14, GxEPD_BLACK);
        Serial.println("Showing happy");
        break;
      case 7: // Sad
        display.fillCircle(90, 65, 14, GxEPD_WHITE);
        display.fillCircle(210, 65, 14, GxEPD_WHITE);
        display.fillCircle(90, 55, 14, GxEPD_BLACK);
        display.fillCircle(210, 55, 14, GxEPD_BLACK);
        Serial.println("Showing sad");
        break;
    }
    

  } while (display.nextPage());
}

void displayEyesSymbol(const char* symbol, bool refreshScreen)
{
  display.setRotation(3);
  display.setPartialWindow(0, 0, display.width(), display.height());
  display.firstPage();
  do {
    if (refreshScreen)
    {
      display.fillScreen(bg);
    }

    // Draw left eye
    display.fillCircle(90, 60, 35, GxEPD_BLACK);
    // Draw right eye
    display.fillCircle(210, 60, 35, GxEPD_BLACK);

    display.setTextColor(GxEPD_WHITE); 
    display.setTextSize(4);            
    display.setCursor(80, 50);         
    display.print(symbol);                

    display.setCursor(200, 50);        
    display.print(symbol);      
    

  } while (display.nextPage());
}

void setDisplayStandard() {
  display.setRotation(3); // 0--> No rotation ,  1--> rotate 90 deg
  u8g2Fonts.setFontMode(1);                 // use u8g2 transparent mode (this is default)
  u8g2Fonts.setFontDirection(0);            // left to right (this is default)
  u8g2Fonts.setForegroundColor(fg);         // apply Adafruit GFX color
  u8g2Fonts.setBackgroundColor(bg);         // apply Adafruit GFX color
}

void displayText0(const char* inputText, bool refreshScreen) {
  setDisplayStandard();
  u8g2Fonts.setFont(u8g2_font_logisoso32_tr); //u8g2_font_logisoso32_tn--->numbers only to save memory ; u8g2_font_logisoso32_tr , u8g2_font_logisoso32_tf -->numbers&letters
  uint16_t x = 165;
  uint16_t y = 75;
  display.setPartialWindow(0, 0, display.width(), 296); //this sets a window for the partial update, so the values can update without refreshing the entire screen.
  display.firstPage();
  do
  {
    if (refreshScreen)
    {
      display.fillScreen(bg);
    }

    u8g2Fonts.setCursor(10, y); 
    u8g2Fonts.print(inputText);
  }
  while (display.nextPage());
}

void displayTextAdv(const char* inputText, int fontSize, uint16_t color, int cX, int cY, bool refreshScreen)
{
  display.setRotation(3);
  display.setPartialWindow(0, 0, display.width(), display.height());
  display.firstPage();
  do {
    if (refreshScreen)
    {
      display.fillScreen(bg);
    }

    display.setTextColor(color); 
    display.setTextSize(fontSize);            
    display.setCursor(cX, cY);         
    display.print(inputText);                 

  } while (display.nextPage());
}

void displayServerState(bool displayOnScreen, bool refreshScreen) {
  Serial.println("---");
  Serial.println("Server State");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Connected: ");
  Serial.println(hasClient ? "True" : "False");
  Serial.print("Last: ");
  Serial.println(lastMessage);

  if (!displayOnScreen)
  {
    return;
  }
  
  u8g2Fonts.setFont(u8g2_font_t0_16_tf);
  uint16_t x = 40;
  uint16_t y = 25;
  display.setPartialWindow(0, 0, display.width(), 296); //this sets a window for the partial update, so the values can update without refreshing the entire screen.
  display.firstPage();
  do
  {
    if (refreshScreen)
    {
      display.fillScreen(bg);
    }
    
    u8g2Fonts.setCursor(x, y); 
    u8g2Fonts.print("IP:");
    u8g2Fonts.setCursor(x + 100, y); 
    u8g2Fonts.print(WiFi.localIP().toString().c_str());

    u8g2Fonts.setCursor(x, y + 25);
    u8g2Fonts.print("Connected:");
    u8g2Fonts.setCursor(x + 100, y + 25);
    u8g2Fonts.print(hasClient ? "True" : "False");

    u8g2Fonts.setCursor(x, y + 75);
    u8g2Fonts.print("Last:");
    u8g2Fonts.setCursor(x + 100, y + 75);
    u8g2Fonts.print(lastMessage);

  }
  while (display.nextPage());
}

void scheduleInterval() {
    nextInterval = random(15000,30000);
}

void blinkEvent() {
    std::map<int,int> rMap;
    rMap[0] = 0;
    rMap[1] = 1;
    rMap[2] = 2;
    rMap[3] = 6;
    rMap[4] = 7;

    lastEventTime = millis();
    displayEyes(-1);
    delay(250);
    int r = random(0,4);
    displayEyes(rMap[r]);
    delay(750);
    displayEyes(-1);
    delay(250);
    displayEyes(0);
}

void servoTest() {
  displayEyesSymbol("8");
  for(int angle = 0; angle <= 180; angle++) {
    setBaseServo(angle);
    setLinkOneServo(angle);
    delay(15); // Small delay for smoother movement
  }

  delay(1000); // Pause for a second at the end

  // Rotate all servos back from 180 to 0 degrees
  for(int angle = 180; angle >= 0; angle--) {
    setBaseServo(angle);
    setLinkOneServo(angle);
    delay(15); // Small delay for smoother movement
  }

  delay(1000); // Pause for a second at the end
}

void servoReset() {
  setBaseServo(90);
  setLinkOneServo(90);
}

void setBaseServo (int angle) {
  angle = max(0, angle);
  angle = min(angle, 180);
  baseServo.write(angle);
  baseServoC = angle;
}

void setLinkOneServo (int angle) {
  angle = max(0, angle);
  angle = min(angle, 180);
  linkOneServo.write(angle);
  linkOneServoC = angle;
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
      displayServerState(true, true);
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
    else {
      displayEyesSymbol("?");
      delay(500);
    }
}

void g_base() {
  displayEyes(0);
  setBaseServo(90);
  setLinkOneServo(110);
}

void g_yes() {
  displayEyes(6, true);
  delay(200);
  setLinkOneServo(linkOneServoC - 25);
  delay(300);
  setLinkOneServo(linkOneServoC + 50);
  delay(300);
  setLinkOneServo(linkOneServoC - 50);
  delay(300);
  setLinkOneServo(linkOneServoC + 50);
  delay(300);
  setLinkOneServo(linkOneServoC - 25);
  displayEyes(0, true);
}

void g_no() {
  displayEyes(7, true);
  delay(200);
  setBaseServo(baseServoC - 25);
  delay(300);
  setBaseServo(baseServoC + 50);
  delay(300);
  setBaseServo(baseServoC - 50);
  delay(300);
  setBaseServo(baseServoC + 50);
  delay(300);
  setBaseServo(baseServoC - 25);
  displayEyes(0, true);
}

void g_search() {
  displayEyesSymbol("?", true);
  delay(200);
  setBaseServo(90);
  setLinkOneServo(90);
  for (int i = 90; i > 60; i -= 5)
  {
    setLinkOneServo(i);
    delay(100);
  }
  for (int i = 90; i < 140; i += 5) {
    setBaseServo(i);
    delay(100);
  }
  for (int i = 140; i > 50; i -= 5) {
    setBaseServo(i);
    delay(100);
  }
  delay(300);
  setBaseServo(90);
  delay(100);
  g_base();
  displayEyes(0, true);
}

void g_full() 
{
  g_base();
  g_yes();
  g_no();
  g_search();
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

/*
void displayValues()
{
  display.fillScreen(GxEPD_WHITE);

  int number; //A0 Value
  number = millis() / 1000;  

  setDisplayStandard();
  //u8g2Fonts.setFont(u8g2_font_helvR14_tf);  // select u8g2 font from here:  https://github.com/olikraus/u8g2/wiki/fntlistall
  
  //u8g2Fonts.setFont(u8g2_font_logisoso32_tr); //u8g2_font_logisoso32_tn--->numbers only to save memory ; u8g2_font_logisoso32_tr , u8g2_font_logisoso32_tf -->numbers&letters
  u8g2Fonts.setFont(u8g2_font_t0_16_tf);
  uint16_t x = 165;
  uint16_t y = 75;
  display.setPartialWindow(0, 0, display.width(), 296); //this sets a window for the partial update, so the values can update without refreshing the entire screen.
  display.firstPage();
  do
  {
    display.fillScreen(bg);

    u8g2Fonts.setCursor(10, y); 
    u8g2Fonts.print("Runtime:");
    u8g2Fonts.setCursor(x, y);
    u8g2Fonts.println(number, 1);
  }
  while (display.nextPage());
}
*/