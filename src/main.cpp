#define ENABLE_GxEPD2_GFX 1
#include <GxEPD2_BW.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <WiFi.h>
#include <map>
#include <Servo.h>

#define SERVO_BASE_PIN 25
#define SERVO_LINK_1_PIN 26

#define SLIDE_PIN 33
#define ROT_PIN 0 // TODO SET PIN

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

void displayEyes(int emote);
void displayEyesSymbol(const char* symbol);
void displayValues();
void displayText0(const char* inputText);
void displayServerState();
void setDisplayStandard();
void scheduleInterval();
void debug_network();
void blinkEvent();
void servoTest();
void servoReset();
float inverseLerp(float a, float b, float x);
float lerp(float start_val, float end_val, float fraction);

void setup()
{
  Serial.begin(115200);

  randomSeed(analogRead(0));

  display.init();
  u8g2Fonts.begin(display); // connect u8g2 procedures to Adafruit GFX

  delay(500);

  baseServo.attach(SERVO_BASE_PIN);
  linkOneServo.attach(SERVO_LINK_1_PIN);

  pinMode(SLIDE_PIN, INPUT);
  //pinMode(ROT_PIN, INPUT);

  displayEyesSymbol("...");

  WiFi.mode(WIFI_STA);
  WiFi.config(static_ip, gateway, subnet, dns);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
  }

  delay(200);
  displayEyes(7);
  delay(200);

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
        displayEyesSymbol("!");
        delay(500);
        
        if (lastMessage == "test")
        {
          displayEyesSymbol(">");
          delay(500);
        }
        else if (lastMessage == "servo")
        {
          servoTest();
          delay(100);
        }
        else if (lastMessage == "servoreset")
        {
          servoReset();
        }
        else {
          displayEyesSymbol("?");
          delay(500);
        }
        displayEyes(0);

        scheduleInterval();
      }
    }
    client.stop();
    Serial.println("Client disconnected");
    hasClient = false;
    displayServerState();
  }
  //displayValues();

  /*
  if (millis() - lastEventTime > nextInterval)
  {
    blinkEvent();
    scheduleInterval();
  }
  */

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

  delay(100);  
}

void displayEyes(int emote) {
  display.setRotation(1);
  display.setPartialWindow(0, 0, display.width(), display.height());
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);

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
        break;;
      case 6: // Happy
        display.fillCircle(90, 65, 14, GxEPD_WHITE);
        display.fillCircle(210, 65, 14, GxEPD_WHITE);
        display.fillCircle(90, 75, 14, GxEPD_BLACK);
        display.fillCircle(210, 75, 14, GxEPD_BLACK);
      case 7: // Sad
        display.fillCircle(90, 65, 14, GxEPD_WHITE);
        display.fillCircle(210, 65, 14, GxEPD_WHITE);
        display.fillCircle(90, 55, 14, GxEPD_BLACK);
        display.fillCircle(210, 55, 14, GxEPD_BLACK);
    }
    

  } while (display.nextPage());
}

void displayEyesSymbol(const char* symbol)
{
  display.setRotation(1);
  display.setPartialWindow(0, 0, display.width(), display.height());
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);

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
  display.setRotation(1); // 0--> No rotation ,  1--> rotate 90 deg
  u8g2Fonts.setFontMode(1);                 // use u8g2 transparent mode (this is default)
  u8g2Fonts.setFontDirection(0);            // left to right (this is default)
  u8g2Fonts.setForegroundColor(fg);         // apply Adafruit GFX color
  u8g2Fonts.setBackgroundColor(bg);         // apply Adafruit GFX color
}

void displayText0(const char* inputText) {
  display.fillScreen(GxEPD_WHITE);
  setDisplayStandard();
  u8g2Fonts.setFont(u8g2_font_logisoso32_tr); //u8g2_font_logisoso32_tn--->numbers only to save memory ; u8g2_font_logisoso32_tr , u8g2_font_logisoso32_tf -->numbers&letters
  uint16_t x = 165;
  uint16_t y = 75;
  display.setPartialWindow(0, 0, display.width(), 296); //this sets a window for the partial update, so the values can update without refreshing the entire screen.
  display.firstPage();
  do
  {
    display.fillScreen(bg);

    u8g2Fonts.setCursor(10, y); 
    u8g2Fonts.print(inputText);
  }
  while (display.nextPage());
}

void displayServerState() {
  Serial.println("---");
  Serial.println("Server State");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Connected: ");
  Serial.println(hasClient ? "True" : "False");
  Serial.print("Last: ");
  Serial.println(lastMessage);

  return; // TEMP

  display.fillScreen(GxEPD_WHITE);
  setDisplayStandard();
  u8g2Fonts.setFont(u8g2_font_t0_16_tf);
  uint16_t x = 40;
  uint16_t y = 25;
  display.setPartialWindow(0, 0, display.width(), 296); //this sets a window for the partial update, so the values can update without refreshing the entire screen.
  display.firstPage();
  do
  {
    display.fillScreen(bg);

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
    baseServo.write(angle);
    linkOneServo.write(angle);
    baseServoC = angle;
    linkOneServoC = angle;
    delay(15); // Small delay for smoother movement
  }

  delay(1000); // Pause for a second at the end

  // Rotate all servos back from 180 to 0 degrees
  for(int angle = 180; angle >= 0; angle--) {
    baseServo.write(angle);
    linkOneServo.write(angle);
    baseServoC = angle;
    linkOneServoC = angle;
    delay(15); // Small delay for smoother movement
  }

  delay(1000); // Pause for a second at the end
}

void servoReset() {
  baseServo.write(90);
  linkOneServo.write(90);
  baseServoC = 90;
  linkOneServoC = 90;
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