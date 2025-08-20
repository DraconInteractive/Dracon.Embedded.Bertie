#include "Motors.h"

#define SERVO_BASE_PIN 25
#define SERVO_LINK_1_PIN 26

Servo baseServo;
Servo linkOneServo;

int baseServoC;
int linkOneServoC;

void ServoInit() {
    baseServo.attach(SERVO_BASE_PIN);
    linkOneServo.attach(SERVO_LINK_1_PIN);
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
