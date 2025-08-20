#pragma once

#include <Servo.h>
#include "Eyes.h"

void ServoInit();

void servoTest();
void servoReset();
void setBaseServo (int angle);
void setLinkOneServo (int angle);

void g_base();
void g_yes();
void g_no();
void g_search();
void g_full();