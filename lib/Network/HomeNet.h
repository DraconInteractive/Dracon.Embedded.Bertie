#pragma once

#include <WiFi.h>

void register_device_arm();
bool connectToHome(IPAddress& address, uint16_t& port);
void sendHomePacket(char* data);
void sendHomePacketAndWait(char* data, String& incoming);
bool checkForHomeIncoming(String& incoming);