#include <WiFi.h>

bool startBeacon();
void waitForBeacon(IPAddress& outIp, uint16_t& outPort);
bool parseBeacon(const char* msg, IPAddress& outIp, uint16_t& outPort);