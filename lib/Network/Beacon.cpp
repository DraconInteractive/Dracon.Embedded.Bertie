#include "Beacon.h"

const char* beaconPrefix = "HOME-TCP";

WiFiUDP udp;
const int discoveryPort = 5001;

bool startBeacon() {
    if (udp.begin(discoveryPort))
    {
        Serial.printf("Listening for discovery beacon on :%d...\n", discoveryPort);
        return true;
    }
    else {
        Serial.printf("ERR: Failed to bind UDP at :%d...\n", discoveryPort);
        return false;
    }
}

// Blocking call
void waitForBeacon(IPAddress& outIp, uint16_t& outPort) {
    bool serverFound = false;
    while(!serverFound)
    {
        int packetSize = udp.parsePacket();
        if (packetSize > 0)
        {
            char buf[256];
            int len = udp.read(buf, sizeof(buf) - 1);
            if (len < 0) continue;
            buf[len] = '\0';

            if (parseBeacon(buf, outIp, outPort))
            {
                Serial.print("Discovered server at ");
                Serial.print(outIp);
                Serial.print(":");
                Serial.println(outPort);
                serverFound = true;
            }
            else {
                Serial.println("Received bad packet: ");
                Serial.println(buf);
            }
        }

        delay(50);
    }
}

bool parseBeacon(const char* msg, IPAddress& outIp, uint16_t& outPort)
{
    if (strncmp(msg, beaconPrefix, strlen(beaconPrefix)) != 0) return false;

    const char* p = msg + strlen(beaconPrefix);
    while (*p == ' ') ++p; // trim single space(s)

    const char* colon = strrchr(p, ':');
    if (!colon) return false;

    String ipStr   = String(p).substring(0, colon - p);
    String portStr = String(colon + 1);

    ipStr.trim();
    portStr.trim();

    IPAddress ip;
    if (!ip.fromString(ipStr)) return false;

    int port = portStr.toInt();
    if (port <= 0 || port > 65535) return false;

    outIp = ip;
    outPort = static_cast<uint16_t>(port);
    return true;
}

