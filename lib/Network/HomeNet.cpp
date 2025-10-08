#include "HomeNet.h"

WiFiClient client;

void register_device_arm()
{
    if (!client.connected())
    {
        Serial.println("Can't register to home, client not connected");
        return;
    }
    Serial.println("Registering device");
    client.print("REGISTER ");
    client.print(ESP.getEfuseMac());
    client.println(" ARM");

    bool responseReceived = false;
    while (!responseReceived) {
        if (client.available() > 0) {
            String response = client.readStringUntil('\n');
            int start = 0; 
            int end = response.indexOf(' ');
            if (end != -1) {
                String startToken = response.substring(start,end);
                if (startToken == "REG")
                {
                    responseReceived = true;
                    Serial.print("Registration response: ");
                    Serial.println(response);
                }
                else {
                    Serial.println("NREG response received: ");
                    Serial.println(response);
                }
            }
            else {
                Serial.println("NREG response received: ");
                Serial.println(response);
            }
        }
        delay(10);        
    }
    
}

bool connectToHome(IPAddress& address, uint16_t& port) {
    return client.connect(address, port);
}

void sendHomePacket(char* data) {
    if (!client.connected())
    {
        Serial.println("Home client not connected, aborting");
        return;
    }
    client.println(data);
}

void sendHomePacketAndWait(char* data, String& incoming) {
    if (!client.connected())
    {
        Serial.println("Home client not connected, aborting");
        return;
    }
    client.println(data);
    while (!client.available())
    {
        delay(10);
    }
    incoming = client.readStringUntil('\n');
}

bool checkForHomeIncoming(String& incoming) {
    if (!client.connected())
    {
        return false;
    }
    if (client.available() > 0)
    {
        incoming = client.readStringUntil('\n');
        return true;
    }
    return false;
}