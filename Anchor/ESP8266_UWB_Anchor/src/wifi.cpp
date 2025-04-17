#include "wifi_handler.hpp"
#include "config.hpp"




const void WifiHandler::begin()
{
    Serial.print("Connecting to WiFi ");
    WiFi.begin(SSID, PWD);

    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < TIMEOUT)
    {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("");
        Serial.print("WiFi connected - local IP address: ");
        Serial.println(WiFi.localIP());
        connectWS();
    } else
    {
        Serial.println("");
        Serial.println("Failed to connect to WiFi");
    }
}

const void WifiHandler::connectWS()
{
    // WebSocket verbinden
    String url = "ws://" + String(SERVER_IP) + ":" + String(SERVER_PORT) + "/ws";
    if (client.connect(url)) {
        Serial.println("WebSocket connected!");
    } else {
        Serial.println("WebSocket Connection failed!");
    }
}

const void WifiHandler::connectWifi()
{
    WiFi.begin(SSID, PWD);
}
const void WifiHandler::loop()
{
    static unsigned long last_try = 0;
    if(AUTOMATIC_RECONNECT)
    {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi lost. Attempting reconnection...");
            connectWifi();
            
        } else{
            if(!client.available() && millis() - last_try >= RETRY_AFTER)
            {
                last_try = millis();
                connectWS();
            }
        }
    }
}

bool WifiHandler::logData(uint32_t deviceID, const char* message)
{
    bool success = false;
    if (WiFi.status() == WL_CONNECTED) {
        //String jsonString = "{\"esp_id\":\"ESP8266-1\",\"message\":" + String(3) + "}";
        String jsonString = "{\"esp_id\":\"" + String(deviceID) + "\",\"message\":\"" + String(message) + "\"}";
        success = client.send(jsonString);
        if(success)
        {
            //Serial.println("Sent: " + jsonString);
             // Nachrichten empfangen
             //TODO bad for interrupt, maybe do somewhere else!
            /*if (client.available()) {
                websockets::WebsocketsMessage message = client.readBlocking();
                Serial.println("Server: " + message.data());
            }*/
        }

    }
    return success;
}
WifiHandler::WifiHandler()
{

}
