#include "WiFiNodeOTALibV5.h"

void startOTA(void);
void endOTA(void);
void progressOTA(unsigned int progress, unsigned int total);
void errorOTA(ota_error_t error);
// void wifiEventConnected(WiFiEvent_t event, WiFiEventInfo_t info);

// WEB SERVER
const uint serverPort = WEB_SERVER_PORT;
WiFiServer webServer(serverPort);
WiFiClient remoteClient;

// WiFiOTALib class constructor
WiFiOTALib::WiFiOTALib() {}

// Initializes the class
boolean WiFiOTALib::begin(void) {
    Serial.println("Begin WiFi ...");
    return true;
}

// Initializes the ESP32 WiFi module in STA mode
boolean WiFiOTALib::initSTA(void) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());

    Serial.println("Wait connecting to WiFi...");
    uint8_t _contWhile = 30;
    while ( (WiFi.status() != WL_CONNECTED) && (_contWhile) ) {
        Serial.print("WIFI_TRY #");
        Serial.println(_contWhile);
        WiFi.reconnect();
        _contWhile--;
        delay(1000);
    }

    boolean connectedModeSTA;
    if ( WiFi.status() == WL_CONNECTED ) {
        webServer.begin(serverPort);
        connectedModeSTA = true;
    } else {
        connectedModeSTA = false;
    }

    return connectedModeSTA;
}

// Returns the status of the STA connection
boolean WiFiOTALib::isConnectedSTA(void) {
    // if ( WiFi.status() == WL_CONNECTED ) {
    //     connectedModeSTA = true;
    // } else {
    //     connectedModeSTA = false;
    // }

    ( WiFi.status() == WL_CONNECTED ) ? connectedModeSTA = true: connectedModeSTA = false;
    return connectedModeSTA;
}

// 
uint8_t WiFiOTALib::listenForClientsWS(void)
{
    // Serial.print(__FUNCTION__);
    // Serial.println(__LINE__);
    if ( webServer.hasClient() ) {
        // If we are already connected to another computer, 
        // then reject the new connection. Otherwise accept
        // the connection. 
        if ( remoteClient.connected() ) {
            Serial.println("Connection rejected");
            webServer.available().stop();

            return 2;
        } else {
            Serial.println("Connection accepted");
            remoteClient = webServer.available();
            
            // isConnectedClientAP = true;
            isActiveServiceWS = true;

            return 1;
        }
        // return true;
    }
    return 0;
}

// Informs if the connection to the wifi network has been established
uint8_t WiFiOTALib::availableClientWS(void) {
    uint8_t _result = remoteClient.connected();
    if ( _result > 0) {
        // isConnectedClientAP = true;
        isActiveServiceWS = true;
    } else {
        // isConnectedClientAP = false;
        isActiveServiceWS = false;
    }
    return _result;
}

// Checks whether the client is connected and sent data
uint8_t WiFiOTALib::availableRecvDataClientWS(void) {
    if ( remoteClient.connected() )     {
        return remoteClient.available();
    } else {
        closeClientWS();
        return 0;
    }
}

// Closes the socket
void WiFiOTALib::closeClientWS(void) {
    remoteClient.stop();
}

// Read data arriving through the socket
String WiFiOTALib::readClientWS(void) {
    if ( remoteClient ) {
        return remoteClient.readStringUntil('\n');
    }
    return "";
}

// Write to the socket port
void WiFiOTALib::printClientWS(String strValue) {
    if ( isActiveServiceWS ) {
        remoteClient.println(strValue.c_str());
    }
}

// Informs if the connection to the wifi network has been established
boolean WiFiOTALib::activeOTA(void) {
    if ( connectedModeSTA ) {
        // The port defaults to 3232
        // ArduinoOTA.setPort(3232);

        // Defines the hostname (optional)
        String _hostname = HOSTNAME + macAddress;
        ArduinoOTA.setHostname(_hostname.c_str());

        // Sets the password (optional)
        //   ArduinoOTA.setPassword("amd1234");

        // It is possible to set a hash md5 encryption for the password using the "setPasswordHash" function
        // MD5 example for password "admin" = 21232f297a57a5a743894a0e4a801fc3
        // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

        // Defines what will be executed when ArduinoOTA starts
        ArduinoOTA.onStart(startOTA); // startOTA is a function created to simplify the code

        // Defines what will be executed when the ArduinoOTA is finished
        ArduinoOTA.onEnd(endOTA); // endOTA is a function designed to simplify the code 

        // Defines what will be performed when ArduinoOTA is recording
        ArduinoOTA.onProgress(progressOTA); // progressOTA is a function created to simplify the code 

        // Defines what will be executed when ArduinoOTA encounters an error
        ArduinoOTA.onError(errorOTA);// errorOTA is a function created to simplify the code
  
        // Initializes ArduinoOTA
        ArduinoOTA.begin();

        // Send IP and Hostname to serial port
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        Serial.print("Hostname: ");
        Serial.println(_hostname);

        return true;
    } else {
        return false;
    }
}

void WiFiOTALib::setMACAddress(String mac) {
    macAddress = mac;
}

// ArduinoOTA upload stages display functions (start, progress, end and error)
void startOTA(void) {
    String type;
   
    // If the update is being saved to the external flash memory, then inform "flash"
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "flash";
    else  // If the update is done via the internal memory (file system), then inform "filesystem"
      type = "filesystem"; // U_SPIFFS

    // Displays message next to the recording type
    Serial.println("Start updating " + type);
}

// Send the outgoing message to serial
void endOTA(void) {
    Serial.println("\nEnd");
}

// Serialize progress in percentage
void progressOTA(unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100))); 
}

// Sends serial messages if an error occurs, specifically displays the type of error
void errorOTA(ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      
      if (error == OTA_AUTH_ERROR) 
        Serial.println("Auth Failed");
      else
      if (error == OTA_BEGIN_ERROR)
        Serial.println("Begin Failed");
      else 
      if (error == OTA_CONNECT_ERROR)
        Serial.println("Connect Failed");
      else
      if (error == OTA_RECEIVE_ERROR) 
        Serial.println("Receive Failed");
      else 
      if (error == OTA_END_ERROR)
        Serial.println("End Failed");
}