#include "WiFiOTALibV4.h"

void startOTA(void);
void endOTA(void);
void progressOTA(unsigned int progress, unsigned int total);
void errorOTA(ota_error_t error);
void wifiEventConnected(WiFiEvent_t event, WiFiEventInfo_t info);

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_SERVER);    // NTPClient timeClient(ntpUDP, NTP_SERVER, -10800, 60000);

// WEB SERVER
const uint serverPort = WEB_SERVER_PORT;
WiFiServer webServer(serverPort);
WiFiClient remoteClient;

long tsDelaySTA, tsDelayAP, currentTimeSTA, currentTimeAP;

// Construtor da classe WiFiOTALib
WiFiOTALib::WiFiOTALib() {}

//======================
// Funcoes Public
//======================

// Inicializa a classe
// Inputs: void
// Return: boolean - TRUE caso seja estabelicida uma conexao WiFi
boolean WiFiOTALib::begin(void)
{
    Serial.println("Begin WiFi ...");

    tsDelaySTA = STA_DELAY_MS;
    currentTimeSTA = -2*tsDelaySTA;
    tsDelayAP = AP_DELAY_MS;
    currentTimeAP = -2*tsDelayAP;

    wifi_apssid = "Node-" + getMacAddress();

    return true;
}

// 
// Inputs: void
// Return: uint8_t
uint8_t WiFiOTALib::listenForClientsWS(void)
{
    // Serial.print(__FUNCTION__);
    // Serial.println(__LINE__);
    if ( webServer.hasClient() )
    {
        // If we are already connected to another computer, 
        // then reject the new connection. Otherwise accept
        // the connection. 
        if ( remoteClient.connected() )
        {
            Serial.println("Connection rejected");
            webServer.available().stop();

            return 2;
        }
        else
        {
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

// Informa se a conexao com a rede wifi foi estabelicida
// Inputs: void
// Return: boolean - TRUE rede wifi conectada
uint8_t WiFiOTALib::availableClientWS(void)
{
    /*
    // Serial.print(__FUNCTION__);
    // Serial.println(__LINE__);
    if ( webServer.hasClient() )
    {
        // If we are already connected to another computer, 
        // then reject the new connection. Otherwise accept
        // the connection. 
        // Serial.print(__FUNCTION__);
        // Serial.println(__LINE__);
        if ( remoteClient.connected() )
        {
            Serial.println("Connection rejected");
            webServer.available().stop();

            return 2;
        }
        else
        {
            Serial.println("Connection accepted");
            remoteClient = webServer.available();

            return 1;
        }
        // return true;
    }
    else
    {
        Serial.print("remoteClient.connected(): ");
        Serial.println(remoteClient.connected());

        return remoteClient.connected();
    }
    
    // return 0;
    */

    // Serial.print("remoteClient.connected(): ");
    // Serial.println(remoteClient.connected());

    uint8_t _result = remoteClient.connected();
    if ( _result > 0)
    {
        isConnectedClientAP = true;
        return _result;
    }
    else
    {
        isConnectedClientAP = false;
        isActiveServiceWS = false;
        return _result;
    }
    

    // return remoteClient.connected();

}

// Verifica se o cliente esta conectado e enviou dados
// Inputs: void
// Return: int buffer de dados recebido do cliente
uint8_t WiFiOTALib::availableRecvDataClientWS(void)
{
    if ( remoteClient.connected() )
    {
        return remoteClient.available();
    }
    else
    {
        // isActiveServiceWS = false;
        closeClientWS();
        return 0;
    }
}

// Fecha o socket
// Inputs: void
// Return: void
void WiFiOTALib::closeClientWS(void)
{
    // Serial.println("closeWebSocket() #1");
    remoteClient.stop();
}

// Ler os dados chegando pelo socket
// Inputs: void
// Return: String com o valor recebido pelo socket
String WiFiOTALib::readClientWS(void)
{
    if ( remoteClient )
    {
        return remoteClient.readStringUntil('\n');
    }
    return "";
}

// Escreve na porta do socket
// Inputs: String com o valor a ser enviado pelo socket
// Return: void
void WiFiOTALib::printClientWS(String strValue)
{

    // Serial.print("isActiveServiceWS: ");
    // Serial.println(isActiveServiceWS);

    if ( isActiveServiceWS )
    {
        remoteClient.println(strValue.c_str());
    }

    // if ( remoteClient )
    // {
    //     remoteClient.println(strValue.c_str());
    // }
}

// Recupera o MAC do modulo
// Input: nenhum
// Return: String - string com parte do MAC (3 bytes) do modulo
String WiFiOTALib::getMacAddress(void) 
{
    char chipID_0[4];
    char chipID_1[4];
    char chipID_2[4];
    uint64_t chipID = ESP.getEfuseMac();
    // Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipID>>32));       // print High 2 bytes
    // Serial.printf("%08X\n",(uint32_t)chipID);                           // print Low 4 bytes.

    uint8_t id8 = (uint8_t)(chipID>>24);
    if (id8 > 9) sprintf(chipID_0, "%0X", id8);
    else sprintf(chipID_0, "0%0X", id8);

    uint16_t id16 = (uint8_t)(chipID>>32);
    if (id16 > 9) sprintf(chipID_1, "%0X", id16);
    else sprintf(chipID_1, "0%0X", id16);

    uint32_t id32 = (uint8_t)(chipID>>40);
    if (id32 > 9) sprintf(chipID_2, "%0X", id32);
    else sprintf(chipID_2, "0%0X", id32);

    return String(chipID_0) + String(chipID_1) + String(chipID_2);
}

// Inicializa o modulo WiFi do ESP32 no modo STA
// Inputs: void
// Return: boolean - TRUE caso seja estabelicida uma conexao WiFi
boolean WiFiOTALib::initSTA(void)
{
    // Serial.println(__LINE__);
    if ( (millis() - currentTimeSTA) > tsDelaySTA )
    {
        //define wifi como AP + station (estação)
        WiFi.mode(WIFI_STA);

        // Inicializa o objeto wifi passando o ssid e pass
        WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());

        // Verifica se a conexao wifi foi estabelicida, caso ela ainda nao tenha ocorrida
        // entra no loop e aguarda por 60 segundos a resposta do roteador ao pedido de
        // conexao.
        Serial.println("Wait connecting to WiFi...");
        uint8_t _contWhile = 30;
        while ( (WiFi.status() != WL_CONNECTED) && (_contWhile) )
        {
            Serial.print("WIFI_TRY #");
            Serial.println(_contWhile);
            WiFi.reconnect();
            _contWhile--;
            delay(1000);
        }

        webServer.begin(serverPort);

        currentTimeSTA = millis();
    }

    if ( WiFi.status() == WL_CONNECTED )
    {
        connectedModeSTA = true;
        return connectedModeSTA;
    }
    else
    {
        WiFi.mode(WIFI_OFF);
        connectedModeSTA = false;
        return connectedModeSTA;
    }
}

// retorna o status da conexao STA
// Inputs: void
// Return: boolean - TRUE se ativa
boolean WiFiOTALib::isConnectedSTA(void)
{

    if ( WiFi.status() == WL_CONNECTED )
    {
        connectedModeSTA = true;
    }
    else
    {
        connectedModeSTA = false;
    }

    return connectedModeSTA;
}

// Coloca o WiFi em off
// Inputs: void
// Return: void
void WiFiOTALib::disconnectSTA(void)
{
    webServer.stopAll();

    WiFi.mode(WIFI_OFF);
    connectedModeSTA = false;
}

// Informa se a conexao com a rede wifi foi estabelicida
// Inputs: void
// Return: boolean - TRUE rede wifi conectada
boolean WiFiOTALib::activeOTA(void)
{
    if ( connectedModeSTA ) {
        // A porta fica default como 3232
        // ArduinoOTA.setPort(3232);

        // Define o hostname (opcional)
        String _hostname = HOSTNAME + getMacAddress();
        ArduinoOTA.setHostname(_hostname.c_str());

        // Define a senha (opcional)
        //   ArduinoOTA.setPassword("amd1234");

        // É possível definir uma criptografia hash md5 para a senha usando a função "setPasswordHash"
        // Exemplo de MD5 para senha "admin" = 21232f297a57a5a743894a0e4a801fc3
        // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

        //define o que será executado quando o ArduinoOTA iniciar
        ArduinoOTA.onStart(startOTA); //startOTA é uma função criada para simplificar o código 

        //define o que será executado quando o ArduinoOTA terminar
        ArduinoOTA.onEnd(endOTA); //endOTA é uma função criada para simplificar o código 

        //define o que será executado quando o ArduinoOTA estiver gravando
        ArduinoOTA.onProgress(progressOTA); //progressOTA é uma função criada para simplificar o código 

        //define o que será executado quando o ArduinoOTA encontrar um erro
        ArduinoOTA.onError(errorOTA);//errorOTA é uma função criada para simplificar o código 
  
        //inicializa ArduinoOTA
        ArduinoOTA.begin();

        //exibe pronto e o ip utilizado pelo ESP
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        Serial.print("Hostname: ");
        Serial.println(_hostname);

        return true;
    }
    else
    {
        return false;
    }
}

// Inicializa o servico NTP
// Inputs: void
// Return: boolean - TRUE se o cliente NTP foi inicilizado com sucesso
boolean WiFiOTALib::bootServiceNTP(void)
{
    if ( WiFi.status() == WL_CONNECTED )
    {
        timeClient.begin();

        // Set offset time in seconds to adjust for your timezone, for example:
        // GMT 1 = 3600
        // GMT 0 = 0
        // GMT -1 = -3600
        // GMT -3 = -10800      (Recife, Brasil)
        timeClient.setTimeOffset( timeOffsetNTP );

        timeClient.setUpdateInterval( updateIntervalNTP );

        timeClient.forceUpdate();

        return true;
        // connectedNTP = true;
        // return connectedNTP;
    }

    return false;
    // connectedNTP = false;
    // return connectedNTP;
}

// Inicializa o servico NTP
// Inputs: void
// Return: boolean - TRUE se o cliente NTP foi inicilizado com sucesso
long WiFiOTALib::getUnixTimeNTP(void)
{
    return timeClient.getEpochTime();
}

// Inicializa o modulo WiFi do ESP32 no modo AP
// Inputs: void
// Return: boolean - TRUE caso seja estabelicida uma conexao WiFi
boolean WiFiOTALib::initAP(void)
{
    // Serial.println(__LINE__);
    if ( (millis() - currentTimeAP) > tsDelayAP )
    {
        // Serial.println(__LINE__);

        //define wifi como AP + station (estação)
        WiFi.mode(WIFI_AP);

        // Inicializa o objeto wifi passando o ssid e pass
        WiFi.softAP(wifi_apssid.c_str(), wifi_appass.c_str());

        Serial.print("WiFi.softAPgetStationNum()");
        Serial.println(WiFi.softAPgetStationNum());

        // Verifica se a conexao wifi foi estabelicida, caso ela ainda nao tenha ocorrida
        // entra no loop e aguarda por 60 segundos a resposta do roteador ao pedido de
        // conexao.
        Serial.println("Wait connecting on AP...");
        uint8_t _contWhile = 30;
        while ( (WiFi.softAPgetStationNum() ==  0) && (_contWhile) )
        {
            // Serial.println(__LINE__);
            Serial.print("AP_TRY #");
            Serial.println(_contWhile);
            // WiFi.reconnect();
            _contWhile--;
            delay(1000);
        }

        webServer.begin(serverPort);

        currentTimeAP = millis();
    }

    if ( WiFi.softAPgetStationNum() > 0 )
    {
        // Serial.println(__LINE__);
        connectedModeAP = true;
        return connectedModeAP;
    }
    else
    {
        // Serial.println(__LINE__);
        WiFi.mode(WIFI_OFF);
        connectedModeAP = false;
        return connectedModeAP;
    }
}

// retorna o status da conexao AP
// Inputs: void
// Return: boolean - TRUE se ativa
boolean WiFiOTALib::isConnectedAP(void)
{
    if ( (WiFi.softAPgetStationNum() >  0) )
    {
        connectedModeAP = true;
    }
    else
    {
        connectedModeAP = false;
    }
    
    return connectedModeAP;
}

// Coloca o WiFi em off
// Inputs: void
// Return: void
void WiFiOTALib::disconnectAP(void)
{
    webServer.stopAll();
    
    WiFi.mode(WIFI_OFF);
    connectedModeAP = false;
}

// Funções de exibição dos estágios de upload (start, progress, end e error) do ArduinoOTA
// Inputs: void
// Return: void
// void ESP32NodeLib::startOTA(void) {
void startOTA(void) 
{
    String type;
   
    //caso a atualização esteja sendo gravada na memória flash externa, então informa "flash"
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "flash";
    else  //caso a atualização seja feita pela memória interna (file system), então informa "filesystem"
      type = "filesystem"; // U_SPIFFS

    //exibe mensagem junto ao tipo de gravação
    Serial.println("Start updating " + type);
}

// Enviar para serial a mensagem de saida
// Inputs: void
// Return: void
// void ESP32NodeLib::endOTA(void) {
void endOTA(void)
{
    Serial.println("\nEnd");
}

// Envia para serial a progresso em porcentagem
// Inputs: int, int valor do progresso e total
// Return: void
// void ESP32NodeLib::progressOTA(unsigned int progress, unsigned int total) {
void progressOTA(unsigned int progress, unsigned int total) 
{
    Serial.printf("Progress: %u%%\r", (progress / (total / 100))); 
}

// Envia para serial mensagens caso aconteça algum erro, exibe especificamente o tipo do erro
// Inputs: ota_error_t gerado na lib ArduinoOTA
// Return: void
// void ESP32NodeLib::errorOTA(ota_error_t error) {
void errorOTA(ota_error_t error) 
{
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