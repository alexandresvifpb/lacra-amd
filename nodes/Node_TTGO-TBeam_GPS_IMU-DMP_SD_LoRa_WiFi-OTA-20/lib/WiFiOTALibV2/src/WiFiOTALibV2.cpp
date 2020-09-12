#include "WiFiOTALibV2.h"

void startOTA(void);
void endOTA(void);
void progressOTA(unsigned int progress, unsigned int total);
void errorOTA(ota_error_t error);

// Define NTP Client to get time
WiFiUDP ntpUDP;
// NTPClient timeClient(ntpUDP, NTP_SERVER, NTP_UTC_OFFSET * NTP_TIME_ADJUSTMENT, NTP_UPDATE_INTERVAL);    // NTPClient timeClient(ntpUDP, NTP_SERVER, -10800, 60000);
// NTPClient timeClient(ntpUDP);
NTPClient timeClient(ntpUDP, NTP_SERVER);    // NTPClient timeClient(ntpUDP, NTP_SERVER, -10800, 60000);

// WEB SERVER
const uint serverPort = WEB_SERVER_PORT;
WiFiServer webServer(serverPort);
WiFiClient remoteClient;

// Construtor da classe WiFiOTALib
WiFiOTALib::WiFiOTALib() {}

//======================
// Funcoes Public
//======================

// Inicializa o modulo WiFi do ESP32 
// Inputs: void
// Return: boolean - TRUE caso seja estabelicida uma conexao WiFi
boolean WiFiOTALib::begin(void)
{
    // bool result = false;

    Serial.println("Begin WiFi ...");

    //define wifi como station (estação)
    WiFi.mode(WIFI_STA);

    // Inicializa o objeto wifi passando o ssid e pass
    // WiFi.begin(WIFI_SSID, WIFI_PASS);
    WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());

    // Verifica se a conexao wifi foi estabelicida, caso ela ainda nao tenha ocorrida
    // entra no loop e aguarda por 60 segundos a resposta do roteador ao pedido de
    // conexao.
    Serial.println("Connecting to WiFi:");
    uint8_t _contWhile = 120;
    while ( (WiFi.status() != WL_CONNECTED) && (_contWhile) )
    {
        Serial.print(".");
        WiFi.reconnect();
        delay(1000);
    }

    if ( WL_CONNECTED )
    {
        Serial.println("Connected to the WiFi network");
        wifiConnected = true;
        // result = true;
    }
    else
    {
        Serial.println("Connection with WiFi network FAIL!!!");
        wifiConnected = false;
        // result = false;
    }

    if ( wifiConnected )
    {
        webServer.begin(serverPort);
    }

    return wifiConnected;
}

// Reinicializa o modulo WiFi do ESP32 
// Inputs: void
// Return: boolean - TRUE caso a comunicacao com o modulo GPS seja bem sucedida
boolean WiFiOTALib::reconnectWiFi(void)
{
    Serial.println("Disconnect WiFi ...");

    WiFi.disconnect();

    return begin();
}

// Informa se a conexao com a rede wifi foi estabelicida
// Inputs: void
// Return: boolean - TRUE rede wifi conectada
boolean WiFiOTALib::isConnectedWiFi(void)
{
    if ( WiFi.status() == WL_CONNECTED )
    {
        return true;
    }
    else
    {
        return false;
    }
    
}

// Seta o SSID da rede WiFi
// Inputs: String SSID da rede wifi
// Return: boolean - TRUE caso a comunicacao com o modulo GPS seja bem sucedida
void WiFiOTALib::setSSID(String ssid)
{
    wifi_ssid = ssid;
}

// Seta o PASS da rede WiFi
// Inputs: String PASS da rede wifi
// Return: boolean - TRUE caso a comunicacao com o modulo GPS seja bem sucedida
void WiFiOTALib::setPASS(String pass)
{
    wifi_pass = pass;
}

// Recupera o MAC do modulo
// Input: nenhum
// Return: String - string com parte do MAC (3 bytes) do modulo
String WiFiOTALib::getMacAddress(void) {
    char chipID_0[4];
    char chipID_1[4];
    char chipID_2[4];
    uint64_t chipID = ESP.getEfuseMac();
    Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipID>>32));       // print High 2 bytes
    Serial.printf("%08X\n",(uint32_t)chipID);                           // print Low 4 bytes.

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

// Informa se a conexao com a rede wifi foi estabelicida
// Inputs: void
// Return: boolean - TRUE rede wifi conectada
boolean WiFiOTALib::activeOTA(void)
{
    if ( wifiConnected ) {
        // A porta fica default como 3232
        // ArduinoOTA.setPort(3232);

        // Define o hostname (opcional)
        hostname = HOSTNAME + getMacAddress();
        ArduinoOTA.setHostname(hostname.c_str());

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
        Serial.println(hostname);

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

        ntpConnected = true;
        return ntpConnected;
    }
    ntpConnected = false;
    return ntpConnected;
}

// Inicializa o servico NTP
// Inputs: void
// Return: boolean - TRUE se o cliente NTP foi inicilizado com sucesso
long WiFiOTALib::getUnixTimeNTP(void)
{
    // if ( ntpConnected )
    // {
    //     Serial.println(timeClient.getEpochTime());
    // }
    return timeClient.getEpochTime();
}

// Funções de exibição dos estágios de upload (start, progress, end e error) do ArduinoOTA
// Inputs: void
// Return: void
uint8_t WiFiOTALib::listenForClientsSocket(void)
{
    if ( webServer.hasClient() )
    {
        // If we are already connected to another computer, 
        // then reject the new connection. Otherwise accept
        // the connection. 
        if ( remoteClient.connected() )
        {
            Serial.println("WIFI_TASK: Connection rejected");
            webServer.available().stop();

            return 2;
        }
        else
        {
            Serial.println("WIFI_TASK: Connection accepted");
            remoteClient = webServer.available();

            return 1;
        }
        // return true;
    }
    return 0;
}

// Verifica se o cliente esta conectado
// Inputs: void
// Return: boolean TRUE se o cliente estiver conectado
boolean WiFiOTALib::availableConnection(void)
{
    return remoteClient.connected();
}

// Escreve na porta do socket
// Inputs: String com o valor a ser enviado pelo socket
// Return: void
void WiFiOTALib::printWebSocket(String strValue)
{
    if ( remoteClient )
    {
        remoteClient.println(strValue.c_str());
    }
}

// Ler os dados chegando pelo socket
// Inputs: void
// Return: String com o valor recebido pelo socket
String WiFiOTALib::readWebSocket(void)
{
    if ( remoteClient )
    {
        return remoteClient.readStringUntil('\n');
    }
    return "";
}

// Fecha o socket
// Inputs: void
// Return: void
void WiFiOTALib::closeWebSocket(void)
{
    // Serial.println("closeWebSocket() #1");
    remoteClient.stop();
}

// Verifica se o cliente esta conectado e enviou dados
// Inputs: void
// Return: int buffer de dados recebido do cliente
int WiFiOTALib::availableRecvData(void)
{
    if ( remoteClient.connected() )
    {
        return remoteClient.available();
    }
    else
    {
        closeWebSocket();
        return 0;
    }
}

// Funções de exibição dos estágios de upload (start, progress, end e error) do ArduinoOTA
// Inputs: void
// Return: void
// void ESP32NodeLib::startOTA(void) {
void startOTA(void) {
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
void endOTA(void) {
    Serial.println("\nEnd");
}

// Envia para serial a progresso em porcentagem
// Inputs: int, int valor do progresso e total
// Return: void
// void ESP32NodeLib::progressOTA(unsigned int progress, unsigned int total) {
void progressOTA(unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100))); 
}

// Envia para serial mensagens caso aconteça algum erro, exibe especificamente o tipo do erro
// Inputs: ota_error_t gerado na lib ArduinoOTA
// Return: void
// void ESP32NodeLib::errorOTA(ota_error_t error) {
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