//by Little_S@tan
//MQTT topics: "mirtek/Request_Status", "mirtek/action"
//#define MY_CC1101

#include <MQTT.h>
#include <IotWebConf.h>
#include <IotWebConfUsing.h> // This loads aliases for easier class names.
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include "CRC8.h"
#include "CRC.h"
#include <TimerMs.h>

typedef struct { uint8_t byt[3]; } uInt24;
const int tmr_mqtt_time=30*1000; //раз в 60 cек запрашиваем и отправляем информацию в mqtt
//#include <cc1101_debug_service.h>

//Настройки для CC1101 с форума (47 бит)
byte rfSettings[] = {
  0x0D,  // IOCFG2              GDO2 Output Pin Configuration
  0x2E,  // IOCFG1              GDO1 Output Pin Configuration
  0x06,  // IOCFG0              GDO0 Output Pin Configuration
  0x4F,  // FIFOTHR             RX FIFO and TX FIFO Thresholds
  0xD3,  // SYNC1               Sync Word, High Byte
  0x91,  // SYNC0               Sync Word, Low Byte
  0x3C,  // PKTLEN              Packet Length
  0x00,  // PKTCTRL1            Packet Automation Control
  0x41,  // PKTCTRL0            Packet Automation Control
  0x00,  // ADDR                Device Address
  0x16,  // CHANNR              Channel Number
  //0x0B,  // CHANNR              Channel Number
  0x0F,  // FSCTRL1             Frequency Synthesizer Control
  0x00,  // FSCTRL0             Frequency Synthesizer Control
  0x10,  // FREQ2               Frequency Control Word, High Byte
  0x8B,  // FREQ1               Frequency Control Word, Middle Byte
  0x55,  // FREQ0     54          Frequency Control Word, Low Byte
  0xD9,  // MDMCFG4             3:0 DRATE_E[3:0] 12 (0x0C) R/W The exponent of the user specified symbol rate
  0x84,  // MDMCFG3  83          7:0 DRATE_M[7:0] 34 (0x22) R/W The mantissa of the user specified symbol rate
  0x13,  // MDMCFG2             Modem Configuration
  0xD2,  // MDMCFG1             Modem Configuration
  0xAA,  // MDMCFG0             Modem Configuration
  0x31,  // DEVIATN             Modem Deviation Setting
  0x07,  // MCSM2               Main Radio Control State Machine Configuration
  0x0C,  // MCSM1               Main Radio Control State Machine Configuration
  0x08,  // MCSM0               Main Radio Control State Machine Configuration
  0x16,  // FOCCFG              Frequency Offset Compensation Configuration
  0x6C,  // BSCFG               Bit Synchronization Configuration
  0x03,  // AGCCTRL2            AGC Control
  0x40,  // AGCCTRL1            AGC Control
  0x91,  // AGCCTRL0            AGC Control
  0x87,  // WOREVT1             High Byte Event0 Timeout
  0x6B,  // WOREVT0             Low Byte Event0 Timeout
  0xF8,  // WORCTRL             Wake On Radio Control
  0x56,  // FREND1              Front End RX Configuration
  0x10,  // FREND0              Front End TX Configuration
  0xE9,  // FSCAL3              Frequency Synthesizer Calibration
  0x2A,  // FSCAL2              Frequency Synthesizer Calibration
  0x00,  // FSCAL1              Frequency Synthesizer Calibration
  0x1F,  // FSCAL0              Frequency Synthesizer Calibration
  0x41,  // RCCTRL1             RC Oscillator Configuration
  0x00,  // RCCTRL0             RC Oscillator Configuration
  0x59,  // FSTEST              Frequency Synthesizer Calibration Control
  0x59,  // PTEST               Production Test
  0x3F,  // AGCTEST             AGC Test
  0x81,  // TEST2               Various Test Settings
  0x35,  // TEST1               Various Test Settings
  //0x0B,  // TEST0               Various Test Settings
  0x09,  // TEST0               Various Test Settings
};

int gdo0 = 2; //for esp32! GDO0 on GPIO2.
//char MeterAdressValue[] = "10870"; //адрес счётчика

CRC8 crc;
int bytecount = 0; //указатель байтов в результирующем буфере
char s[1]; //Промежуточная переменная для вывода в Serial

TimerMs tmr(2500, 0, 0); //инициализируем таймер
TimerMs tmr_mqtt(tmr_mqtt_time, 0, 0); //инициализируем таймер, отправляющий значения в MQTT

// -- Initial name of the Thing. Used e.g. as SSID of the own Access Point.
const char thingName[] = "Mirtek";

// -- Initial password to connect to the Thing, when it creates an own Access Point.
const char wifiInitialApPassword[] = "12345678";

#define STRING_LEN 128
#define NUMBER_LEN 6

// -- Configuration specific key. The value should be modified if config structure was changed.
#define CONFIG_VERSION "mirtek_gw_v1"

// -- Status indicator pin.
//      First it will light up (kept LOW), on Wifi connection it will blink,
//      when connected to the Wifi it will turn off (kept HIGH).
#define STATUS_PIN 16

// -- Method declarations.
void handleRoot();
//void mqttMessageReceived(String& topic, String& payload);
bool connectMqtt();
bool connectMqttOptions();
// -- Callback methods.
void wifiConnected();
void configSaved();
bool formValidator(iotwebconf::WebRequestWrapper* webRequestWrapper);

DNSServer dnsServer;
WebServer server(80);
WiFiClient net;
MQTTClient mqttClient;

char mqttServerValue[STRING_LEN];
char mqttUserNameValue[STRING_LEN];
char mqttUserPasswordValue[STRING_LEN];
char MeterAdressValue[NUMBER_LEN];

float sum;
float t1,t2,t3; 
float cons;
char time_m[20];

IotWebConf iotWebConf(thingName, &dnsServer, &server, wifiInitialApPassword, CONFIG_VERSION);
// -- You can also use namespace formats e.g.: iotwebconf::ParameterGroup
IotWebConfParameterGroup mqttGroup = IotWebConfParameterGroup("mqtt", "MQTT configuration");
IotWebConfTextParameter mqttServerParam = IotWebConfTextParameter("MQTT server", "mqttServer", mqttServerValue, STRING_LEN);
IotWebConfTextParameter mqttUserNameParam = IotWebConfTextParameter("MQTT user", "mqttUser", mqttUserNameValue, STRING_LEN);
IotWebConfPasswordParameter mqttUserPasswordParam = IotWebConfPasswordParameter("MQTT password", "mqttPass", mqttUserPasswordValue, STRING_LEN);

IotWebConfParameterGroup group1 = IotWebConfParameterGroup("group1", "Настройки");
IotWebConfNumberParameter MeterAdress = IotWebConfNumberParameter("Адрес счётчика", "MeterAdress", MeterAdressValue, NUMBER_LEN, "", "1..65534", "min='1' max='65534' step='1'");

bool needMqttConnect = false;
bool needReset = false;
int pinState = HIGH;
unsigned long lastReport = 0;
unsigned long lastMqttConnectionAttempt = 0;
byte myCRC = 0;

// Send Pack type (0x20/21), Meter Addr, Code, Subcode
void SendPack(byte type, char *addr ,byte code, byte subcode) {
//    byte tr1[] = {0x0F, 0x73, 0x55, 0x20, 0x00, 0, 0, 0x09, 0xff, 0, 0, 0, 0, 0, 0, 0x55};
      byte tr[] =  {0x10, 0x73, 0x55, 0x21, 0x00, 0, 0, 0x09, 0xff, 0, 0, 0, 0, 0, 0, 0, 0x55};
    Serial.print("RequestPacket: "); Serial.print(" Type: "); Serial.print(type, HEX); Serial.print(" Code: "); Serial.print(code, HEX); Serial.print(" SubCode: "); Serial.println(subcode, HEX);
	if ((type!=0x21) && (type!=0x20)) { Serial.println("Type not correct "); return;}

    tr[0] = type==0x21?0x10:0x0F; //длина пакета 16/17 байт
    tr[1] = 0x73; // const: 
    tr[2] = 0x55; // const: начало payload
    tr[3] = type; // 0x21const: тип запроса
    tr[4] = 0x00; // const: 0 
    tr[5] = (atoi(addr)) & 0xff; //младший байт адреса счётчика
    tr[6] = ((atoi(addr)) >> 8) & 0xff; //старший байт адреса счётчика
    tr[7] = 0x09;  // interface CC1101
    tr[8] = 0xff;  // const: FF 
    tr[9] = code;  // request code
    tr[10] = 0x00; //PIN
    tr[11] = 0x00; //PIN
    tr[12] = 0x00; //PIN
    tr[13] = 0x00; //PIN
    if (type==0x21) tr[14] = subcode;    
    crc.restart();
    crc.setPolynome(0xA9);
    for (int i = 3; i < (tr[0] - 1); i++)
    {
        crc.add(tr[i]);
    }
    tr[(tr[0] - 1)] = crc.getCRC(); //CRC
    tr[tr[0]] = 0x55; //конец payload

    //Печатаем общий пакет
    char s[60]; s[0]=0;
    for (int i = 0; i < 17; i++) sprintf(s+strlen(s), " %02X", tr[i]);
    Serial.println(s);

//отправка пакета
    ELECHOUSE_cc1101.SpiStrobe(0x33);  //Calibrate frequency synthesizer and turn it off
    delay(10);
    ELECHOUSE_cc1101.SpiStrobe(0x3B);  // Flush the TX FIFO buffer
    ELECHOUSE_cc1101.SpiStrobe(0x36);  // Exit RX / TX, turn off frequency synthesizer and exit
    ELECHOUSE_cc1101.SpiWriteReg(0x3e, 0xC4); //выставляем мощность 10dB
    ELECHOUSE_cc1101.SendData(tr, tr[0]+1); //отправляем пакет
    ELECHOUSE_cc1101.SpiStrobe(0x3A);  // Flush the RX FIFO buffer
    ELECHOUSE_cc1101.SpiStrobe(0x34);  // Enable RX
}

//функция приёма пакета с возвратом валидности, Buff pointer, num of Packs
//кол-во подпакетов в ответе - зависит от типа запроса (3 для запросов 1-4, 4 для запросов 5,6)
bool RecPack(byte resB[], int Packs) {
    byte buffer[61] = { 0 }; //буффер пакетов, принятых из трансивера 
    tmr.start();
    int PackCount = 0; //счётчик принятых из эфира пакетов
    bytecount = 0;     //указатель байтов в результирующем буфере
    while (!tmr.tick() && PackCount != Packs && bytecount <66) {
        if (ELECHOUSE_cc1101.CheckReceiveFlag()) {
            PackCount++;
            int len = ELECHOUSE_cc1101.ReceiveData(buffer);

            for (int i = 1; i < len; i++) {resB[bytecount++] = buffer[i];};//bytecount++;

            ELECHOUSE_cc1101.SpiStrobe(0x36);  // Exit RX / TX, turn off frequency synthesizer and exit
            ELECHOUSE_cc1101.SpiStrobe(0x3A);  // Flush the RX FIFO buffer
            ELECHOUSE_cc1101.SpiStrobe(0x3B);  // Flush the TX FIFO buffer
            delay(1);//delay(0);
            ELECHOUSE_cc1101.SpiStrobe(0x34);  // Enable RX
        }
    }
    Serial.print("Packets received: ");
    Serial.println(PackCount);
    //Печатаем общий пакет
    char s[200]; s[0]=0;
    for (int i = 0; (i < bytecount) && (i < 66); i++) sprintf(s+strlen(s), " %02X", resB[i]);
    Serial.println(s);

    Serial.print("Packet length: "); Serial.print(bytecount);
    // Test CRC ------------------------------
    crc.reset();
    crc.setPolynome(0xA9);
    for (int i = 2; i < (bytecount - 2); i++) crc.add(resB[i]);

    myCRC = crc.getCRC();
    Serial.print("  R_CRC: "); Serial.print(resB[bytecount - 2], HEX); Serial.print(" C_CRC: "); Serial.println(myCRC, HEX);
    return ((myCRC==uint8_t(resB[bytecount - 2]))?true:false);
    //-----------------------------------------
}

void Func_5(byte typ) {
    union {
    struct {
        byte beg[23];
        u32_t Sum,T1,T2,T3;
    } D;
    byte    b[62];
    } PQS;
    PQS.b[0]=0;
    Serial.println(""); Serial.println("Func 5");
	    SendPack(0x21, MeterAdressValue, 0x05,typ); //отправляем пакет
    if (RecPack(&PQS.b[1], 4) && (PQS.b[45]==0x55)) //принимаем и склеиваем пакет; int32 alignment !!!!!!!!  
    { 
    Serial.println("Func 5 rec ok");
    sum = u32_t(PQS.D.Sum) / 100.0;
    t1  = u32_t(PQS.D.T1) / 100.0;
    t2  = u32_t(PQS.D.T2) / 100.0;
    t3  = u32_t(PQS.D.T3) / 100.0;

    Serial.print("SUM:  ");	Serial.println(sum);    
    Serial.print("T1:  "); Serial.println(t1);    
    Serial.print("T2:  "); Serial.println(t2);    
    Serial.print("T3:  "); Serial.println(t3);
  }else{
    Serial.println("Func 5 rec ERROR");
  }	
}

//Функция формирования 1-го (начального) пакета (date, time)
void Func_1C() {
    //73 55 7 0 9 FF E9 99 1C A8 3 5B 0 29 4 0 1 1F A 16 79 55 
    //{"Time":"2022-11-01T16:06:58"  
  byte    resBuff[62];
    Serial.println(""); Serial.println("Func 1C");
    SendPack(0x20, MeterAdressValue, 0x1C, 0); 
    if (RecPack(resBuff, 3)) //принимаем и склеиваем пакет
    {
        Serial.print("  DATE:  ");
        sprintf(time_m, "/0");
        sprintf(time_m,"20%2i-%02i-%02i",resBuff[19], resBuff[18], resBuff[17]);
        sprintf(time_m+ strlen(time_m), "T%02i:%02i:%02i",resBuff[15], resBuff[14],resBuff[13]);
        Serial.println(time_m);
    }else{
        Serial.println("Func 1C rec ERROR");
    }
}

void domoticzPublish(unsigned idx, float t1, float t2, float cons ){
// domoticz
// https://www.domoticz.com/wiki/Domoticz_API/JSON_URL%27s#Electricity_P1_smart_meter
// idx=IDX&nvalue=0&svalue=USAGE1;USAGE2;RETURN1;RETURN2;CONS;PROD
//{
//  "idx": 281,
//  "nvalue": 0,
//  "svalue": "0.0; 554866.4; 0.0;0.0; 425;0",
//  "Battery": 100,
//  "RSSI": 7
//}
  char buffer[128];
  sprintf(buffer, "{\"idx\": %d, \"nvalue\": 0, \"svalue\":\"%d;%d; 0.0;0.0; %d;0\"}", idx, unsigned(t1), unsigned(t2), unsigned(cons) );
  Serial.print("Domotics: "); Serial.println(buffer);
  mqttClient.publish("domoticz/in", buffer);
}

void Func7_0m(){
char buff[200];
union 
{
   struct {
    byte beg[23];
    u16_t F,Cos,V1,V2,V3;
    uInt24 I1,I2,I3;
   } D;
   byte    b[62];
} PQS;
PQS.b[0]=0;
Serial.println(""); Serial.println("Func 7_0");
SendPack(0x21,MeterAdressValue, 0x2b, 0x0); 
if (RecPack(PQS.b, 4)) //принимаем и склеиваем пакет
{
float fr=float(PQS.D.F)/100; //Freq
float Cos=float(PQS.D.Cos)/100;//Cos
float V1=float(PQS.D.V1)/100;//V1
float V2=float(PQS.D.V2)/100; //V2
float V3=float(PQS.D.V3)/100; //V3
float I1=float(PQS.D.I1.byt[0] | (PQS.D.I1.byt[1]<< 8) | ( PQS.D.I1.byt[2]<< 16)) / 1000; //I1
float I2=float(PQS.D.I2.byt[0] | (PQS.D.I2.byt[1]<< 8) | ( PQS.D.I2.byt[2]<< 16)) / 1000; //I2
float I3=float(PQS.D.I3.byt[0] | (PQS.D.I3.byt[1]<< 8) | ( PQS.D.I3.byt[2]<< 16)) / 1000; //I3

sprintf(buff, "/0");
sprintf(buff, "{\"Time\":\"%s\",", time_m); //,
sprintf(buff + strlen(buff), "\"U1\":%.1f,\"I1\":%.2f,", V1, I1); 
sprintf(buff + strlen(buff), "\"U2\":%.1f,\"I2\":%.2f,", V2, I2); 
sprintf(buff + strlen(buff), "\"U3\":%.1f,\"I3\":%.2f,", V3, I3); 
sprintf(buff + strlen(buff), "\"Freq\":%.1f,\"Cos\":%.2f,", fr, Cos); 
sprintf(buff + strlen(buff), "\"T1\":%.1f,\"T2\":%.1f,\"T3\":%.1f", t1, t2, t3); 
sprintf(buff + strlen(buff), "}"); 

  Serial.print("HA: ");Serial.print(strlen(buff)); Serial.println(buff);
  mqttClient.publish("tele/mirtek/SENSOR", buff);
}
else Serial.println("Func 7_0 rec ERROR");
}

void Func7_10m(){
char buff[200];
union 
{
   struct {
    byte beg[23];
    u16_t P1,P2,P3;
    u16_t Q1,Q2,Q3;
    u16_t S1,S2,S3;
    byte T;    
   } D;
   byte    b[62];
} PQS;
PQS.b[0]=0;
Serial.println(""); Serial.println("Func 7_10");
SendPack(0x21,MeterAdressValue, 0x2b, 0x10); 
if (RecPack(PQS.b, 4)) //принимаем и склеиваем пакет
{
int16_t Q1=PQS.D.Q1<25000?PQS.D.Q1:0x8000-PQS.D.Q1;
int16_t Q2=PQS.D.Q2<25000?PQS.D.Q2:0x8000-PQS.D.Q2;
int16_t Q3=PQS.D.Q3<25000?PQS.D.Q3:0x8000-PQS.D.Q3;
sprintf(buff, "/0");
sprintf(buff, "{\"Temp\":\"%i\",", PQS.D.T); //,
sprintf(buff + strlen(buff), "\"P1\":%i,\"P2\":%i,\"P3\":%i,", PQS.D.P1,PQS.D.P2,PQS.D.P3); 
sprintf(buff + strlen(buff), "\"Q1\":%i,\"Q2\":%i,\"Q3\":%i,", Q1,Q2,Q3); 
sprintf(buff + strlen(buff), "\"S1\":%i,\"S2\":%i,\"S3\":%i", PQS.D.S1,PQS.D.S2,PQS.D.S3); 
sprintf(buff + strlen(buff), "}"); 

  Serial.print("P7_10: ");Serial.print(strlen(buff)); Serial.println(buff);
  mqttClient.publish("tele/mirtek/SENSOR2", buff);
}
else Serial.println("Func 7_10 rec ERROR");
}

void setup() {
    Serial.begin(115200);
    mqttGroup.addItem(&mqttServerParam);
    mqttGroup.addItem(&mqttUserNameParam);
    mqttGroup.addItem(&mqttUserPasswordParam);
    group1.addItem(&MeterAdress);
    iotWebConf.setStatusPin(STATUS_PIN);
    //iotWebConf.setConfigPin(CONFIG_PIN);
    iotWebConf.addParameterGroup(&mqttGroup);
    iotWebConf.addParameterGroup(&group1);
    iotWebConf.setConfigSavedCallback(&configSaved);
    iotWebConf.setFormValidator(&formValidator);
    iotWebConf.setWifiConnectionCallback(&wifiConnected);

    // -- Initializing the configuration.
    bool validConfig = iotWebConf.init();
    if (!validConfig)
    {
        mqttServerValue[0] = '\0';
        mqttUserNameValue[0] = '\0';
        mqttUserPasswordValue[0] = '\0';
    }

    // -- Set up required URL handlers on the web server.
    server.on("/", handleRoot);
    server.on("/config", [] { iotWebConf.handleConfig(); });
    server.onNotFound([]() { iotWebConf.handleNotFound(); });

    mqttClient.begin(mqttServerValue, net);
    //mqttClient.onMessage(mqttMessageReceived);

    Serial.println("Ready.");
        ELECHOUSE_cc1101.setGDO0(gdo0);
    if (ELECHOUSE_cc1101.getCC1101()) {     // Check the CC1101 Spi connection.
        Serial.println("CC1101 Connection OK");
    //Инициализируем cc1101
		ELECHOUSE_cc1101.SpiStrobe(0x30);  //reset
    
		ELECHOUSE_cc1101.SpiWriteBurstReg(0x00, rfSettings, 0x2F);
		ELECHOUSE_cc1101.SpiStrobe(0x33);  //Calibrate frequency synthesizer and turn it off
		delay(1);
		ELECHOUSE_cc1101.SpiStrobe(0x3A);  // Flush the RX FIFO buffer  CC1101_SFRX
		ELECHOUSE_cc1101.SpiStrobe(0x3B);  // Flush the TX FIFO buffer  CC1101_SFTX
		ELECHOUSE_cc1101.SpiStrobe(0x34);  // Enable RX                 CC1101_SRX
    }
    else {
        Serial.println("CC1101 Connection Error");
    }
    //Serial.println("Rx Mode");
    tmr_mqtt.start(); //старт таймера MQTT
}

void loop() {
    // -- doLoop should be called as frequently as possible.
    iotWebConf.doLoop();
    mqttClient.loop();
  
    if (needMqttConnect)
    {
        if (connectMqtt())
        {
            needMqttConnect = false;
        }
    }
    else if ((iotWebConf.getState() == iotwebconf::OnLine) && (!mqttClient.connected()))
    {
        Serial.println("MQTT reconnect");
        connectMqtt();
    }

    if (needReset)
    {
        Serial.println("Rebooting after 1 second.");
        iotWebConf.delay(1000);
        ESP.restart();
    }

  if ((iotWebConf.getState() == iotwebconf::OnLine) && (mqttClient.connected())){
    if (tmr_mqtt.tick()) // Запрос информации по таймеру и отправка в MQTT
      {
        Serial.println("--------------------->Request MIRTEK by timer");
	    Func_1C();
        Func_5(0);
        Func7_0m();
        Func7_10m();
        Func_5(0x4);
        SendPack(0x22,MeterAdressValue, 0x2b, 0x10); 
        //domoticzPublish(302, t1, t2, cons);
      }
    }
    //delay(2000);
}

void handleRoot()
{
    // -- Let IotWebConf test and handle captive portal requests.
    if (iotWebConf.handleCaptivePortal())
    {
        // -- Captive portal request were already served.
        return;
    }
    String s = "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/>";
    s += "<title>Mirtek-to-MQTT</title></head><body>Gateway";
    s += "<ul>";
    s += "<li>MQTT server: ";
    s += mqttServerValue;
    s += "</ul>";
    s += "<a href='config'>configu page</a>";
    s += "</body></html>\n";

    server.send(200, "text/html", s);
}

void wifiConnected()
{
    needMqttConnect = true;
}

void configSaved()
{
    Serial.println("Configuration was updated.");
    needReset = true;
}

bool formValidator(iotwebconf::WebRequestWrapper* webRequestWrapper)
{
    Serial.println("Validating form.");
    bool valid = true;

    int l = webRequestWrapper->arg(mqttServerParam.getId()).length();
    if (l < 3)
    {
        mqttServerParam.errorMessage = "Please provide at least 3 characters!";
        valid = false;
    }

    return valid;
}

bool connectMqtt() {
    unsigned long now = millis();
    if (1000 > now - lastMqttConnectionAttempt)
    {
        // Do not repeat within 1 sec.
        return false;
    }
    Serial.println("Connecting to MQTT server...");
    if (!connectMqttOptions()) {
        lastMqttConnectionAttempt = now;
        return false;
    }
    Serial.println("MQTT Connected!");

    mqttClient.subscribe("tele/mirtek");
    return true;
}

bool connectMqttOptions()
{
    bool result;
    if (mqttUserPasswordValue[0] != '\0')
    {
        result = mqttClient.connect(iotWebConf.getThingName(), mqttUserNameValue, mqttUserPasswordValue);
    }
    else if (mqttUserNameValue[0] != '\0')
    {
        result = mqttClient.connect(iotWebConf.getThingName(), mqttUserNameValue);
    }
    else
    {
        result = mqttClient.connect(thingName);//iotWebConf.getThingName()
    }
    return result;
}


//Функция формирования 2-го пакета
//    SendPack(0x20, MeterAdressValue, 0x10); 
//Функция формирования 3-го пакета
//    SendPack(0x20, MeterAdressValue, 0x28, 0x00); 
//Функция формирования 4-го пакета
//    SendPack(0x20, MeterAdressValue, 0x28, 0x01); 
//Функция формирования 5-го пакета
//    SendPack(0x21, MeterAdressValue, 0x05, 0x00); 
//Функция формирования 6-го пакета
//  SendPack(0x21, MeterAdressValue, 0x0a, 0x01); 
//Функция формирования 7-го пакета (Действующие значения напряжения, тока)
//  SendPack(0x21, MeterAdressValue, 0x2a, 0x01); 
//Функция формирования 8-го пакета (Aabs)
//  SendPack(0x21, MeterAdressValue, 0x05, 0x04); 
//Функция формирования 9 (параметры сети)
//  SendPack(0x21, MeterAdressValue, 0x2B, 0x00);

/*
void mqttMessageReceived(String& topic, String& payload)
{
    byte resBuff[61] = { 0 };
    //Очищаем выходной буфер
    for (int i = 0; i < 61; i++) {
    //    resBuff[i] = 0;
    }
    //----------------------
    bool Request_Status = 0;
    Serial.println(payload);
    switch (payload.toInt()) {
    case 1:
        Serial.println("1 reseived from MQTT");
//        RequestPacket_1();
        break;
    case 2:
        Serial.println("2 reseived from MQTT");
        RequestPacket_2();
        break;
    case 3:
        Serial.println("3 reseived from MQTT");
        RequestPacket_3();
        break;
    case 4:
        Serial.println("4 reseived from MQTT");
        RequestPacket_4();
        break;
    case 5:
        Serial.println("5 reseived from MQTT");
        RequestPacket_5();
        break;
    case 6:
        Serial.println("6 reseived from MQTT");
        RequestPacket_6();
        break;
    case 7:
        Serial.println("7 reseived from MQTT");
        RequestPacket_7();
        break;
    case 8:
        Serial.println("8 reseived from MQTT");
        RequestPacket_8();
        break;
    case 9:
        Serial.println("9 reseived from MQTT");
//        RequestPacket_9_pre();
//        delay(5000);
        RequestPacket_9();
        break;
}

    for (int k = 0; k < 3; k++) {  //Пробуем 5 раз сделать запросы и получить ответы
        packetSender(transmitt_byte); //отправляем пакет
        //packetReceiver(); //принимаем и склеиваем пакет
        if ((myCRC == resBuff[bytecount - 2]) & ((atoi(MeterAdressValue) & 0xff) == resBuff[6]) & (((atoi(MeterAdressValue) >> 8) & 0xff) == resBuff[7])) {
            //if ((myCRC == resultbuffer[bytecount - 2]) {
            Serial.println("CRC & adress ok");
            switch (payload.toInt()) {
            case 1:

                break;
            case 2:

                break;
            case 3:

                break;
            case 4:

                break;
            case 5:
                //packetParser_5_mqtt();
                break;
            case 6:

                break;
            case 7:
                //packetParser_7();
                break;
            case 9:
                //packetParser_7_mqtt();
                break;
            }
            Request_Status = 1;
            mqttClient.publish("mirtek/Request_Status", "Ok");
            break;
        }

    }
    if (!Request_Status == 1) {
        mqttClient.publish("mirtek/Request_Status", "Error");
    }
}
*/
