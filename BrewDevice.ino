//    Erstellt:	2021
//    Author:	Innuendo

#include <OneWire.h>           // OneWire Bus Kommunikation
#include <DallasTemperature.h> // Vereinfachte Benutzung der DS18B20 Sensoren
#include <ESP8266WiFi.h>       // Generelle WiFi Funktionalität
#include <ESP8266WebServer.h>  // Unterstützung Webserver
#include <ESP8266HTTPUpdateServer.h>
#include <WiFiManager.h> // WiFiManager zur Einrichtung
#include <DNSServer.h>   // Benötigt für WiFiManager
#include "LittleFS.h"   
#include <ArduinoJson.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <WiFiClientSecure.h>
#include <WiFiClientSecureBearSSL.h>
#include <NTPClient.h>
#include "InnuTicker.h"
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <PubSubClient.h> // MQTT Kommunikation 2.7.0
#include <CertStoreBearSSL.h>
#include <InfluxDbClient.h>

extern "C"
{
#include "user_interface.h"
}

#ifdef DEBUG_ESP_PORT
#define DEBUG_MSG(...)                                                   \
    DEBUG_ESP_PORT.printf("%s ", timeClient.getFormattedTime().c_str()); \
    DEBUG_ESP_PORT.printf(__VA_ARGS__)
#else
#define DEBUG_MSG(...)
#endif

#if SerialDebug == true
#define DBG_PRINTF(...) DebugPort.printf(__VA_ARGS__)
#else
#define DBG_PRINTF(...)
#endif

// Version
#define Version "1.00"

// Definiere Pausen
#define PAUSE1SEC 1000
#define PAUSE2SEC 2000
#define PAUSEDS18 750

// OneWire
#define ONE_WIRE_BUS D3
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

// WiFi und MQTT
ESP8266WebServer server(80);
WiFiManager wifiManager;
WiFiClient espClient;
PubSubClient pubsubClient(espClient);
ESP8266HTTPUpdateServer httpUpdate;
MDNSResponder mdns;

// Induktion Signallaufzeiten
const int SIGNAL_HIGH = 5120;
const int SIGNAL_HIGH_TOL = 1500;
const int SIGNAL_LOW = 1280;
const int SIGNAL_LOW_TOL = 500;
const int SIGNAL_START = 25;
const int SIGNAL_START_TOL = 10;
const int SIGNAL_WAIT = 10;
const int SIGNAL_WAIT_TOL = 5;
#define DEF_DELAY_IND 120000 // Standard Nachlaufzeit nach dem Ausschalten Induktionskochfeld

/*  Binäre Signale für Induktionsplatte */
int CMD[6][33] = {
    {1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},  // Aus
    {1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0},  // P1
    {1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0},  // P2
    {1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0},  // P3
    {1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},  // P4
    {1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0}}; // P5
unsigned char PWR_STEPS[] = {0, 20, 40, 60, 80, 100};                                                     // Prozentuale Abstufung zwischen den Stufen

bool pins_used[17];
const unsigned char numberOfPins = 9;
const unsigned char pins[numberOfPins] = {D0, D1, D2, D3, D4, D5, D6, D7, D8};
const String pin_names[numberOfPins] = {"D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7", "D8"};

// Variablen

// #define WindowSize 4800
// #define minWindow 150
// #define AUTOMATIC 1
// #define MANUAL 0
// #define DIRECT 0
// #define REVERSE 1
// #define P_ON_M 0
// #define P_ON_E 1

struct configStruct
{
    double Kp;
    double Ki;
    double Kd;
};

struct configStruct configMS;
struct configStruct configNG;
double msIn, msOut, msSet;
double ngIn, ngOut, ngSet;

PID msPID(&msIn, &msOut, &msSet, configMS.Kp, configMS.Ki, configMS.Kd, 0); 
PID_ATune msTune(&msIn, &msOut);
PID ngPID(&ngIn, &ngOut, &ngSet, configNG.Kp, configNG.Ki, configNG.Kd, DIRECT); 
PID_ATune ngTune(&ngIn, &ngOut);

//#define pidInput gCurrentTemperature
//#define pidSetpoint gSettingTemperature
// PID thePID(&pidInput,&pidOutput,&pidSetpoint,100,40,0,DIRECT);
// ab Zeile 1500 https://github.com/vitotai/BrewManiacEsp8266/blob/master/src/BrewManiac.cpp

bool msPIDAutoTune = false;
bool ngPIDAutoTune = false;
double aTuneStep, aTuneNoise, aTuneStartValue;
unsigned int aTuneLookBack;
double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;
unsigned long  modelTime;

char sudName[15] = "";
unsigned char anzahlRasten = 0;
#define anzahlRastenMax 10
unsigned char anzahlHopfen = 0;
#define anzahlHopfenMax 7
unsigned char anzahlSonstiges = 0;
#define anzahlSonstigesMax 10
// unsigned char anzahlAufgaben = 0;
// #define anzahlAufgabenMax 7
#define sizeImportMax 8192
#define sizeRezeptMax 1024
#define sizeHopfenMax 1024
#define sizeSonstigesMax 1024
#define fileMaischePlan "/maischeplan.txt"
#define fileHopfenPlan "/hopfenplan.txt"
#define fileSonstigesPlan "/sonstigesplan.txt"
// #define TABLE_WIDTH 4
// #define TABLE_HEIGHT 10
// long data[TABLE_WIDTH][TABLE_HEIGHT] = {};
String maischeResponse;
String hopfenResponse;
String sonstigesResponse;
struct MaischePlan
{
    int ID;
    String Name;
    long Dauer;
    int Temperatur;
    long Count;
};
struct HopfenPlan
{
    String Name;
    int ID;
    long Zeit;
    float Menge;
};

struct SonstigesPlan
{
    String Name;
    int ID;
    String Option;
    int Power;
    int Temp;
    String Zeit;
};

struct MaischePlan structMaischePlan[anzahlRastenMax];
struct HopfenPlan structHopfenPlan[anzahlHopfenMax];
struct SonstigesPlan structSonstigesPlan[anzahlSonstigesMax];

// unsigned char numberOfSensors = 0; // Gesamtzahl der Sensoren
unsigned char numberOfSensors = 2; // Gesamtzahl der Sensoren
#define numberOfSensorsMax 2       // Maximale Anzahl an Sensoren
unsigned char addressesFound[numberOfSensorsMax][8];
unsigned char numberOfSensorsFound = 0;
//unsigned char numberOfActors = 0; // Gesamtzahl der Aktoren
unsigned char numberOfActors = 5; // Gesamtzahl der Aktoren
#define numberOfActorsMax 5       // Maximale Anzahl an Aktoren
// char mqtthost[16];                // MQTT Server
// char mqtt_clientid[16];           // AP-Mode und Gerätename
bool alertState = false; // WebUpdate Status

// Zeitserver Einstellungen
#define NTP_OFFSET 60 * 60                // Offset Winterzeit in Sekunden
#define NTP_INTERVAL 60 * 60 * 1000       // Aktualisierung NTP in ms
#define NTP_ADDRESS "europe.pool.ntp.org" // NTP Server
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_ADDRESS, NTP_OFFSET, NTP_INTERVAL);

// System Fehler Events
// #define EM_WLANER 1
// #define EM_REBOOT 11
// // Loop Events
// #define EM_WLAN 20
// #define EM_NTP 25
// #define EM_DB 33
// #define EM_LOG 35

// // Event für Sensoren, Aktor und Induktion
#define EM_OK 0     // Normal mode
#define EM_CRCER 1  // Sensor CRC failed
#define EM_DEVER 2  // Sensor device error
#define EM_UNPL 3   // Sensor unplugged
#define EM_SENER 4  // Sensor all errors
// #define EM_ACTER 10 // Bei Fehler Behandlung von Aktoren
// #define EM_INDER 10 // Bei Fehler Behandlung Induktion

// Event handling Zeitintervall für Reconnects WLAN und MQTT
#define tickerWLAN 20000 // für Ticker Objekt WLAN in ms

int sensorsStatus = 0;
int actorsStatus = 0;
int inductionStatus = 0;

// MaischeSud
char msName[15] = "Maische-Sud";
#define ID_AGITATOR 0
#define ID_MS 0
// unsigned char ms_pin_white = D7;
// unsigned char ms_pin_blue = D5;
// unsigned char ms_pin_yellow = D6;
float mstempSoll = 0.0;
float mspowerSoll = 0.0;

bool msstateAuto = false;
bool msstatePower = false;

bool msstateTimer = false;
bool msdisabledAuto = false;
bool msdisabledPower = false;
bool msdisabledAgitator = false;
bool msdisabledTimer = false;

// Nachguss
char ngName[15] = "Nachguss";
#define ID_HEATER 1
#define ID_NG 1
float ngtempSoll = 0.0;
float ngpowerSoll = 0.0;
bool ngstateAuto = false;
bool ngstatePower = false;
bool ngstateTimer = false;
bool ngdisabledPower = false;
bool ngdisabledTimer = false;

// Misc
#define maxSign 15
char miNameAgitator[maxSign] = "Agitator";
char miNamePump[maxSign] = "Pumpe";
char miNameRHE[maxSign] = "RHE";
#define ID_BUZZER 2
#define ID_PUMP 3
#define ID_RHE 4
#define ID_MI 2
bool mistateAgitator = false;
bool mistatePump = false;
bool mistateRHE = false;
bool mistateRew = false;
bool mistatePause = false;
bool mistateFor = false;
bool midisabledAgitator = false;
bool midisabledPump = false;
bool midisabledRHE = false;
bool midisabledRew = false;
bool midisabledPause = false;
bool midisabledFor = false;
String maischeCounter;
int aktMaischeStep = 0;

//System
// bool startBuzzer = false;
bool testmode = false;

// Ticker Objekte
InnuTicker TickerSen;
InnuTicker TickerAct;
InnuTicker TickerInd;
InnuTicker TickerWLAN;
InnuTicker TickerNTP;
InnuTicker TickerInfluxDB;
InnuTicker TickerMaische;

// Update Intervalle für Ticker Objekte
int SEN_UPDATE = 5000; //  sensors update delay loop
int ACT_UPDATE = 200; //  actors update delay loop
int IND_UPDATE = 200; //  induction update delay loop

// Systemstart
bool startMDNS = true; // Standard mDNS Name ist ESP8266- mit mqtt_chip_key
char nameMDNS[16] = "BrewDevice";
bool shouldSaveConfig = false; // WiFiManager

// Influx Server (optional)
#define numberOfDBMax 3
InfluxDBClient dbClient;
bool startDB = false;
bool startVis = false;
char dbServer[28] = "http://192.168.100.31:8086"; // InfluxDB Server IP
char dbUser[15] = "";
char dbPass[15] = "";
char dbDatabase[11] = "brewdevice";
char dbVisTag[15] = "";
unsigned long upInflux = 15000;

// FSBrowser
File fsUploadFile; // a File object to temporarily store the received file

#define ALARM_ON 1
#define ALARM_OFF 2
#define ALARM_OK 3
#define ALARM_ERROR 4
const int PIN_BUZZER = D8; // Buzzer
bool startBuzzer = false;  // Aktiviere Buzzer

void configModeCallback(WiFiManager *myWiFiManager)
{
    Serial.print("*** SYSINFO: BrewDevice in AP mode ");
    Serial.println(WiFi.softAPIP());
    Serial.print("*** SYSINFO: Start configuration portal ");
    Serial.println(myWiFiManager->getConfigPortalSSID());
}
