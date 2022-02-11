# 1 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino"
//    Erstellt:	2021
//    Author:	Innuendo

# 5 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2
# 6 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2
# 7 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2
# 8 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2
# 9 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2
# 10 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2

# 12 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2
# 13 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2
# 14 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2
# 15 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2
# 16 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2
# 17 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2
# 18 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2
# 19 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2
# 20 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2
# 21 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2
# 22 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2
# 23 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2
# 24 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2
# 25 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2
# 26 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2
# 27 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2
# 28 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2

extern "C"
{
# 32 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino" 2
}
# 48 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino"
// Version


// Definiere Pausen




// OneWire

OneWire oneWire(D3);
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


/*  Binäre Signale für Induktionsplatte */
int CMD[6][33] = {
    {1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0}, // Aus
    {1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0}, // P1
    {1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0}, // P2
    {1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0}, // P3
    {1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0}, // P4
    {1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0}}; // P5
unsigned char PWR_STEPS[] = {0, 20, 40, 60, 80, 100}; // Prozentuale Abstufung zwischen den Stufen

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
PID ngPID(&ngIn, &ngOut, &ngSet, configNG.Kp, configNG.Ki, configNG.Kd, 0);
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
unsigned long modelTime;

char sudName[15] = "";
unsigned char anzahlRasten = 0;

unsigned char anzahlHopfen = 0;

unsigned char anzahlSonstiges = 0;

// unsigned char anzahlAufgaben = 0;
// #define anzahlAufgabenMax 7







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

struct MaischePlan structMaischePlan[10];
struct HopfenPlan structHopfenPlan[7];
struct SonstigesPlan structSonstigesPlan[10];

// unsigned char numberOfSensors = 0; // Gesamtzahl der Sensoren
unsigned char numberOfSensors = 2; // Gesamtzahl der Sensoren

unsigned char addressesFound[2 /* Maximale Anzahl an Sensoren*/][8];
unsigned char numberOfSensorsFound = 0;
//unsigned char numberOfActors = 0; // Gesamtzahl der Aktoren
unsigned char numberOfActors = 5; // Gesamtzahl der Aktoren

// char mqtthost[16];                // MQTT Server
// char mqtt_clientid[16];           // AP-Mode und Gerätename
bool alertState = false; // WebUpdate Status

// Zeitserver Einstellungen



WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org" /* NTP Server*/, 60 * 60 /* Offset Winterzeit in Sekunden*/, 60 * 60 * 1000 /* Aktualisierung NTP in ms*/);

// System Fehler Events
// #define EM_WLANER 1
// #define EM_REBOOT 11
// // Loop Events
// #define EM_WLAN 20
// #define EM_NTP 25
// #define EM_DB 33
// #define EM_LOG 35

// // Event für Sensoren, Aktor und Induktion





// #define EM_ACTER 10 // Bei Fehler Behandlung von Aktoren
// #define EM_INDER 10 // Bei Fehler Behandlung Induktion

// Event handling Zeitintervall für Reconnects WLAN und MQTT


int sensorsStatus = 0;
int actorsStatus = 0;
int inductionStatus = 0;

// MaischeSud
char msName[15] = "Maische-Sud";


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


float ngtempSoll = 0.0;
float ngpowerSoll = 0.0;
bool ngstateAuto = false;
bool ngstatePower = false;
bool ngstateTimer = false;
bool ngdisabledPower = false;
bool ngdisabledTimer = false;

// Misc

char miNameAgitator[15] = "Agitator";
char miNamePump[15] = "Pumpe";
char miNameRHE[15] = "RHE";




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
enum { MSG_OK, CUSTOM, NOT_FOUND, BAD_REQUEST, ERROR };
# 333 "c:\\Arduino\\git\\BrewDevice\\BrewDevice.ino"
const int PIN_BUZZER = D8; // Buzzer
bool startBuzzer = false; // Aktiviere Buzzer

void configModeCallback(WiFiManager *myWiFiManager)
{
    Serial.print("*** SYSINFO: BrewDevice in AP mode ");
    Serial.println(WiFi.softAPIP());
    Serial.print("*** SYSINFO: Start configuration portal ");
    Serial.println(myWiFiManager->getConfigPortalSSID());
}
# 1 "c:\\Arduino\\git\\BrewDevice\\0_SETUP.ino"
void setup()
{
  Serial.begin(115200);
// Debug Ausgaben prüfen




  Serial.println();
  Serial.println();
  // Setze Namen für das MQTTDevice
  // snprintf(mqtt_clientid, 16, "ESP8266-%08X", ESP.getChipId());
  Serial.printf("*** SYSINFO: Starte BrewDevice\n");

  wifiManager.setDebugOutput(false);
  wifiManager.setMinimumSignalQuality(10);
  wifiManager.setConfigPortalTimeout(300);
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  // WiFiManagerParameter cstm_mqtthost("host", "MQTT Server IP (CBPi)", mqtthost, 16);
  // WiFiManagerParameter p_hint("<small>*Sobald das MQTTDevice mit deinem WLAN verbunden ist, öffne im Browser http://mqttdevice </small>");
  // wifiManager.addParameter(&cstm_mqtthost);
  // wifiManager.addParameter(&p_hint);
  // wifiManager.autoConnect(mqtt_clientid);
  wifiManager.autoConnect(nameMDNS);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.mode(WIFI_STA);
  WiFi.persistent(true);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);

  // Lade Dateisystem
  if (LittleFS.begin())
  {
    Serial.printf("*** SYSINFO Starte Setup LITTLEFS Free Heap: %d\n", ESP.getFreeHeap());

    // Prüfe WebUpdate
    // updateSys();

    // Erstelle Ticker Objekte
    setTicker();

    // Starte NTP
    timeClient.begin();
    timeClient.forceUpdate();
    TickerNTP.start();

    if (shouldSaveConfig) // WiFiManager
    {
      // strcpy(mqtthost, cstm_mqtthost.getValue());
      saveConfig();
    }

    if (LittleFS.exists("/config.txt")) // Lade Konfiguration
      loadConfig();
    else
      Serial.println("*** SYSINFO: Konfigurationsdatei config.txt nicht vorhanden. Setze Standardwerte ...");
  }
  else
    Serial.println("*** SYSINFO: Fehler - Dateisystem LITTLEFS konnte nicht eingebunden werden!");

  // Starte Webserver
  setupServer();
  // Pinbelegung
  pins_used[D3] = true;

  // Starte Sensoren
  DS18B20.begin();

  // Starte mDNS
  if (startMDNS)
    setMDNS();
  else
  {
    Serial.printf("*** SYSINFO: ESP8266 IP Addresse: %s Time: %s RSSI: %d\n", WiFi.localIP().toString().c_str(), timeClient.getFormattedTime().c_str(), WiFi.RSSI());
  }

  if (startBuzzer)
  {
    pins_used[PIN_BUZZER] = true;
    pinMode(PIN_BUZZER, 0x01);
    digitalWrite(PIN_BUZZER, 0x0);
  }

  // numberOfSensorsFound = searchSensors();

  setLog(); // webUpdate log
    if (LittleFS.exists("/maischeplan.txt")) // Lade Konfiguration
    readMaischePlan();
  if (LittleFS.exists("/hopfenplan.txt")) // Lade Konfiguration
    readHopfenPlan();
  if (LittleFS.exists("/sonstigesplan.txt")) // Lade Konfiguration
    readSonstigesPlan();
}

void setupServer()
{
  server.on("/", handleRoot);
  server.on("/reqPins", handlereqPins);
  server.on("/reqSearchSensorAdresses", handleRequestSensorAddresses);
  server.on("/reboot", rebootDevice);
  server.on("/visualisieren", visualisieren);

  server.on("/reqMS", handleRequestMS);
  server.on("/reqMSInd", handleRequestMSInd);
  server.on("/setMS", handleSetMS);
  server.on("/reqNG", handleRequestNG);
  server.on("/setNG", handleSetNG);
  server.on("/reqMI", handleRequestMI);
  server.on("/setMI", handleSetMI);
  server.on("/reqSys", handleRequestSys);
  server.on("/setSys", handleSetSys);
  server.on("/reqMaische", handleRequestMaische);
  server.on("/reqHopfen", handleRequestHopfen);
  server.on("/reqSonstiges", handleRequestSonstiges);
  // server.on("/reqReminder", handleRequestReminder);
  server.on("/reqWeb", handleRequestWeb);


  server.on("/Btn-MaischeSud-Auto", BtnMaischeSudAuto);
  server.on("/Btn-MaischeSud-Power", BtnMaischeSudPower);
  server.on("/Btn-MaischeSud-Timer", BtnMaischeSudTimer);
  server.on("/Btn-Nachguss-Auto", BtnNachgussAuto);
  server.on("/Btn-Nachguss-Power", BtnNachgussPower);
  server.on("/Btn-Nachguss-Timer", BtnNachgussTimer);
  server.on("/Btn-Misc-Agitator", BtnMiscAgitator);
  server.on("/Btn-Misc-Pump", BtnMiscPump);
  server.on("/Btn-Misc-RHE", BtnMiscRHE);
  server.on("/Btn-Misc-Rewind", BtnMiscRew);
  server.on("/Btn-Misc-Pause", BtnMiscPause);
  server.on("/Btn-Misc-Forwind", BtnMiscFor);
  server.on("/Slider-MaischeSud-TempSoll", SliderMaischeSudTemp);
  server.on("/Slider-MaischeSud-PowerSoll", SliderMaischeSudPower);
  server.on("/Slider-Nachguss-TempSoll", SliderNachgussTemp);
  server.on("/Slider-Nachguss-PowerSoll", SliderNachgussPower);
  server.on("/Btn-Import-KBH2", BtnImportKBH2);
  server.on("/setMaischeTable", SetMaischeTable);
  server.on("/setHopfenTable", SetHopfenTable);
  server.on("/setSonstigesTable", SetSonstigesTable);
  server.on("/msAutoTune", msAutoTune);
  server.on("/msCancelAutoTune", msCancelAutoTune);
  server.on("/ngAutoTune", ngAutoTune);
  server.on("/ngCancelAutoTune", ngCancelAutoTune);
  server.on("/reqStepsCount", handleRequestStepsCount);
  server.on("/reqActNames", handleRequestActorNames);


  // FSBrowser initialisieren
  server.on("/edit", HTTP_GET, handleGetEdit);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/list", HTTP_GET, handleFileList);
  server.on("/edit", HTTP_PUT, handleFileCreate);
  server.on("/favicon.ico", HTTP_GET, replyOK);
  server.on("/edit", HTTP_DELETE, handleFileDelete);
  server.on("/edit", HTTP_POST, []() { server.send(200, "text/plain", ""); }, handleFileUpload);
  server.onNotFound(handleWebRequests);

  httpUpdate.setup(&server);
  server.begin();
}
# 1 "c:\\Arduino\\git\\BrewDevice\\1_LOOP.ino"
void loop()
{
  server.handleClient(); // Webserver handle

  if (startMDNS) // MDNS handle
    mdns.update();



  // if (numberOfSensors > 0)  // Sensoren Ticker
    TickerSen.update();
  // if (numberOfActors > 0)   // Aktoren Ticker
    TickerAct.update();
  // if (inductionStatus > 0)  // Induktion Ticker
    TickerInd.update();
  // if (useDisplay)           // Display Ticker
    // TickerDisp.update();
  // if (startDB && startVis)  // InfluxDB Ticker
  //   TickerInfluxDB.update();
  if (TickerMaische.state() == RUNNING)
    TickerMaische.update();

  TickerNTP.update(); // NTP Ticker
}
# 1 "c:\\Arduino\\git\\BrewDevice\\2_SENSOREN.ino"
class TemperatureSensor
{
public:
  unsigned char sens_address[8]; // 1-Wire Adresse
  String sens_name; // Name für Anzeige auf Website
  float sens_value = -127.0; // Aktueller Wert
  bool sens_isConnected; // ist der Sensor verbunden
  float sens_offset = 0.0; // Offset - Temp kalibrieren
  bool sens_state = true; // Fehlerstatus ensor
  int sens_err = 0;

  String getSens_adress_string()
  {
    return SensorAddressToString(sens_address);
  }

  TemperatureSensor(String new_address, String new_name, float new_offset)
  {
    change(new_address, new_name, new_offset);
  }

  void Update()
  {
    DS18B20.requestTemperatures(); // new conversion to get recent temperatures
    sens_isConnected = DS18B20.isConnected(sens_address); // attempt to determine if the device at the given address is connected to the bus
    sens_isConnected ? sens_value = DS18B20.getTempC(sens_address) : sens_value = -127.0;

    if (!sens_isConnected && sens_address[0] != 0xFF && sens_address[0] != 0x00) // double check on !sens_isConnected. Billig Tempfühler ist manchmal für 1-2 loops nicht connected. 0xFF default address. 0x00 virtual test device (adress 00 00 00 00 00)
    {
      millis2wait(750); // Wartezeit ca 750ms bevor Lesen vom Sensor wiederholt wird (Init Zeit)
      sens_isConnected = DS18B20.isConnected(sens_address); // hat der Sensor ene Adresse und ist am Bus verbunden?
      sens_isConnected ? sens_value = DS18B20.getTempC(sens_address) : sens_value = -127.0;
    }

    if (sens_value == 85.0)
    { // 85 Grad ist Standard Temp Default Reset. Wenn das Kabel zu lang ist, kommt als Fehler 85 Grad
      millis2wait(750); // Wartezeit 750ms vor einer erneuten Sensorabfrage
      DS18B20.requestTemperatures();
    }
    sensorsStatus = 0;
    sens_state = true;
    if (OneWire::crc8(sens_address, 7) != sens_address[7])
    {
      sensorsStatus = 1 /* Sensor CRC failed*/;
      sens_state = false;
    }
    else if (sens_value == -127.00 || sens_value == 85.00)
    {
      if (sens_isConnected && sens_address[0] != 0xFF)
      { // Sensor connected AND sensor address exists (not default FF)
        sensorsStatus = 2 /* Sensor device error*/;
        sens_state = false;
      }
      else if (!sens_isConnected && sens_address[0] != 0xFF)
      { // Sensor with valid address not connected
        sensorsStatus = 3 /* Sensor unplugged*/;
        sens_state = false;
      }
      else // not connected and unvalid address
      {
        sensorsStatus = 4 /* Sensor all errors*/;
        sens_state = false;
      }
    } // sens_value -127 || +85
    else
    {
      sensorsStatus = 0 /* Normal mode*/;
      sens_state = true;
    }
    sens_err = sensorsStatus;
  } // void Update

  void change(const String &new_address, const String &new_name, float new_offset)
  {
    sens_name = new_name;
    sens_offset = new_offset;

    if (new_address.length() == 16)
    {
      char address_char[16];

      new_address.toCharArray(address_char, 17);

      char hexbyte[2];
      int octets[8];

      for (int d = 0; d < 16; d += 2)
      {
        // Assemble a digit pair into the hexbyte string
        hexbyte[0] = address_char[d];
        hexbyte[1] = address_char[d + 1];

        // Convert the hex pair to an integer
        sscanf(hexbyte, "%x", &octets[d / 2]);
        yield();
      }
      for (int i = 0; i < 8; i++)
      {
        sens_address[i] = octets[i];
      }
    }
    DS18B20.setResolution(sens_address, 10);
  }

  char buf[5];
  char *getValueString()
  {
    // char buf[5];
    dtostrf(sens_value, 2, 1, buf);
    return buf;
  }
};

// Initialisierung des Arrays -> max 6 Sensoren
TemperatureSensor sensors[2 /* Maximale Anzahl an Sensoren*/] = {
    TemperatureSensor("", "", 0.0), // MaischeSud
    TemperatureSensor("", "", 0.0)}; // Nachguss

// Funktion für Loop im Timer Objekt
void handleSensors()
{
  int max_status = 0;
  for (int i = 0; i < numberOfSensors; i++)
  {
    sensors[i].Update();
    // get max sensorstatus
    if (max_status < sensors[i].sens_err)
      max_status = sensors[i].sens_err;

    yield();
  }
  sensorsStatus = max_status;
}

unsigned char searchSensors()
{
  unsigned char i;
  unsigned char n = 0;
  unsigned char addr[8];

  while (oneWire.search(addr))
  {

    if (OneWire::crc8(addr, 7) == addr[7])
    {
      for (i = 0; i < 8; i++)
      {
        addressesFound[n][i] = addr[i];
      }
      n += 1;
    }
    yield();
  }
  return n;
  oneWire.reset_search();
}

String SensorAddressToString(unsigned char addr[8])
{
  char charbuffer[50];
  sprintf(charbuffer, "%02x%02x%02x%02x%02x%02x%02x%02x", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7]);
  return charbuffer;
}

void handleRequestSensorAddresses()
{
  numberOfSensorsFound = searchSensors();
  if (numberOfSensorsFound == 0)
  {
    delay(500);
    numberOfSensorsFound = searchSensors();
    // DEBUG_MSG("ReqSenAddress search %d \n", numberOfSensorsFound);
  }
  int id = server.arg(0).toInt();
  // DEBUG_MSG("ReqSenAddress: ID %d numberOfSen %d\n", id, numberOfSensorsFound);
  String message;
  if (id != -1)
  {
    message += ((reinterpret_cast<const __FlashStringHelper *>(
# 179 "c:\\Arduino\\git\\BrewDevice\\2_SENSOREN.ino" 3
              (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "2_SENSOREN.ino" "." "179" "." "293" "\", \"aSM\", @progbits, 1 #"))) = (
# 179 "c:\\Arduino\\git\\BrewDevice\\2_SENSOREN.ino"
              "<option>"
# 179 "c:\\Arduino\\git\\BrewDevice\\2_SENSOREN.ino" 3
              ); &__pstr__[0];}))
# 179 "c:\\Arduino\\git\\BrewDevice\\2_SENSOREN.ino"
              )));
    message += SensorAddressToString(sensors[id].sens_address);
    message += ((reinterpret_cast<const __FlashStringHelper *>(
# 181 "c:\\Arduino\\git\\BrewDevice\\2_SENSOREN.ino" 3
              (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "2_SENSOREN.ino" "." "181" "." "294" "\", \"aSM\", @progbits, 1 #"))) = (
# 181 "c:\\Arduino\\git\\BrewDevice\\2_SENSOREN.ino"
              "</option><option disabled>──────────</option>"
# 181 "c:\\Arduino\\git\\BrewDevice\\2_SENSOREN.ino" 3
              ); &__pstr__[0];}))
# 181 "c:\\Arduino\\git\\BrewDevice\\2_SENSOREN.ino"
              )));
    // DEBUG_MSG("ReqSenAddress: ID %d SenAddress %s\n", id, sensors[id].sens_address );

  }
  for (int i = 0; i < numberOfSensorsFound; i++)
  {
    message += ((reinterpret_cast<const __FlashStringHelper *>(
# 187 "c:\\Arduino\\git\\BrewDevice\\2_SENSOREN.ino" 3
              (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "2_SENSOREN.ino" "." "187" "." "295" "\", \"aSM\", @progbits, 1 #"))) = (
# 187 "c:\\Arduino\\git\\BrewDevice\\2_SENSOREN.ino"
              "<option>"
# 187 "c:\\Arduino\\git\\BrewDevice\\2_SENSOREN.ino" 3
              ); &__pstr__[0];}))
# 187 "c:\\Arduino\\git\\BrewDevice\\2_SENSOREN.ino"
              )));
    message += SensorAddressToString(addressesFound[i]);
    message += ((reinterpret_cast<const __FlashStringHelper *>(
# 189 "c:\\Arduino\\git\\BrewDevice\\2_SENSOREN.ino" 3
              (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "2_SENSOREN.ino" "." "189" "." "296" "\", \"aSM\", @progbits, 1 #"))) = (
# 189 "c:\\Arduino\\git\\BrewDevice\\2_SENSOREN.ino"
              "</option>"
# 189 "c:\\Arduino\\git\\BrewDevice\\2_SENSOREN.ino" 3
              ); &__pstr__[0];}))
# 189 "c:\\Arduino\\git\\BrewDevice\\2_SENSOREN.ino"
              )));
    yield();
  }
  server.send(200, "text/html", message);
}
# 1 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino"
class Actor
{
  unsigned long powerLast; // Zeitmessung für High oder Low
  int dutycycle_actor = 5000;
  unsigned char OFF;
  unsigned char ON;
  bool isInverted = false;
  bool pwm = false;
  bool setGrafana = false;
  bool isOn;

public:
  unsigned char pin_actor = 9; // the number of the LED pin
  String name_actor;
  unsigned char power_actor;

  bool actor_state = true; // Error state actor

  Actor(String pin, String aname, bool ainverted, bool apwm, bool agrafana)
  {
    change(pin, aname, ainverted, apwm, agrafana);
  }

  void Update()
  {
    if (isPin(pin_actor))
    {
      if (isOn && power_actor > 0)
      {
        if (millis() > powerLast + dutycycle_actor)
        {
          powerLast = millis();
        }
        if (millis() > powerLast + (dutycycle_actor * power_actor / 100L))
        {
          digitalWrite(pin_actor, OFF);
        }
        else
        {
          digitalWrite(pin_actor, ON);
        }
      }
      else
      {
        digitalWrite(pin_actor, OFF);
      }
    }
  }

  void change(const String &pin, const String &aname, bool ainverted, bool apwm, bool agrafana)
  {
    // Set PIN
    if (isPin(pin_actor))
    {
      digitalWrite(pin_actor, 0x1);
      pins_used[pin_actor] = false;
      millis2wait(10);
    }

    pin_actor = StringToPin(pin);
    if (isPin(pin_actor))
    {
      pinMode(pin_actor, 0x01);
      digitalWrite(pin_actor, 0x1);
      pins_used[pin_actor] = true;
    }

    isOn = false;

    name_actor = aname;
    if (ainverted == true)
    {
      isInverted = true;
      ON = 0x1;
      OFF = 0x0;
    }
    else
    {
      isInverted = false;
      ON = 0x0;
      OFF = 0x1;
    }
    if (apwm)
      pwm = true;
    else
      pwm = false;

    actor_state = true;
    //agrafana == "1" ? setGrafana = true :
    setGrafana = agrafana;
  }

  void powerOn(int newpower)
  {
    isOn = true;
    power_actor = min(100, newpower);
    power_actor = max(0, newpower);
  }

  void powerOff()
  {
    isOn = false;
    power_actor = 0;
  }

  bool getInverted()
  {
    return isInverted;
  }

  bool getPWM()
  {
    return pwm;
  }

  bool getGrafana()
  {
    return setGrafana;
  }
};

// Initialisierung des Arrays max 5
Actor actors[5 /* Maximale Anzahl an Aktoren*/] = {
    Actor("", "", false, false, false), // Agitator
    Actor("", "", false, false, false), // Heater
    Actor("", "", false, false, false), // Buzzer
    Actor("", "", false, false, false), // Pump
    Actor("", "", false, false, false)}; // RHE

// Funktionen für Loop im Timer Objekt
void handleActors()
{
  for (int i = 0; i < numberOfActors; i++)
  {
    actors[i].Update();
    yield();
  }
}

void handlereqPins()
{
  int id = server.arg(0).toInt();
  String message;

  if (id != -1)
  {
    message += ((reinterpret_cast<const __FlashStringHelper *>(
# 147 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino" 3
              (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "3_AKTOREN.ino" "." "147" "." "297" "\", \"aSM\", @progbits, 1 #"))) = (
# 147 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino"
              "<option>"
# 147 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino" 3
              ); &__pstr__[0];}))
# 147 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino"
              )));
    message += PinToString(actors[id].pin_actor);
    message += ((reinterpret_cast<const __FlashStringHelper *>(
# 149 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino" 3
              (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "3_AKTOREN.ino" "." "149" "." "298" "\", \"aSM\", @progbits, 1 #"))) = (
# 149 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino"
              "</option><option disabled>──────────</option>"
# 149 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino" 3
              ); &__pstr__[0];}))
# 149 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino"
              )));
  }
  for (int i = 0; i < numberOfPins; i++)
  {
    if (pins_used[pins[i]] == false)
    {
      message += ((reinterpret_cast<const __FlashStringHelper *>(
# 155 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino" 3
                (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "3_AKTOREN.ino" "." "155" "." "299" "\", \"aSM\", @progbits, 1 #"))) = (
# 155 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino"
                "<option>"
# 155 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino" 3
                ); &__pstr__[0];}))
# 155 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino"
                )));
      message += pin_names[i];
      message += ((reinterpret_cast<const __FlashStringHelper *>(
# 157 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino" 3
                (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "3_AKTOREN.ino" "." "157" "." "300" "\", \"aSM\", @progbits, 1 #"))) = (
# 157 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino"
                "</option>"
# 157 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino" 3
                ); &__pstr__[0];}))
# 157 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino"
                )));
    }
    yield();
  }
  message += ((reinterpret_cast<const __FlashStringHelper *>(
# 161 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino" 3
            (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "3_AKTOREN.ino" "." "161" "." "301" "\", \"aSM\", @progbits, 1 #"))) = (
# 161 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino"
            "<option>"
# 161 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino" 3
            ); &__pstr__[0];}))
# 161 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino"
            )));
  message += "deaktivieren";
  message += ((reinterpret_cast<const __FlashStringHelper *>(
# 163 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino" 3
            (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "3_AKTOREN.ino" "." "163" "." "302" "\", \"aSM\", @progbits, 1 #"))) = (
# 163 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino"
            "</option>"
# 163 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino" 3
            ); &__pstr__[0];}))
# 163 "c:\\Arduino\\git\\BrewDevice\\3_AKTOREN.ino"
            )));
  server.send(200, "text/plain", message);
}

unsigned char StringToPin(String pinstring)
{
  for (int i = 0; i < numberOfPins; i++)
  {
    if (pin_names[i] == pinstring)
    {
      return pins[i];
    }
  }
  return 9;
}

String PinToString(unsigned char pinbyte)
{
  for (int i = 0; i < numberOfPins; i++)
  {
    if (pins[i] == pinbyte)
    {
      return pin_names[i];
    }
  }
  // return "NaN";
  return "";
}

bool isPin(unsigned char pinbyte)
{
  bool returnValue = false;
  for (int i = 0; i < numberOfPins; i++)
  {
    if (pins[i] == pinbyte)
    {
      returnValue = true;
      goto Ende;
    }
  }
Ende:
  return returnValue;
}
# 1 "c:\\Arduino\\git\\BrewDevice\\4_INDUKTION.ino"
class induction
{
  unsigned long timeTurnedoff;

  long timeOutCommand = 5000; // TimeOut für Seriellen Befehl
  long timeOutReaction = 2000; // TimeOut für Induktionskochfeld
  unsigned long lastInterrupt;
  unsigned long lastCommand;
  bool inputStarted = false;
  unsigned char inputCurrent = 0;
  unsigned char inputBuffer[33];
  bool isError = false;
  unsigned char error = 0;
  long powerSampletime = 20000;
  unsigned long powerLast;
  long powerHigh = powerSampletime; // Dauer des "HIGH"-Anteils im Schaltzyklus
  long powerLow = 0;

public:
  unsigned char PIN_WHITE = 9; // RELAIS
  unsigned char PIN_YELLOW = 9; // AUSGABE AN PLATTE
  unsigned char PIN_INTERRUPT = 9; // EINGABE VON PLATTE
  int power = 0;
  int newPower = 0;
  unsigned char CMD_CUR = 0; // Aktueller Befehl
  boolean isRelayon = false; // Systemstatus: ist das Relais in der Platte an?
  boolean isInduon = false; // Systemstatus: ist Power > 0?
  boolean isPower = false;
  boolean isEnabled = false;
  long delayAfteroff = 120000;
  bool setGrafana = false;

  induction()
  {
    setupCommands();
  }

  void change(unsigned char pinwhite, unsigned char pinyellow, unsigned char pinblue, long delayoff, bool is_enabled, bool new_grafana)
  {
    if (isEnabled)
    {
      // aktuelle PINS deaktivieren
      if (isPin(PIN_WHITE))
      {
        digitalWrite(PIN_WHITE, 0x1);
        pins_used[PIN_WHITE] = false;
      }

      if (isPin(PIN_YELLOW))
      {
        digitalWrite(PIN_YELLOW, 0x1);
        pins_used[PIN_YELLOW] = false;
      }

      if (isPin(PIN_INTERRUPT))
      {
        digitalWrite(PIN_INTERRUPT, 0x1);
        pins_used[PIN_INTERRUPT] = false;
      }
    }

    // Neue Variablen Speichern
    PIN_WHITE = pinwhite;
    PIN_YELLOW = pinyellow;
    PIN_INTERRUPT = pinblue;
    delayAfteroff = delayoff;
    setGrafana = new_grafana;
    isEnabled = is_enabled;

    if (isEnabled)
    {
      // neue PINS aktiveren
      if (isPin(PIN_WHITE))
      {
        pinMode(PIN_WHITE, 0x01);
        digitalWrite(PIN_WHITE, 0x0);
        pins_used[PIN_WHITE] = true;
      }

      if (isPin(PIN_YELLOW))
      {
        pinMode(PIN_YELLOW, 0x01);
        digitalWrite(PIN_YELLOW, 0x0);
        pins_used[PIN_YELLOW] = true;
      }

      if (isPin(PIN_INTERRUPT))
      {
        pinMode(PIN_INTERRUPT, 0x02);
        pins_used[PIN_INTERRUPT] = true;
      }
    }
  }

  void powerInd(int value)
  {
    newPower = value;
  }

  void setupCommands()
  {
    for (int i = 0; i < 33; i++)
    {
      for (int j = 0; j < 6; j++)
      {
        if (CMD[j][i] == 1)
        {
          CMD[j][i] = SIGNAL_HIGH;
        }
        else
        {
          CMD[j][i] = SIGNAL_LOW;
        }
      }
    }
  }

  bool updateRelay()
  {
    if (isInduon == true && isRelayon == false)
    { /* Relais einschalten */
      digitalWrite(PIN_WHITE, 0x1);
      return true;
    }

    if (isInduon == false && isRelayon == true)
    { /* Relais ausschalten */
      if (millis() > timeTurnedoff + delayAfteroff)
      {
        digitalWrite(PIN_WHITE, 0x0);
        return false;
      }
    }

    if (isInduon == false && isRelayon == false)
    { /* Ist aus, bleibt aus. */
      return false;
    }

    return true; /* Ist an, bleibt an. */
  }

  void Update()
  {
    updatePower();

    isRelayon = updateRelay();

    if (isInduon && power > 0)
    {
      if (millis() > powerLast + powerSampletime)
      {
        powerLast = millis();
      }
      if (millis() > powerLast + powerHigh)
      {
        sendCommand(CMD[CMD_CUR - 1]);
        isPower = false;
      }
      else
      {
        sendCommand(CMD[CMD_CUR]);
        isPower = true;
      }
    }
    else if (isRelayon)
    {
      sendCommand(CMD[0]);
    }
  }

  void updatePower()
  {
    lastCommand = millis();

    if (power != newPower)
    { /* Neuer Befehl empfangen */

      if (newPower > 100)
      {
        newPower = 100; /* Nicht > 100 */
      }
      if (newPower < 0)
      {
        newPower = 0; /* Nicht < 0 */
      }
      power = newPower;

      timeTurnedoff = 0;
      isInduon = true;
      long difference = 0;

      if (power == 0)
      {
        CMD_CUR = 0;
        timeTurnedoff = millis();
        isInduon = false;
        difference = 0;
        goto setPowerLevel;
      }

      for (int i = 1; i < 7; i++)
      {
        if (power <= PWR_STEPS[i])
        {
          CMD_CUR = i;
          difference = PWR_STEPS[i] - power;
          goto setPowerLevel;
        }
      }

    setPowerLevel: /* Wie lange "HIGH" oder "LOW" */
      if (difference != 0)
      {
        powerLow = powerSampletime * difference / 20L;
        powerHigh = powerSampletime - powerLow;
      }
      else
      {
        powerHigh = powerSampletime;
        powerLow = 0;
      }
    }
  }

  void sendCommand(int command[33])
  {
    digitalWrite(PIN_YELLOW, 0x1);
    millis2wait(SIGNAL_START);
    digitalWrite(PIN_YELLOW, 0x0);
    millis2wait(SIGNAL_WAIT);
    for (int i = 0; i < 33; i++)
    {
      digitalWrite(PIN_YELLOW, 0x1);
      delayMicroseconds(command[i]);
      digitalWrite(PIN_YELLOW, 0x0);
      delayMicroseconds(SIGNAL_LOW);
    }
  }
};

induction inductionCooker = induction();

void handleInduction()
{
  inductionCooker.Update();
}
# 1 "c:\\Arduino\\git\\BrewDevice\\6_InfluxDB.ino"
class DBServer
{
public:
    String kettle_id = ""; // Kettle ID
    String kettle_topic = ""; // Kettle Topic
    String kettle_heater_topic = ""; // Kettle Heater MQTT Topic
    // String kettle_name = "";          // Kettle Name
    float kettle_sensor_temp = 0.0; // Kettle Sensor aktuelle Temperatur
    int kettle_target_temp = 0; // Kettle Heater TargetTemp
    int kettle_heater_powerlevel = 0; // Kettle Heater aktueller Powerlevel
    int kettle_heater_state = 0; // Kettle Heater Status (on/off)
    int dbEnabled = -1; // Topic existiert nicht

    DBServer(String new_kettle_id, String new_kettle_topic)
    {
        kettle_id = new_kettle_id;
        kettle_topic = new_kettle_topic;
    }
/*

    void mqtt_subscribe()

    {

        if (pubsubClient.connected())

        {

            char subscribemsg[50];

            kettle_topic.toCharArray(subscribemsg, 50);

            pubsubClient.subscribe(subscribemsg);

            if (!pubsubClient.subscribe(subscribemsg))

            {

                DEBUG_MSG("%s\n", "InfluxMQTT Fehler");

            }

            else

            {

                DEBUG_MSG("InfluxMQTT: Subscribing to %s\n", subscribemsg);

            }

        }

    }

    void mqtt_unsubscribe()

    {

        if (pubsubClient.connected())

        {

            char subscribemsg[50];

            kettle_topic.toCharArray(subscribemsg, 50);

            DEBUG_MSG("InfluxMQTT: Unsubscribing from %s\n", subscribemsg);

            pubsubClient.unsubscribe(subscribemsg);

        }

    }



    void handlemqtt(char *payload)

    {

        StaticJsonDocument<256> doc;

        DeserializationError error = deserializeJson(doc, (const char *)payload);

        if (error)

        {

            DEBUG_MSG("TCP: handlemqtt deserialize Json error %s\n", error.c_str());

            return;

        }

        kettle_id = doc["id"].as<String>();

        if (isValidInt(doc["tt"].as<String>()))

            kettle_target_temp = doc["tt"];

        else

            kettle_target_temp = 0;

        if (isValidFloat(doc["te"].as<String>()))

            kettle_sensor_temp = doc["te"];

        else

            kettle_sensor_temp = 0.0;

        kettle_heater_topic = doc["he"].as<String>();

        DEBUG_MSG("Influx handleMQTT dbEn: %d ID: %s State: %d Target: %d Temp: %f Power: %d Topic: %s\n", dbEnabled, kettle_id.c_str(), kettle_heater_state, kettle_target_temp, kettle_sensor_temp, kettle_heater_powerlevel, kettle_heater_topic.c_str());

        //if (dbEnabled == -1)

        if (dbEnabled != 0)

        {

            if (kettle_heater_topic == inductionCooker.mqtttopic)

            {

                if (inductionCooker.setGrafana)

                {

                    dbEnabled = 1;

                    kettle_heater_state = inductionCooker.isInduon;

                    kettle_heater_powerlevel = inductionCooker.power;

                    return;

                }

            }

            for (int j = 0; j < numberOfActors; j++) // Array Aktoren

            {

                if (kettle_heater_topic == actors[j].argument_actor)

                {

                    if (actors[j].setGrafana)

                    {

                        dbEnabled = 1;

                        kettle_heater_state = actors[j].actor_state;

                        kettle_heater_powerlevel = actors[j].power_actor;

                        return;

                    }

                }

            }

            if (dbEnabled == -1)

            {

                dbEnabled = 0;

                mqtt_unsubscribe();

            }

        }

    }

    */
# 102 "c:\\Arduino\\git\\BrewDevice\\6_InfluxDB.ino"
};

// Erstelle Array mit Kettle ID CBPi
DBServer dbInflux[3] = {
    DBServer("1", "MQTTDevice/kettle/1"),
    DBServer("2", "MQTTDevice/kettle/2"),
    DBServer("3", "MQTTDevice/kettle/3")};

void sendData()
{
    for (int i = 0; i < 3; i++)
    {
        if (dbInflux[i].dbEnabled != 1)
            continue;

        Point dbData("mqttdevice_status");
        dbData.addTag("ID", dbInflux[i].kettle_id);
        if (dbVisTag[0] != '\0')
            dbData.addTag("Sud-ID", dbVisTag);
        dbData.addField("Temperatur", dbInflux[i].kettle_sensor_temp);
        dbData.addField("TargetTemp", dbInflux[i].kettle_target_temp);
        if (dbInflux[i].kettle_heater_state == 1)
            dbData.addField("Powerlevel", dbInflux[i].kettle_heater_powerlevel);
        else
            dbData.addField("Powerlevel", 0);
        ;

        if (!dbClient.writePoint(dbData))
        {
            ;
        }
    }
}

void setInfluxDB()
{
    // Setze Parameter
    dbClient.setConnectionParamsV1(dbServer, dbDatabase, dbUser, dbPass);
}

bool checkDBConnect()
{
    if (dbClient.validateConnection())
    {
        ;
        return true;
    }
    else
    {
        ;
        return false;
    }
}
# 1 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
void handleRoot()
{
  server.sendHeader("Location", "/index.html", true); //Redirect to our html web page
  server.send(302, "text/plain", "");
  // server.sendHeader(PSTR("Content-Encoding"), "gzip");
  // server.send(200, "text/html", index_htm_gz, sizeof(index_htm_gz));
}

void handleWebRequests()
{
  if (loadFromLittlefs(server.uri()))
  {
    return;
  }
  String message = "File Not Detected\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++)
  {
    message += " NAME:" + server.argName(i) + "\n VALUE:" + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

bool loadFromLittlefs(String path)
{
  String dataType = "text/plain";
  if (path.endsWith("/"))
    path += "index.html";

  if (path.endsWith(".src"))
    path = path.substring(0, path.lastIndexOf("."));
  else if (path.endsWith(".html"))
    dataType = "text/html";
  else if (path.endsWith(".htm"))
    dataType = "text/html";
  else if (path.endsWith(".css"))
    dataType = "text/css";
  else if (path.endsWith(".js"))
    dataType = "application/javascript";
  else if (path.endsWith(".png"))
    dataType = "image/png";
  else if (path.endsWith(".gif"))
    dataType = "image/gif";
  else if (path.endsWith(".jpg"))
    dataType = "image/jpeg";
  else if (path.endsWith(".ico"))
    dataType = "image/x-icon";
  else if (path.endsWith(".xml"))
    dataType = "text/xml";
  else if (path.endsWith(".pdf"))
    dataType = "application/pdf";
  else if (path.endsWith(".zip"))
    dataType = "application/zip";

  if (!LittleFS.exists(path.c_str()))
  {
    return false;
  }
  File dataFile = LittleFS.open(path.c_str(), "r");
  if (server.hasArg("download"))
    dataType = "application/octet-stream";
  if (server.streamFile(dataFile, dataType) != dataFile.size())
  {
  }
  dataFile.close();
  return true;
}

void handleRequestFirm()
{
  String request = server.arg(0);
  String message;
  if (request == "firmware")
  {
    if (startMDNS)
    {
      message = nameMDNS;
      message += " V";
    }
    else
      message = "MQTTDevice V ";
    message += "1.00";
    goto SendMessage;
  }

SendMessage:
  server.send(200, "text/plain", message);
}

void handleRequestSys()
{
  StaticJsonDocument<512> doc;
  doc["buzzer"] = startBuzzer;
  doc["mdnsname"] = nameMDNS;
  doc["mdns"] = startMDNS;
  doc["upsen"] = SEN_UPDATE / 1000;
  doc["upact"] = ACT_UPDATE / 1000;
  doc["upind"] = IND_UPDATE / 1000;
  doc["updb"] = upInflux / 1000;
  doc["dbserver"] = dbServer;
  doc["startdb"] = startDB;
  doc["dbdatabase"] = dbDatabase;
  doc["dbuser"] = dbUser;
  doc["dbpass"] = dbPass;
  doc["dbup"] = (upInflux / 1000);
  doc["testmode"] = testmode;

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleSetSys()
{
  for (int i = 0; i < server.args(); i++)
  {
    if (server.argName(i) == "reset")
    {
      if (server.arg(i) == "1")
      {
        // DEBUG_MSG("%s\n", "Reset settings");
        WiFi.disconnect();
        wifiManager.resetSettings();
        delay(2000);
        ESP.reset();
      }
    }
    if (server.argName(i) == "clear")
    {
      if (server.arg(i) == "1")
      {
        // DEBUG_MSG("%s\n", "Clear settings");
        LittleFS.remove("/config.txt");
        WiFi.disconnect();
        wifiManager.resetSettings();
        delay(2000);
        ESP.reset();
      }
    }
    if (server.argName(i) == "buzzer")
    {
      if (server.arg(i) == "true")
        startBuzzer = true;
      else
        startBuzzer = false;
    }
    if (server.argName(i) == "mdns_name")
    {
      server.arg(i).toCharArray(nameMDNS, 16);
      checkChars(nameMDNS);
    }
    if (server.argName(i) == "mdns")
    {
      if (server.arg(i) == "true")
        startMDNS = true;
      else
        startMDNS = false;
    }
    if (server.argName(i) == "upsen")
    {
      if (isValidInt(server.arg(i)))
      {
        int newsup = server.arg(i).toInt();
        if (newsup > 0)
          SEN_UPDATE = newsup * 1000;
      }
    }
    if (server.argName(i) == "upact")
    {
      if (isValidInt(server.arg(i)))
      {
        int newaup = server.arg(i).toInt();
        if (newaup > 0)
          ACT_UPDATE = newaup * 1000;
      }
    }
    if (server.argName(i) == "upind")
    {
      if (isValidInt(server.arg(i)))
      {
        int newiup = server.arg(i).toInt();
        if (newiup > 0)
          IND_UPDATE = newiup * 1000;
      }
    }
    if (server.argName(i) == "dbserver")
    {
      server.arg(i).toCharArray(dbServer, 30);
      checkChars(dbServer);
    }
    if (server.argName(i) == "startdb")
    {
      if (server.arg(i) == "true")
        startDB = true;
      else
        startDB = false;
    }
    if (server.argName(i) == "dbdatabase")
    {
      server.arg(i).toCharArray(dbDatabase, 15);
      checkChars(dbDatabase);
    }
    if (server.argName(i) == "dbuser")
    {
      server.arg(i).toCharArray(dbUser, 15);
      checkChars(dbUser);
    }
    if (server.argName(i) == "dbpass")
    {
      server.arg(i).toCharArray(dbPass, 15);
      checkChars(dbPass);
    }
    if (server.argName(i) == "updb")
    {
      if (isValidInt(server.arg(i)))
      {
        upInflux = server.arg(i).toInt() * 1000;
      }
    }
    if (server.argName(i) == "testmode")
    {
      if (server.arg(i) == "true")
        testmode = true;
      else
        testmode = false;
    }
    yield();
  }
  server.send(201, "text/plain", "created");
  saveConfig();
}

// Some helper functions WebIf
void visualisieren()
{
  for (int i = 0; i < server.args(); i++)
  {
    if (server.argName(i) == "vistag")
    {
      server.arg(i).toCharArray(dbVisTag, 15);
      checkChars(dbVisTag);
    }
    if (server.argName(i) == "startvis")
    {
      if (server.arg(i) == "1")
        startVis = true;
      else
        startVis = false;
    }
    yield();
  }
  if (startDB && startVis)
  {
    if (checkDBConnect())
    {
      startVis = true;
      TickerInfluxDB.interval(upInflux);
      TickerInfluxDB.start();
    }
    else
    {
      startVis = false;
      TickerInfluxDB.stop();
    }
    // TickerInfluxDB.resume();
  }
  else
    TickerInfluxDB.pause();
}

void BtnMaischeSudAuto()
{
  for (int i = 0; i < server.args(); i++)
  {
    if (server.argName(i) == "msstateAuto")
    {
      if (server.arg(i) == "true")
      {
        msstateAuto = true;
        TickerMaische.config(structMaischePlan[aktMaischeStep].Dauer, 1);
        mstempSoll = structMaischePlan[aktMaischeStep].Temperatur;
        TickerMaische.start();
        ;
      }
      else
      {
        msstateAuto = false;
        TickerMaische.stop();
      }
    }
    if (server.argName(i) == "msdisabledPower")
    {
      if (server.arg(i) == "true")
        msdisabledPower = true;
      else
        msdisabledPower = false;
    }
  }

  server.send(201, "text/plain", "created");
}

void BtnMaischeSudPower()
{
  for (int i = 0; i < server.args(); i++)
  {
    if (server.argName(i) == "msstatePower")
    {
      if (server.arg(i) == "true")
        msstatePower = true;
      else
        msstatePower = false;
    }

    if (server.argName(i) == "msdisabledAuto")
    {
      if (server.arg(i) == "true")
        msdisabledAuto = true;
      else
        msdisabledAuto = false;
    }
  }
  server.send(201, "text/plain", "created");
}
void BtnMaischeSudTimer()
{
  for (int i = 0; i < server.args(); i++)
  {
    if (server.argName(i) == "msstateTimer")
    {
      if (server.arg(i) == "true")
        msstateTimer = true;
      else
        msstateTimer = false;
    }
    if (server.argName(i) == "msdisabledTimer")
    {
      if (server.arg(i) == "true")
        msdisabledTimer = true;
      else
        msdisabledTimer = false;
    }
  }
  server.send(201, "text/plain", "created");
}
void BtnNachgussAuto()
{
  for (int i = 0; i < server.args(); i++)
  {
    if (server.argName(i) == "ngstateAuto")
    {
      if (server.arg(i) == "true")
        ngstateAuto = true;
      else
        ngstateAuto = false;
    }
    if (server.argName(i) == "ngdisabledPower")
    {
      if (server.arg(i) == "true")
        ngdisabledPower = true;
      else
        ngdisabledPower = false;
    }
  }
  server.send(201, "text/plain", "created");
}
void BtnNachgussPower()
{
  for (int i = 0; i < server.args(); i++)
  {
    if (server.argName(i) == "ngstatePower")
    {
      if (server.arg(i) == "true")
        ngstatePower = true;
      else
        ngstatePower = false;
    }
    if (server.argName(i) == "ngdisabledPower")
    {
      if (server.arg(i) == "true")
        ngdisabledPower = true;
      else
        ngdisabledPower = false;
    }
  }
  server.send(201, "text/plain", "created");
}
void BtnNachgussTimer()
{
  for (int i = 0; i < server.args(); i++)
  {
    if (server.argName(i) == "ngstateTimer")
    {
      if (server.arg(i) == "true")
        ngstateTimer = true;
      else
        ngstateTimer = false;
    }
    if (server.argName(i) == "ngdisabledTimer")
    {
      if (server.arg(i) == "true")
        ngdisabledTimer = true;
      else
        ngdisabledTimer = false;
    }
  }
  server.send(201, "text/plain", "created");
}
void BtnMiscAgitator()
{
  for (int i = 0; i < server.args(); i++)
  {
    if (server.argName(i) == "mistateAgitator")
    {
      if (server.arg(i) == "true")
        mistateAgitator = true;
      else
        mistateAgitator = false;
    }
    if (server.argName(i) == "midisabledAgitator")
    {
      if (server.arg(i) == "true")
        midisabledAgitator = true;
      else
        midisabledAgitator = false;
    }
  }
  server.send(201, "text/plain", "created");
}
void BtnMiscPump()
{
  for (int i = 0; i < server.args(); i++)
  {
    if (server.argName(i) == "mistatePump")
    {
      if (server.arg(i) == "true")
        mistatePump = true;
      else
        mistatePump = false;
    }
    if (server.argName(i) == "midisabledPump")
    {
      if (server.arg(i) == "true")
        midisabledPump = true;
      else
        midisabledPump = false;
    }
  }
  server.send(201, "text/plain", "created");
}
void BtnMiscRHE()
{
  for (int i = 0; i < server.args(); i++)
  {
    if (server.argName(i) == "mistateRHE")
    {
      if (server.arg(i) == "true")
        mistateRHE = true;
      else
        mistateRHE = false;
    }
    if (server.argName(i) == "midisabledRHE")
    {
      if (server.arg(i) == "true")
        midisabledRHE = true;
      else
        midisabledRHE = false;
    }
  }
  server.send(201, "text/plain", "created");
}
void BtnMiscRew()
{
  TickerMaische.stop();
  TickerSen.config(structMaischePlan[aktMaischeStep].Dauer, 1);
  mstempSoll = structMaischePlan[aktMaischeStep].Temperatur;
  TickerMaische.start();
  ;
  server.send(201, "text/plain", "created");
}
void BtnMiscPause()
{
  for (int i = 0; i < server.args(); i++)
  {
    if (server.argName(i) == "mistatePause")
    {
      ;
      if (server.arg(i) == "true")
        TickerMaische.pause();
      else
        TickerMaische.resume();
    }
  }
  server.send(201, "text/plain", "created");
}
void BtnMiscFor()
{
  if (aktMaischeStep < anzahlRasten)
  {
    TickerMaische.stop();
    aktMaischeStep++;
    TickerSen.config(structMaischePlan[aktMaischeStep].Dauer, 1);
    mstempSoll = structMaischePlan[aktMaischeStep].Temperatur;
    TickerMaische.start();
    ;
  }
  server.send(201, "text/plain", "created");
}

void handleRequestMaische()
{
  server.send(200, "application/json", maischeResponse);
}
void handleRequestHopfen()
{
  server.send(200, "application/json", hopfenResponse);
}

void handleRequestSonstiges()
{
  server.send(200, "application/json", sonstigesResponse);
}

void handleRequestWeb()
{
  StaticJsonDocument<512> webDoc;
  webDoc["msName"] = msName;
  webDoc["mstempIst"] = sensors[0].sens_value;
  webDoc["mstempSoll"] = mstempSoll; //structMaischePlan[aktMaischeStep].Temperatur;
  webDoc["mspowerIst"] = inductionCooker.power;
  webDoc["mspowerSoll"] = mspowerSoll;
  webDoc["ngName"] = ngName;
  webDoc["ngtempIst"] = sensors[1].sens_value;
  webDoc["ngtempSoll"] = ngtempSoll;
  webDoc["ngpowerIst"] = actors[1].power_actor;
  webDoc["ngpowerSoll"] = ngpowerSoll;
  webDoc["msstateAuto"] = msstateAuto;
  webDoc["msstatePower"] = msstatePower;
  webDoc["msstateTimer"] = msstateTimer;
  webDoc["ngstateAuto"] = ngstateAuto;
  webDoc["ngstatePower"] = ngstatePower;
  webDoc["ngstateTimer"] = ngstateTimer;
  webDoc["miNameAgi"] = miNameAgitator;
  webDoc["miNamePump"] = miNamePump;
  webDoc["miNameRHE"] = miNameRHE;
  webDoc["mistateAgitator"] = mistateAgitator;
  webDoc["mistatePump"] = mistatePump;
  webDoc["mistateRHE"] = mistateRHE;
  webDoc["mistateRew"] = mistateRew;
  webDoc["mistatePause"] = mistatePause;
  webDoc["mistateFor"] = mistateFor;
  webDoc["msdisabledPower"] = msdisabledPower;
  webDoc["msdisabledAuto"] = msdisabledAuto;
  webDoc["maischeCounter"] = (structMaischePlan[aktMaischeStep].Dauer / 60000);
  webDoc["maischeName"] = structMaischePlan[aktMaischeStep].Name;
  // webDoc["reminder"] = structHopfenPlan[0].Name;
  webDoc["rezeptname"] = sudName;
  String response = "";
  serializeJson(webDoc, response);
  server.send(200, "application/json", response);
}

void SliderMaischeSudTemp()
{
  for (int i = 0; i < server.args(); i++)
  {
    if (server.argName(i) == "sliderTemp")
    {
      mstempSoll = formatDOT(server.arg(i));
    }
  }
  server.send(201, "text/plain", "created");
}

void SliderMaischeSudPower()
{
  for (int i = 0; i < server.args(); i++)
  {
    if (server.argName(i) == "sliderPower")
    {
      mspowerSoll = formatDOT(server.arg(i));
    }
  }
  server.send(201, "text/plain", "created");
}

void SliderNachgussTemp()
{
  for (int i = 0; i < server.args(); i++)
  {
    if (server.argName(i) == "sliderTemp")
    {
      ngtempSoll = formatDOT(server.arg(i));
    }
  }
  server.send(201, "text/plain", "created");
}

void SliderNachgussPower()
{
  for (int i = 0; i < server.args(); i++)
  {
    if (server.argName(i) == "sliderPower")
    {
      ngpowerSoll = formatDOT(server.arg(i));
    }
  }
  server.send(201, "text/plain", "created");
}

// void handleRequestHopfen()
// {
//   // Serial.print("Send hopfenResponse size: ");
//   // Serial.println(hopfenResponse.length());
//   server.send(200, "application/json", hopfenResponse);
// }

void rebootDevice()
{
  server.send(200, "text/plain", "rebooting...");
  LittleFS.end(); // unmount LittleFS
  ESP.restart();
}

void handleRezeptUp()
{
  ;
  int typeRezept = 0;
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START)
  {
    String filename = "upRezept.json"; //upload.filename;
    if (!filename.startsWith("/"))
      filename = "/" + filename;
    ;
    fsUploadFile = LittleFS.open(filename, "w"); // Open the file for writing in LittleFS (create if it doesn't exist)
    filename = String();
  }
  else if (upload.status == UPLOAD_FILE_WRITE)
  {
    if (fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize); // Write the received bytes to the file
  }
  else if (upload.status == UPLOAD_FILE_END)
  {
    if (fsUploadFile)
    { // If the file was successfully created
      fsUploadFile.close(); // Close the file again
      ;
      server.sendHeader("Location", "/rezept.html"); // Redirect the client to the success page
      server.send(303); // Antwort für Post und Put

      fsUploadFile = LittleFS.open("/upRezept.json", "r");
      DynamicJsonDocument testDoc(8192);
      DeserializationError error = deserializeJson(testDoc, fsUploadFile);
      fsUploadFile.close();

      if (error)
      {
        ;
        if (startBuzzer)
          sendAlarm(4);
        return;
      }
      if (testDoc.containsKey("Global")) // Datenbankversion KBH2
        typeRezept = 1;
      else
        typeRezept = 2;

      server.send(201, "text/plain", "Upload successful");
    }
    else
    {
      server.send(500, "text/plain", "500: couldn't create file");
    }
    if (typeRezept == 1)
      BtnImportKBH2();
    else if (typeRezept == 2)
      BtnImportMMUM();
    LittleFS.remove("/upRezept.json");
  }
}

void handleRequestMS()
{
  StaticJsonDocument<512> doc;
  doc["msName"] = msName;
  doc["sensoraddress"] = SensorAddressToString(sensors[0].sens_address);
  doc["sensoroffset"] = sensors[0].sens_offset;
  doc["ids2"] = inductionCooker.isEnabled;
  doc["ids2delay"] = inductionCooker.delayAfteroff / 1000;
  doc["ids2autotune"] = msPIDAutoTune;
  doc["ids2kp"] = configMS.Kp;
  doc["ids2ki"] = configMS.Ki;
  doc["ids2kd"] = configMS.Kd;
  doc["grafana"] = inductionCooker.setGrafana;
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleRequestMSInd()
{
  String request = server.arg(0);
  String message;

  if (request == "pins")
  {
    int id = server.arg(1).toInt();
    unsigned char pinswitched;
    switch (id)
    {
    case 0:
      pinswitched = inductionCooker.PIN_WHITE;
      break;
    case 1:
      pinswitched = inductionCooker.PIN_YELLOW;
      break;
    case 2:
      pinswitched = inductionCooker.PIN_INTERRUPT;
      break;
    }
    if (isPin(pinswitched))
    {
      message += ((reinterpret_cast<const __FlashStringHelper *>(
# 731 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
                (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "7_WEB.ino" "." "731" "." "303" "\", \"aSM\", @progbits, 1 #"))) = (
# 731 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
                "<option>"
# 731 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
                ); &__pstr__[0];}))
# 731 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
                )));
      message += PinToString(pinswitched);
      message += ((reinterpret_cast<const __FlashStringHelper *>(
# 733 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
                (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "7_WEB.ino" "." "733" "." "304" "\", \"aSM\", @progbits, 1 #"))) = (
# 733 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
                "</option><option disabled>──────────</option>"
# 733 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
                ); &__pstr__[0];}))
# 733 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
                )));
    }

    for (int i = 0; i < numberOfPins; i++)
    {
      if (pins_used[pins[i]] == false)
      {
        message += ((reinterpret_cast<const __FlashStringHelper *>(
# 740 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
                  (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "7_WEB.ino" "." "740" "." "305" "\", \"aSM\", @progbits, 1 #"))) = (
# 740 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
                  "<option>"
# 740 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
                  ); &__pstr__[0];}))
# 740 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
                  )));
        message += pin_names[i];
        message += ((reinterpret_cast<const __FlashStringHelper *>(
# 742 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
                  (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "7_WEB.ino" "." "742" "." "306" "\", \"aSM\", @progbits, 1 #"))) = (
# 742 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
                  "</option>"
# 742 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
                  ); &__pstr__[0];}))
# 742 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
                  )));
      }
      yield();
    }
    goto SendMessage;
  }

SendMessage:
  server.send(200, "text/plain", message);
}

void handleSetMS()
{
  String ms_sensoraddress = sensors[0].getSens_adress_string();
  float ms_sensoroffset = sensors[0].sens_offset;
  bool is_enabled = inductionCooker.isEnabled;
  long ms_ind_delay = inductionCooker.delayAfteroff;
  unsigned char pin_white = inductionCooker.PIN_WHITE;
  unsigned char pin_blue = inductionCooker.PIN_INTERRUPT;
  unsigned char pin_yellow = inductionCooker.PIN_YELLOW;
  bool ms_grafana = inductionCooker.setGrafana;

  for (int i = 0; i < server.args(); i++)
  {

    if (server.argName(i) == "msName")
    {
      server.arg(i).toCharArray(msName, sizeof(msName));
    }
    if (server.argName(i) == "sensoraddress")
    {

      ms_sensoraddress = server.arg(i);
    }
    if (server.argName(i) == "sensoroffset")
    {
      ms_sensoroffset = formatDOT(server.arg(i));
    }
    if (server.argName(i) == "ids2")
    {
      if (server.arg(i) == "true")
        is_enabled = true;
      else
        is_enabled = false;
    }
    if (server.argName(i) == "ids2delay")
    {
      ms_ind_delay = server.arg(i).toInt() * 1000;
    }
    if (server.argName(i) == "ids2kp")
    {
      configMS.Kp = server.arg(i).toDouble();
    }
    if (server.argName(i) == "ids2ki")
    {
      configMS.Ki = server.arg(i).toDouble();
    }
    if (server.argName(i) == "ids2kd")
    {
      configMS.Kd = server.arg(i).toDouble();
    }
    if (server.argName(i) == "ids2white")
    {
      pin_white = StringToPin(server.arg(i));
    }
    if (server.argName(i) == "ids2yellow")
    {
      pin_yellow = StringToPin(server.arg(i));
    }
    if (server.argName(i) == "ids2blue")
    {
      pin_blue = StringToPin(server.arg(i));
    }
    if (server.argName(i) == "grafana")
    {
      if (server.arg(i) == "true")
        ms_grafana = true;
    }
  }
  sensors[0].change(ms_sensoraddress, "MaischeSud", ms_sensoroffset);
  inductionCooker.change(pin_white, pin_yellow, pin_blue, ms_ind_delay, is_enabled, ms_grafana);
  saveConfig();
  server.send(201, "text/plain", "created");
}

void handleSetNG()
{
  char ng_sensoraddress[20] = "";
  float ng_sensoroffset = 0.0;
  char ng_heater[5] = "";
  bool ng_inverted = false;
  bool ng_pwm = false;
  bool ng_grafana = false;
  for (int i = 0; i < server.args(); i++)
  {

    if (server.argName(i) == "ngName")
    {
      server.arg(i).toCharArray(ngName, sizeof(ngName));
    }
    if (server.argName(i) == "sensoraddress")
    {
      server.arg(i).toCharArray(ng_sensoraddress, 20);
    }
    if (server.argName(i) == "sensoroffset")
    {
      ng_sensoroffset = formatDOT(server.arg(i));
    }
    if (server.argName(i) == "heater")
    {
      server.arg(i).toCharArray(ng_heater, 5);
    }
    if (server.argName(i) == "inverted")
    {
      if (server.arg(i) == "true")
        ng_inverted = true;
    }
    if (server.argName(i) == "ngkp")
    {
      configNG.Kp = server.arg(i).toDouble();
    }
    if (server.argName(i) == "ngki")
    {
      configNG.Ki = server.arg(i).toDouble();
    }
    if (server.argName(i) == "ngkd")
    {
      configNG.Kd = server.arg(i).toDouble();
    }
    if (server.argName(i) == "grafana")
    {
      if (server.arg(i) == "true")
        ng_grafana = true;
    }
  }

  sensors[1].change(ng_sensoraddress, "Nachguss", ng_sensoroffset);
  actors[1].change(ng_heater, "Heater", ng_inverted, ng_pwm, ng_grafana);
  saveConfig();
  server.send(201, "text/plain", "created");
}

void handleRequestNG()
{
  StaticJsonDocument<256> doc;
  doc["ngName"] = ngName;
  doc["sensoraddress"] = SensorAddressToString(sensors[1].sens_address);
  doc["sensoroffset"] = sensors[1].sens_offset;
  doc["heater"] = PinToString(actors[1].pin_actor);
  doc["inverted"] = actors[1].getInverted();
  doc["ngautotune"] = ngPIDAutoTune;
  doc["ngkp"] = configNG.Kp;
  doc["ngki"] = configNG.Ki;
  doc["ngkd"] = configNG.Kd;
  doc["grafana"] = actors[1].getGrafana();
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleRequestMI()
{
  StaticJsonDocument<512> doc;

  doc["miNameAgi"] = miNameAgitator;
  doc["miNamePump"] = miNamePump;
  doc["miNameRHE"] = miNameRHE;
  doc["agitator"] = PinToString(actors[0].pin_actor);
  doc["pump"] = PinToString(actors[3].pin_actor);
  doc["rhe"] = PinToString(actors[4].pin_actor);
  doc["agitator_inverted"] = actors[0].getInverted();
  doc["pump_inverted"] = actors[3].getInverted();
  doc["rhe_inverted"] = actors[4].getInverted();
  doc["agitator_pwm"] = actors[0].getPWM();
  doc["pump_pwm"] = actors[3].getPWM();
  doc["rhe_pwm"] = actors[4].getPWM();
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleSetMI()
{
  // char mi_buzzer[5] = "";
  char mi_agitator[5] = "";
  char mi_pump[5] = "";
  char mi_rhe[5] = "";
  bool mi_agitator_inverted = false;
  bool mi_pump_inverted = false;
  bool mi_rhe_inverted = false;
  bool mi_agitator_pwm = false;
  bool mi_pump_pwm = false;
  bool mi_rhe_pwm = false;

  for (int i = 0; i < server.args(); i++)
  {
    if (server.argName(i) == "miNameAgi")
    {
      server.arg(i).toCharArray(miNameAgitator, sizeof(miNameAgitator));
    }
    if (server.argName(i) == "miNamePump")
    {
      server.arg(i).toCharArray(miNamePump, sizeof(miNamePump));
    }
    if (server.argName(i) == "miNameRHE")
    {
      server.arg(i).toCharArray(miNameRHE, sizeof(miNameRHE));
    }
    // if (server.argName(i) == "buzzer")
    // {
    //   server.arg(i).toCharArray(mi_buzzer, 5);
    // }
    if (server.argName(i) == "agitator")
    {
      server.arg(i).toCharArray(mi_agitator, 5);
    }
    if (server.argName(i) == "pump")
    {
      server.arg(i).toCharArray(mi_pump, 5);
    }
    if (server.argName(i) == "rhe")
    {
      server.arg(i).toCharArray(mi_rhe, 5);
    }
    if (server.argName(i) == "agitator_inverted")
    {
      if (server.arg(i) == "true")
        mi_agitator_inverted = true;
    }
    if (server.argName(i) == "pump_inverted")
    {
      if (server.arg(i) == "true")
        mi_pump_inverted = true;
    }
    if (server.argName(i) == "rhe_inverted")
    {
      if (server.arg(i) == "true")
        mi_rhe_inverted = true;
    }
    if (server.argName(i) == "agitator_pwm")
    {
      if (server.arg(i) == "true")
        mi_agitator_pwm = true;
    }
    if (server.argName(i) == "pump_pwm")
    {
      if (server.arg(i) == "true")
        mi_pump_pwm = true;
    }
    if (server.argName(i) == "rhe_pwm")
    {
      if (server.arg(i) == "true")
        mi_rhe_pwm = true;
    }
  }

  actors[0].change(mi_agitator, "Agitator", mi_agitator_inverted, mi_agitator_pwm, false);
  // actors[2].change(mi_buzzer, "Buzzer", false, false, false);
  actors[3].change(mi_pump, "Pump", mi_pump_inverted, mi_pump_pwm, false);
  actors[4].change(mi_rhe, "RHE", mi_rhe_inverted, mi_rhe_pwm, false);
  saveConfig();
  server.send(201, "text/plain", "created");
}

void SetMaischeTable()
{
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, server.arg(0));
  if (error)
  {
    ;
    if (startBuzzer)
      sendAlarm(4);
    return;
  }
  initMaischePlan();

  JsonArray rastenArray = doc.as<JsonArray>();
  anzahlRasten = rastenArray.size();
  if (anzahlRasten > 10)
    anzahlRasten = 10;
  int i = 0;
  for (JsonObject rastenObj : rastenArray)
  {
    if (i < anzahlRasten)
    {
      String tmpName = rastenObj["Rasten"];
      long tmpDauer = rastenObj["Dauer"];
      int tmpTemp = rastenObj["Temperatur"];
      int tmpID = rastenObj["Nr"];

      // structMaischePlan[i].Id = i+1;
      structMaischePlan[i].ID = tmpID;
      structMaischePlan[i].Name = tmpName;
      structMaischePlan[i].Temperatur = tmpTemp;
      structMaischePlan[i].Dauer = tmpDauer * 60 * 1000;
      structMaischePlan[i].Count = 0;
      i++;
    }
  }
  server.send(201, "text/plain", "JSON successful");
  if (LittleFS.exists("/maischeplan.txt"))
    LittleFS.remove("/maischeplan.txt");

  maischeResponse = "";
  serializeJson(doc, maischeResponse);

  File maischeFile = LittleFS.open("/maischeplan.txt", "w");
  if (!maischeFile)
  {
    ;
    ;
  }
  else
  {
    serializeJson(doc, maischeFile);
    maischeFile.close();
  }
}

void SetHopfenTable()
{
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, server.arg(0));
  if (error)
  {
    ;
    if (startBuzzer)
      sendAlarm(4);
    return;
  }
  initMaischePlan();

  JsonArray hopfenArray = doc.as<JsonArray>();
  anzahlHopfen = hopfenArray.size();
  if (anzahlHopfen > 7)
    anzahlHopfen = 7;
  int i = 0;
  for (JsonObject hopfenObj : hopfenArray)
  {
    if (i < anzahlHopfen)
    {

      String tmpName = hopfenObj["Hopfen"];
      float tmpMenge = hopfenObj["Menge"];
      long tmpZeit = hopfenObj["Kochdauer"];
      int tmpID = hopfenObj["Rast"];

      structHopfenPlan[i].Name = tmpName;
      structHopfenPlan[i].ID = tmpID;
      structHopfenPlan[i].Menge = roundf(tmpMenge * 10.0) / 10.0;
      structHopfenPlan[i].Zeit = tmpZeit * 60 * 1000;
      i++;
    }
  }
  server.send(201, "text/plain", "JSON successful");
  if (LittleFS.exists("/hopfenplan.txt"))
    LittleFS.remove("/hopfenplan.txt");

  hopfenResponse = "";
  serializeJson(doc, hopfenResponse);

  File hopfenFile = LittleFS.open("/hopfenplan.txt", "w");
  if (!hopfenFile)
  {
    ;
    ;
  }
  else
  {
    serializeJson(doc, hopfenFile);
    hopfenFile.close();
  }
}

void SetSonstigesTable()
{
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, server.arg(0));
  if (error)
  {
    ;
    if (startBuzzer)
      sendAlarm(4);
    return;
  }
  initMaischePlan();

  JsonArray sonstigesArray = doc.as<JsonArray>();
  anzahlSonstiges = sonstigesArray.size();
  if (anzahlSonstiges > 10)
    anzahlSonstiges = 10;
  int i = 0;
  for (JsonObject sonstigesObj : sonstigesArray)
  {
    if (i < anzahlSonstiges)
    {
      String tmpName = sonstigesObj["Aufgabe"];
      int tmpID = sonstigesObj["Rast"];
      String tmpOption = sonstigesObj["Option"];
      int tmpPower = sonstigesObj["Power"];
      int tmpTemp = sonstigesObj["Temperatur"];
      String tmpZeit = sonstigesObj["Zeitpunkt"];

      structSonstigesPlan[i].Name = tmpName;
      structSonstigesPlan[i].ID = tmpID;
      structSonstigesPlan[i].Option = tmpOption;
      structSonstigesPlan[i].Power = tmpPower;
      structSonstigesPlan[i].Temp = tmpTemp;
      structSonstigesPlan[i].Zeit = tmpZeit;
      i++;
    }
  }
  server.send(201, "text/plain", "JSON successful");
  if (LittleFS.exists("/sonstigesplan.txt"))
    LittleFS.remove("/sonstigesplan.txt");

  sonstigesResponse = "";
  serializeJson(doc, sonstigesResponse);

  File sonstigesFile = LittleFS.open("/sonstigesplan.txt", "w");
  if (!sonstigesFile)
  {
    ;
    ;
  }
  else
  {
    serializeJson(doc, sonstigesFile);
    sonstigesFile.close();
  }
}

void handleRequestStepsCount()
{
  ;
  String message;
  // if (anzahlRasten < 1)
  // {
  //   message += F("<option>");
  //   message += F("0");
  //   message += F("</option><option disabled>──────────</option>");
  // }
  for (int i = 0; i < anzahlRasten; i++)
  {
    message += ((reinterpret_cast<const __FlashStringHelper *>(
# 1187 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
              (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "7_WEB.ino" "." "1187" "." "307" "\", \"aSM\", @progbits, 1 #"))) = (
# 1187 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
              "<option>"
# 1187 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
              ); &__pstr__[0];}))
# 1187 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
              )));
    message += i+1;
    message += ((reinterpret_cast<const __FlashStringHelper *>(
# 1189 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
              (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "7_WEB.ino" "." "1189" "." "308" "\", \"aSM\", @progbits, 1 #"))) = (
# 1189 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
              "</option>"
# 1189 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
              ); &__pstr__[0];}))
# 1189 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
              )));
    yield();
  }
  server.send(200, "text/html", message);
}

void handleRequestActorNames()
{
  ;
  String message;
  message += ((reinterpret_cast<const __FlashStringHelper *>(
# 1199 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "7_WEB.ino" "." "1199" "." "309" "\", \"aSM\", @progbits, 1 #"))) = (
# 1199 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            "<option>"
# 1199 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            ); &__pstr__[0];}))
# 1199 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            )));
  message += msName;
  message += ((reinterpret_cast<const __FlashStringHelper *>(
# 1201 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "7_WEB.ino" "." "1201" "." "310" "\", \"aSM\", @progbits, 1 #"))) = (
# 1201 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            "</option>"
# 1201 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            ); &__pstr__[0];}))
# 1201 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            )));
  message += ((reinterpret_cast<const __FlashStringHelper *>(
# 1202 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "7_WEB.ino" "." "1202" "." "311" "\", \"aSM\", @progbits, 1 #"))) = (
# 1202 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            "<option>"
# 1202 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            ); &__pstr__[0];}))
# 1202 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            )));
  message += ngName;
  message += ((reinterpret_cast<const __FlashStringHelper *>(
# 1204 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "7_WEB.ino" "." "1204" "." "312" "\", \"aSM\", @progbits, 1 #"))) = (
# 1204 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            "</option>"
# 1204 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            ); &__pstr__[0];}))
# 1204 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            )));
  message += ((reinterpret_cast<const __FlashStringHelper *>(
# 1205 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "7_WEB.ino" "." "1205" "." "313" "\", \"aSM\", @progbits, 1 #"))) = (
# 1205 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            "<option>"
# 1205 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            ); &__pstr__[0];}))
# 1205 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            )));
  message += miNameAgitator;
  message += ((reinterpret_cast<const __FlashStringHelper *>(
# 1207 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "7_WEB.ino" "." "1207" "." "314" "\", \"aSM\", @progbits, 1 #"))) = (
# 1207 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            "</option>"
# 1207 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            ); &__pstr__[0];}))
# 1207 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            )));
  message += ((reinterpret_cast<const __FlashStringHelper *>(
# 1208 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "7_WEB.ino" "." "1208" "." "315" "\", \"aSM\", @progbits, 1 #"))) = (
# 1208 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            "<option>"
# 1208 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            ); &__pstr__[0];}))
# 1208 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            )));
  message += miNamePump;
  message += ((reinterpret_cast<const __FlashStringHelper *>(
# 1210 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "7_WEB.ino" "." "1210" "." "316" "\", \"aSM\", @progbits, 1 #"))) = (
# 1210 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            "</option>"
# 1210 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            ); &__pstr__[0];}))
# 1210 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            )));
  message += ((reinterpret_cast<const __FlashStringHelper *>(
# 1211 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "7_WEB.ino" "." "1211" "." "317" "\", \"aSM\", @progbits, 1 #"))) = (
# 1211 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            "<option>"
# 1211 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            ); &__pstr__[0];}))
# 1211 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            )));
  message += miNameRHE;
  message += ((reinterpret_cast<const __FlashStringHelper *>(
# 1213 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "7_WEB.ino" "." "1213" "." "318" "\", \"aSM\", @progbits, 1 #"))) = (
# 1213 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            "</option>"
# 1213 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            ); &__pstr__[0];}))
# 1213 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            )));
  message += ((reinterpret_cast<const __FlashStringHelper *>(
# 1214 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "7_WEB.ino" "." "1214" "." "319" "\", \"aSM\", @progbits, 1 #"))) = (
# 1214 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            "<option>"
# 1214 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            ); &__pstr__[0];}))
# 1214 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            )));
  message += ((reinterpret_cast<const __FlashStringHelper *>(
# 1215 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "7_WEB.ino" "." "1215" "." "320" "\", \"aSM\", @progbits, 1 #"))) = (
# 1215 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            "no_actor"
# 1215 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            ); &__pstr__[0];}))
# 1215 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            )));
  message += ((reinterpret_cast<const __FlashStringHelper *>(
# 1216 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "7_WEB.ino" "." "1216" "." "321" "\", \"aSM\", @progbits, 1 #"))) = (
# 1216 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            "</option>"
# 1216 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino" 3
            ); &__pstr__[0];}))
# 1216 "c:\\Arduino\\git\\BrewDevice\\7_WEB.ino"
            )));
  yield();

  server.send(200, "text/html", message);
}
# 1 "c:\\Arduino\\git\\BrewDevice\\8_CONFIGFILE.ino"
bool loadConfig()
{
  ;
  File configFile = LittleFS.open("/config.txt", "r");
  if (!configFile)
  {
    ;
    ;
    return false;
  }

  size_t size = configFile.size();
  if (size > 2048)
  {
    ;
    ;
    if (startBuzzer)
      sendAlarm(4);
    return false;
  }

  DynamicJsonDocument doc(1536);
  DeserializationError error = deserializeJson(doc, configFile);
  if (error)
  {
    ;
    if (startBuzzer)
      sendAlarm(4);
    return false;
  }

  JsonArray msArray = doc["MS"];
  JsonObject msObj = msArray[0];

  char ms_sensoraddress[20] = "";
  float ms_sensoroffset = 0.0;

  strlcpy(ms_sensoraddress, msObj["ADDRESS"] | "", sizeof(ms_sensoraddress));
  ms_sensoroffset = msObj["OFFSET"] | 0;

  sensors[0].change(ms_sensoraddress, "MaischeSud", ms_sensoroffset);
  ;

  bool ms_grafana = ms_grafana = msObj["GRAF"];
  configMS.Kp = 2;
  configMS.Ki = 1;
  configMS.Kd = 1;
  if (msObj.containsKey("IDS2"))
  {
    inductionStatus = 1;
    strlcpy(msName, msObj["MSNAME"] | "", sizeof(msName));
    String WHITE = msObj["WHITE"];
    String YELLOW = msObj["YELLOW"];
    String BLUE = msObj["BLUE"];
    configMS.Kp = msObj["KP"] | 0;
    configMS.Ki = msObj["KI"] | 0;
    configMS.Kd = msObj["KD"] | 0;
    int ids2Delay = msObj["DELAY"] | 120000 /* Standard Nachlaufzeit nach dem Ausschalten Induktionskochfeld*/;
    inductionCooker.change(StringToPin(WHITE), StringToPin(YELLOW), StringToPin(BLUE), ids2Delay, true, ms_grafana);
    ;
    ;
  }
  else
  {
    inductionStatus = 0;
    ;
  }
  msPID.SetControllerDirection(0);
  msPID.SetOutputLimits(0, 100);
  msPID.SetMode(1);
  msPID.SetTunings(configMS.Kp, configMS.Ki, configMS.Kd);

  JsonArray ngArray = doc["NG"];
  JsonObject ngObj = ngArray[0];

  char ng_sensorsAddress[20] = "\0";
  float ng_sensorsOffset = 0.0;
  char ng_heater[5] = "";
  bool ng_inverted = false;
  bool ng_pwm = false;
  bool ng_grafana = false;
  strlcpy(ng_sensorsAddress, ngObj["ADDRESS"] | "", sizeof(ng_sensorsAddress));
  ng_sensorsOffset = ngObj["OFFSET"] | 0;

  sensors[1].change(ng_sensorsAddress, "Nachguss", ng_sensorsOffset);
  ;

  configNG.Kp = 2;
  configNG.Ki = 1;
  configNG.Kd = 1;

  strlcpy(ngName, ngObj["NGNAME"] | "", sizeof(ngName));
  strlcpy(ng_heater, ngObj["PIN"] | "", sizeof(ng_heater));
  ng_inverted = ngObj["INV"] | false;
  ng_grafana = ngObj["GRAF"] | false;

  ng_pwm = ngObj["PWM"] | false;

  configNG.Kp = msObj["KP"] | 0;
  configNG.Ki = msObj["KI"] | 0;
  configNG.Kd = msObj["KD"] | 0;
  actors[1].change(ng_heater, "Heater", ng_inverted, ng_pwm, ng_grafana);

  ngPID.SetControllerDirection(0);
  ngPID.SetOutputLimits(0, 100);
  ngPID.SetMode(1);
  ngPID.SetTunings(configNG.Kp, configNG.Ki, configNG.Kd);

  JsonArray miArray = doc["MI"];
  JsonObject miObj = miArray[0];

  char mi_agitator[5];
  char mi_pump[5];
  char mi_rhe[5];
  bool mi_agitator_inverted = false;
  bool mi_pump_inverted = false;
  bool mi_rhe_inverted = false;
  bool mi_agitator_pwm = false;
  bool mi_pump_pwm = false;
  bool mi_rhe_pwm = false;

  strlcpy(miNameAgitator, miObj["MINAMEAGI"] | "", 15);
  strlcpy(miNamePump, miObj["MINAMEPUMP"] | "", 15);
  strlcpy(miNameRHE, miObj["MINAMERHE"] | "", 15);

  strlcpy(mi_agitator, miObj["AGI"] | "", sizeof(mi_agitator));
  mi_agitator_inverted = miObj["INVAGI"] | false;
  mi_agitator_pwm = miObj["PWMAGI"] | false;

  actors[0].change(mi_agitator, "Agitator", mi_agitator_inverted, mi_agitator_pwm, false);
  // DEBUG_MSG("Actor Agitator PIN: %s INV: %s GRAF: %s\n", ms_agitator, ms_inverted, ms_grafana);

  strlcpy(mi_pump, miObj["PUMP"] | "", sizeof(mi_pump));
  mi_pump_inverted = miObj["INVPUMP"] | false;
  mi_pump_pwm = miObj["PWMPUMP"] | false;

  actors[3].change(mi_pump, "Pump", mi_pump_inverted, mi_pump_pwm, false);

  strlcpy(mi_rhe, miObj["RHE"] | "", sizeof(mi_rhe));
  mi_rhe_inverted = miObj["INVRHE"] | false;
  mi_rhe_inverted = true;
  mi_rhe_pwm = miObj["PWMRHE"] | false;
  actors[4].change(mi_rhe, "RHE", mi_rhe_inverted, mi_rhe_pwm, false);

  strlcpy(sudName, miObj["SUD"] | "", sizeof(sudName));

  // System Settings
  JsonArray sysArray = doc["Sys"];
  JsonObject sysObj = sysArray[0];

  startMDNS = sysObj["MDNS"] | false;
  strlcpy(nameMDNS, sysObj["NAME"] | "", sizeof(nameMDNS));
  ;

  startBuzzer = sysObj["BUZ"] | false;

  ;

  startDB = sysObj["STARTDB"] | false;
  strlcpy(dbServer, sysObj["DBSERVER"] | "", sizeof(dbServer));
  strlcpy(dbDatabase, sysObj["DB"] | "", sizeof(dbDatabase));
  strlcpy(dbUser, sysObj["DBUSER"] | "", sizeof(dbUser));
  strlcpy(dbPass, sysObj["DBPASS"] | "", sizeof(dbPass));
  upInflux = sysObj["DBUP"] | 15000;

  ;

  SEN_UPDATE = sysObj["UPSEN"] | 5000;
  ACT_UPDATE = sysObj["UPACT"] | 200;
  IND_UPDATE = sysObj["UPIND"] | 200;

  TickerSen.config(SEN_UPDATE, 0);
  TickerAct.config(ACT_UPDATE, 0);
  TickerInd.config(IND_UPDATE, 0);

  if (numberOfSensors > 0)
    TickerSen.start();
  if (numberOfActors > 0)
    TickerAct.start();
  if (inductionCooker.isEnabled)
    TickerInd.start();

  ;
  ;
  ;
  ;

  testmode = sysObj["TEST"] | false;
  ;
  ;
  configFile.close();
  ;
  size_t len = measureJson(doc);
  ;
  int memoryUsed = doc.memoryUsage();
  ;

  if (startBuzzer)
    sendAlarm(1);

  initMaischePlan();
  initHopfenPlan();
  initSonstigesPlan();
  return true;
}

void saveConfigCallback()
{

  if (LittleFS.begin())
  {
    saveConfig();
    shouldSaveConfig = true;
  }
  else
  {
    Serial.println("*** SYSINFO: WiFiManager failed to save MQTT broker IP");
  }
}

bool saveConfig()
{
  ;
  DynamicJsonDocument doc(1024);

  JsonArray msArray = doc.createNestedArray("MS");
  JsonObject msObj = msArray.createNestedObject();
  msObj["ADDRESS"] = sensors[0].getSens_adress_string();
  msObj["OFFSET"] = sensors[0].sens_offset;
  if (inductionCooker.isEnabled)
  {
    inductionStatus = 1;
    msObj["MSNAME"] = msName;
    msObj["IDS2"] = (int)inductionCooker.isEnabled;
    msObj["DELAY"] = inductionCooker.delayAfteroff;
    msObj["WHITE"] = PinToString(inductionCooker.PIN_WHITE);
    msObj["YELLOW"] = PinToString(inductionCooker.PIN_YELLOW);
    msObj["BLUE"] = PinToString(inductionCooker.PIN_INTERRUPT);
    msObj["KP"] = configMS.Kp;
    msObj["KI"] = configMS.Ki;
    msObj["KD"] = configMS.Kd;
    msObj["GRAF"] = (int)inductionCooker.setGrafana;
    ;
  }
  else
  {
    inductionStatus = 0;
    ;
  }

  ;
  ;

  JsonArray ngArray = doc.createNestedArray("NG");
  JsonObject ngObj = ngArray.createNestedObject();

  ngObj["NGNAME"] = ngName;
  ngObj["ADDRESS"] = sensors[1].getSens_adress_string();
  ngObj["OFFSET"] = sensors[1].sens_offset;

  ;

  ngObj["PIN"] = PinToString(actors[1].pin_actor);
  ngObj["INV"] = (int)actors[1].getInverted();
  ngObj["PWM"] = (int)actors[1].getPWM();
  ngObj["KP"] = configNG.Kp;
  ngObj["KI"] = configNG.Ki;
  ngObj["KD"] = configNG.Kd;
  ngObj["GRAF"] = (int)actors[1].getGrafana();
  ;
  ;

  JsonArray miArray = doc.createNestedArray("MI");
  JsonObject miObj = miArray.createNestedObject();

  miObj["MINAMEAGI"] = miNameAgitator;
  miObj["MINAMEPUMP"] = miNamePump;
  miObj["MINAMRHE"] = miNameRHE;
  miObj["AGI"] = PinToString(actors[0].pin_actor);
  miObj["PUMP"] = PinToString(actors[3].pin_actor);
  miObj["RHE"] = PinToString(actors[4].pin_actor);
  miObj["INVAGI"] = (int)actors[0].getInverted();
  miObj["INVPUMP"] = (int)actors[3].getInverted();
  miObj["INVRHE"] = (int)actors[4].getInverted();
  miObj["PWMAGI"] = (int)actors[0].getPWM();
  miObj["PWMPUMP"] = (int)actors[3].getPWM();
  miObj["PWMRHE"] = (int)actors[4].getPWM();
  miObj["SUD"] = sudName;

  ;
  ;
  ;
  ;

  // Write System
  JsonArray sysArray = doc.createNestedArray("Sys");
  JsonObject sysObj = sysArray.createNestedObject();

  sysObj["MDNS"] = (int)startMDNS;
  sysObj["NAME"] = nameMDNS;
  sysObj["BUZ"] = (int)startBuzzer;

  sysObj["STARTDB"] = (int)startDB;
  sysObj["DBSERVER"] = dbServer;
  sysObj["DB"] = dbDatabase;
  sysObj["DBUSER"] = dbUser;
  sysObj["DBPASS"] = dbPass;

  sysObj["UPSEN"] = SEN_UPDATE;
  sysObj["UPACT"] = ACT_UPDATE;
  sysObj["UPIND"] = IND_UPDATE;
  sysObj["UPDB"] = upInflux;
  sysObj["TEST"] = (int)testmode;

  TickerSen.config(SEN_UPDATE, 0);
  TickerAct.config(ACT_UPDATE, 0);
  TickerInd.config(IND_UPDATE, 0);

  if (numberOfSensors > 0)
    TickerSen.start();
  else
    TickerSen.stop();
  if (numberOfActors > 0)
    TickerAct.start();
  else
    TickerAct.stop();
  if (inductionCooker.isEnabled)
    TickerInd.start();

  ;
  ;
  ;
  ;
  ;
  ;
  ;
  ;

  size_t len = measureJson(doc);
  int memoryUsed = doc.memoryUsage();

  if (len > 2048 || memoryUsed > 2500)
  {
    ;
    ;
    ;
    ;
    if (startBuzzer)
      sendAlarm(4);
    return false;
  }

  File configFile = LittleFS.open("/config.txt", "w");
  if (!configFile)
  {
    ;
    ;
    if (startBuzzer)
      sendAlarm(4);
    return false;
  }
  serializeJson(doc, configFile);
  configFile.close();
  ;
  String Network = WiFi.SSID();
  ;
  ;
  ;
  if (startBuzzer)
    sendAlarm(1);
  return true;
}
# 1 "c:\\Arduino\\git\\BrewDevice\\990_tickerCallback.ino"
void tickerSenCallback() // Timer Objekt Sensoren
{
  handleSensors();
  // Test PID
  if (msPIDAutoTune)
    msCalcPID();
  else if (ngPIDAutoTune)
    ngCalcPID();
  if (msstateAuto)
  {
    msIn = sensors[0].sens_value;
    msSet = structMaischePlan[aktMaischeStep].Temperatur;
    mspowerSoll = msOut;
    msPID.Compute();
    ;
  }
}
void tickerActCallback() // Timer Objekt Aktoren
{
  handleActors();
}
void tickerIndCallback() // Timer Objekt Induktion
{
  // Test PID

  ;
  // inductionCooker.newPower = msOut;

  handleInduction();
}
void tickerInfluxDBCallback() // Timer Objekt Influx Datenbank
{
  sendData();
}

void tickerWLANCallback() // Ticker helper function calling Event WLAN Error
{
  ;
  if (TickerWLAN.counter() == 1)
  {
    switch (WiFi.status())
    {
    case 0: // WL_IDLE_STATUS
      ;
      break;
    case 1: // WL_NO_SSID_AVAIL
      ;
      break;
    case 2: // WL_SCAN_COMPLETED
      ;
      break;
    case 3: // WL_CONNECTED
      ;
      break;
    case 4: // WL_CONNECT_FAILED
      ;
      break;
    case 5: // WL_CONNECTION_LOST
      ;
      break;
    case 6: // WL_DISCONNECTED
      ;
      break;
    case 255: // WL_NO_SHIELD
      ;
      break;
    default:
      break;
    }
  }
  // oledDisplay.wlanOK = false;
  WiFi.reconnect();
  if (WiFi.status() == WL_CONNECTED)
  {
    // oledDisplay.wlanOK = true;
  }
  ;
}

void tickerNTPCallback() // Ticker helper function calling Event WLAN Error
{
  timeClient.update();
  Serial.printf("*** SYSINFO: %s\n", timeClient.getFormattedTime().c_str());
}

void tickerMaischeCallback() // Ticker helper function calling Event WLAN Error
{
  if (aktMaischeStep == 0)
    handleTasksStart();

  handleMaische();
}
# 1 "c:\\Arduino\\git\\BrewDevice\\992_MaischePlan.ino"
void readMaischePlan()
{
    File rezMaische = LittleFS.open("/maischeplan.txt", "r");
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, rezMaische);
    if (error)
    {
        ;
        if (startBuzzer)
            sendAlarm(4);
        return;
    }
    initMaischePlan();

    JsonArray rastenArray = doc.as<JsonArray>();
    anzahlRasten = rastenArray.size();
    if (anzahlRasten > 10)
        anzahlRasten = 10;
    int i = 0;
    for (JsonObject rastenObj : rastenArray)
    {
        if (i < anzahlRasten)
        {
            String tmpName = rastenObj["Name"];
            // int tmpID = rastenObj["ID"];
            long tmpDauer = rastenObj["Dauer"];
            int tmpTemp = rastenObj["Temp"];

            structMaischePlan[i].Name = tmpName;
            // structMaischePlan[i].ID = tmpID;
            structMaischePlan[i].Temperatur = tmpTemp;
            structMaischePlan[i].Dauer = tmpDauer * 60 * 1000;
            structMaischePlan[i].Count = 0;
            i++;
        }
    }
    maischeResponse = "";
    serializeJson(doc, maischeResponse);
    rezMaische.close();
}
void handleMaische()
{
    if (aktMaischeStep < anzahlRasten)
        aktMaischeStep++;
    ;
    TickerMaische.stop();
    // handleTasksEnde();
    TickerSen.config(structMaischePlan[aktMaischeStep].Dauer, 1);
    mstempSoll = structMaischePlan[aktMaischeStep].Temperatur;
    // handleTasksStart();
    TickerMaische.start();
}

void handleTasksStart()
{
    for (int i = 0; i < anzahlSonstiges; i++)
    {
        if (structSonstigesPlan[i].ID == aktMaischeStep)
        {
            if (structSonstigesPlan[i].Zeit == "Start")
            {
                ;
            }
        }
    }
}

void handleTasksEnde()
{
    for (int i = 0; i < anzahlSonstiges; i++)
    {
        if (structSonstigesPlan[i].ID == aktMaischeStep)
        {
            if (structSonstigesPlan[i].Zeit == "Ende")
            {
                //DEBUG_MSG("Ticker handleTaskEnde: %d Aufgabe: %s\n", aktMaischeStep, structMaischePlan[i].Name);
            }
        }
    }
}

void readHopfenPlan()
{
    File rezHopfen = LittleFS.open("/hopfenplan.txt", "r");
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, rezHopfen);
    if (error)
    {
        ;
        if (startBuzzer)
            sendAlarm(4);
        return;
    }
    initHopfenPlan();
    JsonArray hopfenArray = doc.as<JsonArray>();
    anzahlHopfen = hopfenArray.size();
    if (anzahlHopfen > 7)
        anzahlHopfen = 7;
    int i = 0;
    for (JsonObject hopfenObj : hopfenArray)
    {
        if (i < anzahlHopfen)
        {
            String tmpName = hopfenObj["Name"];
            int tmpID = hopfenObj["Rast"];
            float tmpMenge = hopfenObj["Menge"];
            long tmpZeit = hopfenObj["Zeit"];
            structHopfenPlan[i].Name = tmpName;
            structHopfenPlan[i].ID = tmpID;
            structHopfenPlan[i].Menge = roundf(tmpMenge * 10.0) / 10.0;
            structHopfenPlan[i].Zeit = tmpZeit * 60 * 1000;
            i++;
        }
    }
    hopfenResponse = "";
    serializeJson(doc, hopfenResponse);
    rezHopfen.close();
}

void readSonstigesPlan()
{
    File rezSonstiges = LittleFS.open("/sonstigesplan.txt", "r");
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, rezSonstiges);
    if (error)
    {
        ;
        if (startBuzzer)
            sendAlarm(4);
        return;
    }
    initSonstigesPlan();
    JsonArray sonstigesArray = doc.as<JsonArray>();
    anzahlSonstiges = sonstigesArray.size();
    if (anzahlSonstiges > 10)
        anzahlSonstiges = 10;
    int i = 0;
    for (JsonObject sonstigesObj : sonstigesArray)
    {
        if (i < anzahlSonstiges)
        {
            String tmpName = sonstigesObj["Name"];
            int tmpID = sonstigesObj["Rast"];
            int tmpPower = sonstigesObj["Power"];
            int tmpTemp = sonstigesObj["Temperatur"];
            String tmpOption = sonstigesObj["Option"];
            // float tmpDauer = sonstigesObj["Dauer"];
            long tmpZeit = sonstigesObj["Zeitpunkt"];
            structSonstigesPlan[i].Name = tmpName;
            structSonstigesPlan[i].ID = tmpID;
            structSonstigesPlan[i].Power = tmpPower;
            structSonstigesPlan[i].Temp = tmpTemp;
            structSonstigesPlan[i].Zeit = tmpZeit;
            structSonstigesPlan[i].Option = tmpOption;
            // structSonstigesPlan[i].Dauer = tmpDauer * 60 * 1000;
            i++;
        }
    }
    sonstigesResponse = "";
    serializeJson(doc, sonstigesResponse);
    rezSonstiges.close();
}

void BtnImportKBH2()
{

    File kbh2File = LittleFS.open("/upRezept.json", "r");
    DynamicJsonDocument dataDoc(1024);
    DynamicJsonDocument doc(8192);
    DeserializationError error = deserializeJson(doc, kbh2File);
    if (error)
    {
        ;
        if (startBuzzer)
            sendAlarm(4);
        return;
    }
    if (LittleFS.exists("/maischeplan.txt"))
        LittleFS.remove("/maischeplan.txt");

    initMaischePlan();

    JsonArray rastenArray = doc["Rasten"];
    anzahlRasten = rastenArray.size();
    if (anzahlRasten > 10)
        anzahlRasten = 10;
    int i = 0;
    for (JsonObject rastenObj : rastenArray)
    {
        if (i < anzahlRasten)
        {
            // rastenObj aus KBH2 JSON
            // dataObj zur Verarbeitung

            JsonObject dataObj = dataDoc.createNestedObject();
            dataObj["Rasten"] = rastenObj["Name"];
            dataObj["Temperatur"] = rastenObj["Temp"];
            dataObj["Dauer"] = rastenObj["Dauer"];
            dataObj["Count"] = 0;

            String tmpName = rastenObj["Name"];
            long tmpDauer = rastenObj["Dauer"];
            int tmpTemp = rastenObj["Temp"];

            structMaischePlan[i].Name = tmpName;
            structMaischePlan[i].Temperatur = tmpTemp;
            structMaischePlan[i].Dauer = tmpDauer * 60 * 1000;
            structMaischePlan[i].Count = 0;

            ;
            i++;
        }
    }

    JsonObject Sud = doc["Sud"];
    String leseDauer = Sud["Kochdauer"];

    strlcpy(sudName, Sud["Sudname"], sizeof(sudName));

    String leseName = "Kochen";
    String leseTemp = "100°C";
    ;
    JsonObject dataObj = dataDoc.createNestedObject();
    dataObj["Rasten"] = "Kochen";
    dataObj["Temperatur"] = "100";
    dataObj["Dauer"] = Sud["Kochdauer"];
    dataObj["Count"] = 0;

    structMaischePlan[i].Name = "Kochen";
    structMaischePlan[i].Temperatur = 100;
    structMaischePlan[i].Dauer = int(Sud["Kochdauer"]) * 60 * 1000;
    structMaischePlan[i].Count = 0;

    maischeResponse = "";
    serializeJson(dataDoc, maischeResponse);

    File maischeFile = LittleFS.open("/maischeplan.txt", "w");
    if (!maischeFile)
    {
        ;
        ;
    }
    else
    {
        serializeJson(dataDoc, maischeFile);
        maischeFile.close();
    }

    size_t len = measureJson(dataDoc);
    int memoryUsed = dataDoc.memoryUsage();

    ;
    ;

    if (LittleFS.exists("/hopfenplan.txt"))
        LittleFS.remove("/hopfenplan.txt");
    initHopfenPlan();
    DynamicJsonDocument hopfenDoc(512);

    JsonArray readArray = doc["Hopfengaben"];
    anzahlHopfen = readArray.size();
    if (anzahlHopfen > 7)
        anzahlHopfen = 7;
    i = 0;
    for (JsonObject readObj : readArray)
    {
        if (i < anzahlHopfen)
        {
            JsonObject hopfenObj = hopfenDoc.createNestedObject();
            String tmpName = readObj["Name"];
            // float tmpMenge = readObj["Menge"];
            float tmpMenge = readObj["erg_Menge"];
            long tmpZeit = readObj["Zeit"];

            hopfenObj["Hopfen"] = tmpName;
            hopfenObj["Rast"] = int(anzahlRasten + 1);
            hopfenObj["Kochdauer"] = tmpZeit;
            hopfenObj["Menge"] = roundf(tmpMenge * 10.0) / 10.0; //menge;

            structHopfenPlan[i].Name = tmpName;
            structHopfenPlan[i].Menge = roundf(tmpMenge * 10.0) / 10.0;
            structHopfenPlan[i].Zeit = tmpZeit * 60 * 1000;

            ;
            i++;
        }
    }
    hopfenResponse = "";
    serializeJson(hopfenDoc, hopfenResponse);

    File hopfenFile = LittleFS.open("/hopfenplan.txt", "w");
    if (!hopfenFile)
    {
        ;
        ;
    }
    else
    {
        serializeJson(hopfenDoc, hopfenFile);
        hopfenFile.close();
    }

    size_t hopfenLen = measureJson(hopfenDoc);
    int hopfenMemoryUsed = hopfenDoc.memoryUsage();

    ;
    ;
    kbh2File.close();
}

void BtnImportMMUM()
{
    File mmumFile = LittleFS.open("/upRezept.json", "r");
    DynamicJsonDocument dataDoc(1024);
    DynamicJsonDocument doc(8192);
    DeserializationError error = deserializeJson(doc, mmumFile);
    if (error)
    {
        ;
        if (startBuzzer)
            sendAlarm(4);
        return;
    }
    if (LittleFS.exists("/maischeplan.txt"))
        LittleFS.remove("/maischeplan.txt");
    initMaischePlan();
    int i = 0;
    if (doc.containsKey("Infusion_Einmaischtemperatur"))
    {
        JsonObject dataObj = dataDoc.createNestedObject();
        dataObj["Name"] = "Einmaischen";
        dataObj["Temp"] = doc["Infusion_Einmaischtemperatur"];
        dataObj["Dauer"] = 10;
        dataObj["Count"] = 0;

        String tmpName = "Einmaischen";
        long tmpDauer = 10;
        int tmpTemp = doc["Infusion_Einmaischtemperatur"];

        structMaischePlan[i].Name = tmpName;
        structMaischePlan[i].Temperatur = tmpTemp;
        structMaischePlan[i].Dauer = tmpDauer * 60 * 1000;
        structMaischePlan[i].Count = 0;
        i++;
    }
    if (doc.containsKey("Abmaischtemperatur"))
    {
        JsonObject dataObj = dataDoc.createNestedObject();
        dataObj["Name"] = "Abmaischen";
        dataObj["Temp"] = doc["Abmaischtemperatur"];
        dataObj["Dauer"] = 1;
        dataObj["Count"] = 0;

        String tmpName = "Abmaischen";
        long tmpDauer = 1;
        int tmpTemp = doc["Abmaischtemperatur"];

        structMaischePlan[i].Name = tmpName;
        structMaischePlan[i].Temperatur = tmpTemp;
        structMaischePlan[i].Dauer = tmpDauer * 60 * 1000;
        structMaischePlan[i].Count = 0;
        i++;
    }
    if (doc.containsKey("Infusion_Rasttemperatur1"))
    {
        JsonObject dataObj = dataDoc.createNestedObject();
        dataObj["Name"] = "Rast1";
        dataObj["Temp"] = doc["Infusion_Rasttemperatur1"];
        dataObj["Dauer"] = doc["Infusion_Rastzeit1"];
        dataObj["Count"] = 0;

        String tmpName = "Rast1";
        int tmpTemp = doc["Infusion_Rasttemperatur1"];
        long tmpDauer = doc["Infusion_Rastzeit1"];

        structMaischePlan[i].Name = tmpName;
        structMaischePlan[i].Temperatur = tmpTemp;
        structMaischePlan[i].Dauer = tmpDauer * 60 * 1000;
        structMaischePlan[i].Count = 0;
        i++;
    }

    if (doc.containsKey("Infusion_Rasttemperatur2"))
    {
        JsonObject dataObj = dataDoc.createNestedObject();
        dataObj["Name"] = "Rast2";
        dataObj["Temp"] = doc["Infusion_Rasttemperatur2"];
        dataObj["Dauer"] = doc["Infusion_Rastzeit2"];
        dataObj["Count"] = 0;

        String tmpName = "Rast2";
        int tmpTemp = doc["Infusion_Rasttemperatur2"];
        long tmpDauer = doc["Infusion_Rastzeit2"];

        structMaischePlan[i].Name = tmpName;
        structMaischePlan[i].Temperatur = tmpTemp;
        structMaischePlan[i].Dauer = tmpDauer * 60 * 1000;
        structMaischePlan[i].Count = 0;
        i++;
    }
    if (doc.containsKey("Infusion_Rasttemperatur3"))
    {
        JsonObject dataObj = dataDoc.createNestedObject();
        dataObj["Name"] = "Rast3";
        dataObj["Temp"] = doc["Infusion_Rasttemperatur3"];
        dataObj["Dauer"] = doc["Infusion_Rastzeit3"];
        dataObj["Count"] = 0;

        String tmpName = "Rast3";
        int tmpTemp = doc["Infusion_Rasttemperatur3"];
        long tmpDauer = doc["Infusion_Rastzeit3"];

        structMaischePlan[i].Name = tmpName;
        structMaischePlan[i].Temperatur = tmpTemp;
        structMaischePlan[i].Dauer = tmpDauer * 60 * 1000;
        structMaischePlan[i].Count = 0;
        i++;
    }
    if (doc.containsKey("Infusion_Rasttemperatur4"))
    {
        JsonObject dataObj = dataDoc.createNestedObject();
        dataObj["Name"] = "Rast4";
        dataObj["Temp"] = doc["Infusion_Rasttemperatur4"];
        dataObj["Dauer"] = doc["Infusion_Rastzeit4"];
        dataObj["Count"] = 0;

        String tmpName = "Rast4";
        int tmpTemp = doc["Infusion_Rasttemperatur4"];
        long tmpDauer = doc["Infusion_Rastzeit4"];

        structMaischePlan[i].Name = tmpName;
        structMaischePlan[i].Temperatur = tmpTemp;
        structMaischePlan[i].Dauer = tmpDauer * 60 * 1000;
        structMaischePlan[i].Count = 0;
        i++;
    }
    if (doc.containsKey("Infusion_Rasttemperatur5"))
    {
        JsonObject dataObj = dataDoc.createNestedObject();
        dataObj["Name"] = "Rast5";
        dataObj["Temp"] = doc["Infusion_Rasttemperatur5"];
        dataObj["Dauer"] = doc["Infusion_Rastzeit5"];
        dataObj["Count"] = 0;

        String tmpName = "Rast5";
        int tmpTemp = doc["Infusion_Rasttemperatur5"];
        long tmpDauer = doc["Infusion_Rastzeit5"];

        structMaischePlan[i].Name = tmpName;
        structMaischePlan[i].Temperatur = tmpTemp;
        structMaischePlan[i].Dauer = tmpDauer * 60 * 1000;
        structMaischePlan[i].Count = 0;
        i++;
    }
    if (doc.containsKey("Infusion_Rasttemperatur6"))
    {
        JsonObject dataObj = dataDoc.createNestedObject();
        dataObj["Name"] = "Rast6";
        dataObj["Temp"] = doc["Infusion_Rasttemperatur6"];
        dataObj["Dauer"] = doc["Infusion_Rastzeit6"];
        dataObj["Count"] = 0;

        String tmpName = "Rast6";
        int tmpTemp = doc["Infusion_Rasttemperatur6"];
        long tmpDauer = doc["Infusion_Rastzeit6"];

        structMaischePlan[i].Name = tmpName;
        structMaischePlan[i].Temperatur = tmpTemp;
        structMaischePlan[i].Dauer = tmpDauer * 60 * 1000;
        structMaischePlan[i].Count = 0;
        i++;
    }
    if (doc.containsKey("Infusion_Rasttemperatur7"))
    {
        JsonObject dataObj = dataDoc.createNestedObject();
        dataObj["Name"] = "Rast7";
        dataObj["Temp"] = doc["Infusion_Rasttemperatur7"];
        dataObj["Dauer"] = doc["Infusion_Rastzeit7"];
        dataObj["Count"] = 0;

        String tmpName = "Rast7";
        int tmpTemp = doc["Infusion_Rasttemperatur7"];
        long tmpDauer = doc["Infusion_Rastzeit7"];

        structMaischePlan[i].Name = tmpName;
        structMaischePlan[i].Temperatur = tmpTemp;
        structMaischePlan[i].Dauer = tmpDauer * 60 * 1000;
        structMaischePlan[i].Count = 0;
        i++;
    }
    if (doc.containsKey("Kochzeit_Wuerze"))
    {
        JsonObject dataObj = dataDoc.createNestedObject();
        dataObj["Name"] = "Kochen";
        dataObj["Temp"] = 100;
        dataObj["Dauer"] = doc["Kochzeit_Wuerze"];
        dataObj["Count"] = 0;

        String tmpName = "Kochen";
        long tmpDauer = doc["Kochzeit_Wuerze"];
        int tmpTemp = 100;

        structMaischePlan[i].Name = tmpName;
        structMaischePlan[i].Temperatur = tmpTemp;
        structMaischePlan[i].Dauer = tmpDauer * 60 * 1000;
        structMaischePlan[i].Count = 0;
        i++;
    }

    maischeResponse = "";
    serializeJson(dataDoc, maischeResponse);

    File maischeFile = LittleFS.open("/maischeplan.txt", "w");
    if (!maischeFile)
    {
        ;
        ;
    }
    else
    {
        serializeJson(dataDoc, maischeFile);
        maischeFile.close();
    }

    size_t len = measureJson(dataDoc);
    int memoryUsed = dataDoc.memoryUsage();

    ;
    ;

    if (LittleFS.exists("/hopfenplan.txt"))
        LittleFS.remove("/hopfenplan.txt");
    initHopfenPlan();
    DynamicJsonDocument hopfenDoc(512);
    i = 0;
    if (doc.containsKey("Hopfen_1_Sorte"))
    {
        JsonObject hopfenObj = hopfenDoc.createNestedObject();
        String tmpName = doc["Hopfen_1_Sorte"];
        float tmpMenge = doc["Hopfen_1_Menge"];
        long tmpZeit = doc["Hopfen_1_Kochzeit"];

        hopfenObj["Name"] = tmpName;
        hopfenObj["Rast"] = anzahlRasten + 1;
        hopfenObj["Zeit"] = tmpZeit;
        hopfenObj["Menge"] = roundf(tmpMenge * 10.0) / 10.0; //menge;

        structHopfenPlan[i].Name = tmpName;
        structHopfenPlan[i].Menge = roundf(tmpMenge * 10.0) / 10.0;
        structHopfenPlan[i].Zeit = tmpZeit * 60 * 1000;
        i++;
    }
    if (doc.containsKey("Hopfen_2_Sorte"))
    {
        JsonObject hopfenObj = hopfenDoc.createNestedObject();
        String tmpName = doc["Hopfen_2_Sorte"];
        float tmpMenge = doc["Hopfen_2_Menge"];
        long tmpZeit = doc["Hopfen_2_Kochzeit"];

        hopfenObj["Name"] = tmpName;
        hopfenObj["Rast"] = anzahlRasten + 1;
        hopfenObj["Zeit"] = tmpZeit;
        hopfenObj["Menge"] = roundf(tmpMenge * 10.0) / 10.0; //menge;

        structHopfenPlan[i].Name = tmpName;
        structHopfenPlan[i].Menge = roundf(tmpMenge * 10.0) / 10.0;
        structHopfenPlan[i].Zeit = tmpZeit * 60 * 1000;
        i++;
    }
    if (doc.containsKey("Hopfen_3_Sorte"))
    {
        JsonObject hopfenObj = hopfenDoc.createNestedObject();
        String tmpName = doc["Hopfen_3_Sorte"];
        float tmpMenge = doc["Hopfen_3_Menge"];
        long tmpZeit = doc["Hopfen_3_Kochzeit"];

        hopfenObj["Name"] = tmpName;
        hopfenObj["Rast"] = anzahlRasten + 1;
        hopfenObj["Zeit"] = tmpZeit;
        hopfenObj["Menge"] = roundf(tmpMenge * 10.0) / 10.0; //menge;

        structHopfenPlan[i].Name = tmpName;
        structHopfenPlan[i].Menge = roundf(tmpMenge * 10.0) / 10.0;
        structHopfenPlan[i].Zeit = tmpZeit * 60 * 1000;
        i++;
    }
    if (doc.containsKey("Hopfen_4_Sorte"))
    {
        JsonObject hopfenObj = hopfenDoc.createNestedObject();
        String tmpName = doc["Hopfen_4_Sorte"];
        float tmpMenge = doc["Hopfen_4_Menge"];
        long tmpZeit = doc["Hopfen_4_Kochzeit"];

        hopfenObj["Name"] = tmpName;
        hopfenObj["Rast"] = anzahlRasten + 1;
        hopfenObj["Zeit"] = tmpZeit;
        hopfenObj["Menge"] = roundf(tmpMenge * 10.0) / 10.0; //menge;

        structHopfenPlan[i].Name = tmpName;
        structHopfenPlan[i].Menge = roundf(tmpMenge * 10.0) / 10.0;
        structHopfenPlan[i].Zeit = tmpZeit * 60 * 1000;
        i++;
    }
    if (doc.containsKey("Hopfen_5_Sorte"))
    {
        JsonObject hopfenObj = hopfenDoc.createNestedObject();
        String tmpName = doc["Hopfen_5_Sorte"];
        float tmpMenge = doc["Hopfen_5_Menge"];
        long tmpZeit = doc["Hopfen_5_Kochzeit"];

        hopfenObj["Name"] = tmpName;
        hopfenObj["Rast"] = anzahlRasten + 1;
        hopfenObj["Zeit"] = tmpZeit;
        hopfenObj["Menge"] = roundf(tmpMenge * 10.0) / 10.0; //menge;

        structHopfenPlan[i].Name = tmpName;
        structHopfenPlan[i].Menge = roundf(tmpMenge * 10.0) / 10.0;
        structHopfenPlan[i].Zeit = tmpZeit * 60 * 1000;
        i++;
    }
    if (doc.containsKey("Hopfen_6_Sorte"))
    {
        JsonObject hopfenObj = hopfenDoc.createNestedObject();
        String tmpName = doc["Hopfen_6_Sorte"];
        float tmpMenge = doc["Hopfen_6_Menge"];
        long tmpZeit = doc["Hopfen_6_Kochzeit"];

        hopfenObj["Name"] = tmpName;
        hopfenObj["Rast"] = anzahlRasten + 1;
        hopfenObj["Zeit"] = tmpZeit;
        hopfenObj["Menge"] = roundf(tmpMenge * 10.0) / 10.0; //menge;

        structHopfenPlan[i].Name = tmpName;
        structHopfenPlan[i].Menge = roundf(tmpMenge * 10.0) / 10.0;
        structHopfenPlan[i].Zeit = tmpZeit * 60 * 1000;
        i++;
    }
    if (doc.containsKey("Hopfen_7_Sorte"))
    {
        JsonObject hopfenObj = hopfenDoc.createNestedObject();
        String tmpName = doc["Hopfen_7_Sorte"];
        float tmpMenge = doc["Hopfen_7_Menge"];
        long tmpZeit = doc["Hopfen_7_Kochzeit"];

        hopfenObj["Name"] = tmpName;
        hopfenObj["Rast"] = anzahlRasten + 1;
        hopfenObj["Zeit"] = tmpZeit;
        hopfenObj["Menge"] = roundf(tmpMenge * 10.0) / 10.0; //menge;

        structHopfenPlan[i].Name = tmpName;
        structHopfenPlan[i].Menge = roundf(tmpMenge * 10.0) / 10.0;
        structHopfenPlan[i].Zeit = tmpZeit * 60 * 1000;
        i++;
    }

    hopfenResponse = "";
    serializeJson(hopfenDoc, hopfenResponse);

    File hopfenFile = LittleFS.open("/hopfenplan.txt", "w");
    if (!hopfenFile)
    {
        ;
        ;
    }
    else
    {
        serializeJson(hopfenDoc, hopfenFile);
        hopfenFile.close();
    }

    size_t hopfenLen = measureJson(hopfenDoc);
    int hopfenMemoryUsed = hopfenDoc.memoryUsage();

    ;
    ;
    mmumFile.close();
}

void initMaischePlan()
{
    struct MaischePlan structMaischePlan[10] = {
        { 1, "", 0, 0, 0},
        { 2, "", 0, 0, 0},
        { 3, "", 0, 0, 0},
        { 4, "", 0, 0, 0},
        { 5, "", 0, 0, 0},
        { 6, "", 0, 0, 0},
        { 7, "", 0, 0, 0},
        { 8, "", 0, 0, 0},
        { 9, "", 0, 0, 0},
        { 10, "", 0, 0, 0}};
}
void initHopfenPlan()
{
    struct HopfenPlan structHopfenPlan[7] = {
        {"", 0, 0, 0.0},
        {"", 0, 0, 0.0},
        {"", 0, 0, 0.0},
        {"", 0, 0, 0.0},
        {"", 0, 0, 0.0},
        {"", 0, 0, 0.0},
        {"", 0, 0, 0.0}};
}

void initSonstigesPlan()
{
    struct SonstigesPlan structSonstigesPlan[10] = {
        {"", 0, "", 0, 0, ""},
        {"", 0, "", 0, 0, ""},
        {"", 0, "", 0, 0, ""},
        {"", 0, "", 0, 0, ""},
        {"", 0, "", 0, 0, ""},
        {"", 0, "", 0, 0, ""},
        {"", 0, "", 0, 0, ""},
        {"", 0, "", 0, 0, ""},
        {"", 0, "", 0, 0, ""},
        {"", 0, "", 0, 0, ""}};
}

// void initAufgaben()
// {
//     struct Aufgaben structAufgaben[anzahlAufgabenMax] = {
//         {"", "", "", ""},
//         {"", "", "", ""},
//         {"", "", "", ""},
//         {"", "", "", ""},
//         {"", "", "", ""},
//         {"", "", "", ""},
//         {"", "", "", ""}};
// }

/*

    "Abmaischtemperatur": "78",

    "Nachguss": "48",

    "Kochzeit_Wuerze": "90", 

    -------> folgende nur fÃ¼r Infusion relevant

    "Infusion_Einmaischtemperatur": 52,

    "Infusion_Rasttemperatur1": "52",

    "Infusion_Rastzeit1": "15",

    "Infusion_Rasttemperatur2": "65",

    "Infusion_Rastzeit2": "60",

    "Infusion_Rasttemperatur3": "",

    "Infusion_Rastzeit3": "",

    "Infusion_Rasttemperatur4": "",

    "Infusion_Rastzeit4": "",

    "Infusion_Rasttemperatur5": "",

    "Infusion_Rastzeit5": "",

    "Infusion_Rasttemperatur6": "",

    "Infusion_Rastzeit6": "",

    "Infusion_Rasttemperatur7": "",

    "Infusion_Rastzeit7": "",

    

    --------> folgende nur fÃ¼r Dekoktionsmaischen relevant

    "Dekoktion_0_Volumen": "",

    "Dekoktion_0_Temperatur_ist": "",

    "Dekoktion_0_Temperatur_resultierend": "",

    "Dekoktion_0_Rastzeit": "",

    "Dekoktion_1_Volumen": "",

    "Dekoktion_1_Form": "",

    "Dekoktion_1_Temperatur_resultierend": "",

    "Dekoktion_1_Rastzeit": "",

    "Dekoktion_1_Teilmaische_Temperatur": "",

    "Dekoktion_1_Teilmaische_Rastzeit": "",

    "Dekoktion_1_Teilmaische_Kochzeit": "",

    "Dekoktion_2_Volumen": "",

    "Dekoktion_2_Form": "",

    "Dekoktion_2_Temperatur_resultierend": "",

    "Dekoktion_2_Rastzeit": "",

    "Dekoktion_2_Teilmaische_Temperatur": "",

    "Dekoktion_2_Teilmaische_Rastzeit": "",

    "Dekoktion_2_Teilmaische_Kochzeit": "",

    "Dekoktion_3_Volumen": "",

    "Dekoktion_3_Form": "",

    "Dekoktion_3_Temperatur_resultierend": "",

    "Dekoktion_3_Rastzeit": "",

    "Dekoktion_3_Teilmaische_Temperatur": "",

    "Dekoktion_3_Teilmaische_Rastzeit": "",

    "Dekoktion_3_Teilmaische_Kochzeit": "",



    "Abmaischtemperatur": "78",

    "Nachguss": "48",

    "Kochzeit_Wuerze": "90",

    

    ------> VWH = Vorderwuerzehopfen

    "Hopfen_VWH_1_Sorte": "",   

    "Hopfen_VWH_1_Menge": "",

    "Hopfen_VWH_1_alpha": "",

    "Hopfen_VWH_2_Sorte": "",

    "Hopfen_VWH_2_Menge": "",

    "Hopfen_VWH_2_alpha": "",

    "Hopfen_VWH_3_Sorte": "",

    "Hopfen_VWH_3_Menge": "",

    "Hopfen_VWH_3_alpha": "",

    "Hopfen_1_Sorte": "Fuggles, Pellets",

    "Hopfen_1_Menge": "280",

    "Hopfen_1_alpha": "4.1",

    "Hopfen_1_Kochzeit": "70",

    "Hopfen_2_Sorte": "Fuggles, Pellets",

    "Hopfen_2_Menge": "100",

    "Hopfen_2_alpha": "4.1",

    "Hopfen_2_Kochzeit": "15",

    "Hopfen_3_Sorte": "Fuggles, Pellets",

    "Hopfen_3_Menge": "70",

    "Hopfen_3_alpha": "4.1",

    "Hopfen_3_Kochzeit": "Whirlpool",

    "Hopfen_4_Sorte": "",

    "Hopfen_4_Menge": "",

    "Hopfen_4_alpha": "",

    "Hopfen_4_Kochzeit": "",

    "Hopfen_5_Sorte": "",

    "Hopfen_5_Menge": "",

    "Hopfen_5_alpha": "",

    "Hopfen_5_Kochzeit": "",

    "Hopfen_6_Sorte": "",

    "Hopfen_6_Menge": "",

    "Hopfen_6_alpha": "",

    "Hopfen_6_Kochzeit": "",

    "Hopfen_7_Sorte": "",

    "Hopfen_7_Menge": "",

    "Hopfen_7_alpha": "",

    "Hopfen_7_Kochzeit": "",

*/
# 1 "c:\\Arduino\\git\\BrewDevice\\993_PID.ino"
void msAutoTune()
{
    server.send(201, "text/plain", "created");
    msPIDAutoTune = true;
    msSet = 64;
    configMS.Kp = 2;
    configMS.Ki = 1;
    configMS.Kd = 2;

    aTuneStep = 50;
    aTuneNoise = 1;
    aTuneStartValue = 50;
    aTuneLookBack = 20;
    msOut = aTuneStartValue;
    outputStart = 5;
    ;
    if (!testmode)
        msIn = sensors[0].sens_value;
    else
    {
        msIn = 10;

        for (byte i = 0; i < 50; i++)
        {
            theta[i] = outputStart;
        }
        modelTime = 0;
    }

    msPID.SetTunings(configMS.Kp, configMS.Ki, configMS.Kd);
    msTune.SetNoiseBand(aTuneNoise);
    msTune.SetOutputStep(aTuneStep);
    msTune.SetLookbackSec((int)aTuneLookBack);
    msTune.SetControlType(1);
    server.send(200, "text/plain", "AutoTune");
}
void msCalcPID()
{
    if (!testmode)
        msIn = sensors[0].sens_value;
    else
    {
        unsigned long now = millis();
        theta[30] = msOut;
        if (now >= modelTime)
        {
            modelTime += 100;
            DoModel();
        }
    }

    if (msTune.Runtime())
    {
        // complete
        msSaveAutoTune();
        msEndAutoTune();
    }
    else
    {
        if (!testmode)
            inductionCooker.newPower = msOut;

        Serial.print("setpoint: ");
        Serial.print(msSet);
        Serial.print(" ");
        Serial.print("input: ");
        Serial.print(msIn);
        Serial.print(" ");
        Serial.print("output: ");
        Serial.print(msOut);
        Serial.print(" ");
        Serial.print("kp: ");
        Serial.print(msTune.GetKp());
        Serial.print(" ");
        Serial.print("ki: ");
        Serial.print(msTune.GetKi());
        Serial.print(" ");
        Serial.print("kd: ");
        Serial.print(msTune.GetKd());
        Serial.print(" ");
        Serial.print("Type: ");
        Serial.print(msTune.GetControlType());
        Serial.print(" ");
        Serial.print("LookBack: ");
        Serial.print(msTune.GetLookbackSec());
        Serial.print(" ");
        Serial.print("Noise: ");
        Serial.print(msTune.GetNoiseBand());
        Serial.print(" ");
        Serial.print("OutStep: ");
        Serial.print(msTune.GetOutputStep());
        Serial.println();
    }
}

void msCancelAutoTune(void)
{
    msTune.Cancel();
    msEndAutoTune();
    server.send(200, "text/plain", "AutoTune cancled");
}

void msEndAutoTune(void)
{
    msPIDAutoTune = false;
    msPID.SetMode(1);
}

void msSaveAutoTune(void)
{
    ;
    configMS.Kp = msTune.GetKp();
    configMS.Ki = msTune.GetKi();
    configMS.Kd = msTune.GetKd();
    msPID.SetTunings(configMS.Kp, configMS.Ki, configMS.Kd);
    ;
    saveConfig();
}
void DoModel()
{
    //cycle the dead time
    for (byte i = 0; i < 49; i++)
    {
        theta[i] = theta[i + 1];
    }
    //compute the input
    msIn = (kpmodel / taup) * (theta[0] - outputStart) + msIn * (1 - 1 / taup) + ((float)random(-10, 10)) / 100;
}

void ngAutoTune()
{
    server.send(201, "text/plain", "created");
    ngPIDAutoTune = true;
    ngSet = 64;
    configNG.Kp = 2;
    configNG.Ki = 1;
    configNG.Kd = 2;

    aTuneStep = 50;
    aTuneNoise = 1;
    aTuneStartValue = 50;
    aTuneLookBack = 20;
    ngOut = aTuneStartValue;
    outputStart = 5;
    ;
    if (!testmode)
        ngIn = sensors[1].sens_value;
    else
    {
        ngIn = 10;
        for (byte i = 0; i < 50; i++)
        {
            theta[i] = outputStart;
        }
        modelTime = 0;
    }

    ngPID.SetTunings(configNG.Kp, configNG.Ki, configNG.Kd);
    ngTune.SetNoiseBand(aTuneNoise);
    ngTune.SetOutputStep(aTuneStep);
    ngTune.SetLookbackSec((int)aTuneLookBack);
    ngTune.SetControlType(1);
    server.send(200, "text/plain", "AutoTune NG");
}
void ngCalcPID()
{
    if (!testmode)
        ngIn = sensors[1].sens_value;
    else
    {
        unsigned long now = millis();
        theta[30] = ngOut;
        if (now >= modelTime)
        {
            modelTime += 100;
            ngDoModel();
        }
    }

    if (ngTune.Runtime())
    {
        // complete
        ngSaveAutoTune();
        ngEndAutoTune();
    }
    else
    {
        if (!testmode)
            analogWrite(actors[1].pin_actor, ngOut);

        Serial.print("setpoint: ");
        Serial.print(ngSet);
        Serial.print(" ");
        Serial.print("input: ");
        Serial.print(ngIn);
        Serial.print(" ");
        Serial.print("output: ");
        Serial.print(ngOut);
        Serial.print(" ");
        Serial.print("kp: ");
        Serial.print(ngTune.GetKp());
        Serial.print(" ");
        Serial.print("ki: ");
        Serial.print(ngTune.GetKi());
        Serial.print(" ");
        Serial.print("kd: ");
        Serial.print(ngTune.GetKd());
        Serial.print(" ");
        Serial.print("Type: ");
        Serial.print(ngTune.GetControlType());
        Serial.print(" ");
        Serial.print("LookBack: ");
        Serial.print(ngTune.GetLookbackSec());
        Serial.print(" ");
        Serial.print("Noise: ");
        Serial.print(ngTune.GetNoiseBand());
        Serial.print(" ");
        Serial.print("OutStep: ");
        Serial.print(ngTune.GetOutputStep());
        Serial.println();
    }
}

void ngCancelAutoTune(void)
{
    ngTune.Cancel();
    ngEndAutoTune();
    server.send(200, "text/plain", "AutoTune NG cancled");
}

void ngEndAutoTune(void)
{
    ngPIDAutoTune = false;
    ngPID.SetMode(1);
}

void ngSaveAutoTune(void)
{
    ;
    configNG.Kp = ngTune.GetKp();
    configNG.Ki = ngTune.GetKi();
    configNG.Kd = ngTune.GetKd();
    ngPID.SetTunings(configNG.Kp, configNG.Ki, configNG.Kd);
    ;
    saveConfig();
}

void ngDoModel()
{
    //cycle the dead time
    for (byte i = 0; i < 49; i++)
    {
        theta[i] = theta[i + 1];
    }
    //compute the input
    ngIn = (kpmodel / taup) * (theta[0] - outputStart) + ngIn * (1 - 1 / taup) + ((float)random(-10, 10)) / 100;
}
# 1 "c:\\Arduino\\git\\BrewDevice\\995_Hysterese.ino"
uint16_t getOutputLevel( uint16_t inputLevel ) {
  // https://forum.arduino.cc/index.php?topic=526806.0

  // adjust these 4 constants to suit your application

  // ========================================
  // margin sets the 'stickyness' of the hysteresis or the relucatance to leave the current state.
  // It is measured in units of the the input level. As a guide it is a few percent of the
  // difference between two end points. Don't make the margin too wide or ranges may overlap.
  const uint16_t margin = 10 ; //  +/- 10

  // set the number of output levels. These are numbered starting from 0.
  const uint16_t numberOfLevelsOutput = 10 ; // 0..9


  // define input to output conversion table/formula by specifying endpoints of the levels.
  // the number of end points is equal to the number of output levels plus one.
  // in the example below, output level 0 results from an input of between 0 and 112.
  // 1 results from an input of between 113 and 212 etc.
  const uint16_t endPointInput[ numberOfLevelsOutput + 1 ] = { 0, 112, 212, 312, 412, 512, 612, 712, 812, 912, 1023 } ;

  // initial output level (usually zero)
  const uint16_t initialOutputLevel = 0 ;
  // ========================================


  // the current output level is retained for the next calculation.
  // Note: initial value of a static variable is set at compile time.
  static uint16_t currentOutputLevel = initialOutputLevel ;

  // get lower and upper bounds for currentOutputLevel
  uint16_t lb = endPointInput[ currentOutputLevel ] ;
  if ( currentOutputLevel > 0 ) lb -= margin ; // subtract margin

  uint16_t ub = endPointInput[ currentOutputLevel + 1 ] ;
  if ( currentOutputLevel < numberOfLevelsOutput ) ub += margin ; // add margin

  // now test if input is between the outer margins for current output value
  if ( inputLevel < lb || inputLevel > ub ) {
    // determine new output level by scanning endPointInput array
    uint16_t i;
    for ( i = 0 ; i < numberOfLevelsOutput ; i++ ) {
      if ( inputLevel >= endPointInput[ i ] && inputLevel <= endPointInput[ i + 1 ] ) break ;
    }
    currentOutputLevel = i ;
  }
  return currentOutputLevel ;
}
# 1 "c:\\Arduino\\git\\BrewDevice\\996_PWM.ino"
float UpdatePWM(float tempIST, float tempSOLL)
{
    float T_diff = tempSOLL - tempIST;
    float heaterPWM = 0.0;
    int heaterProzent = 0;
    int maxPWM = 64;

    if (T_diff >= 3)
    {
        heaterPWM = maxPWM * 1;
        heaterProzent = 100;
    }
    else if (T_diff < 3 && T_diff >= 2.5)
    {
        heaterPWM = maxPWM * 0.9;
        heaterProzent = 90;
    }
    else if (T_diff < 2.5 && T_diff >= 2.0)
    {
        heaterPWM = maxPWM * 0.8;
        heaterProzent = 80;
    }
    else if (T_diff < 2.0 && T_diff >= 1.5)
    {
        heaterPWM = maxPWM * 0.7;
        heaterProzent = 70;
    }
    else if (T_diff < 1.5 && T_diff >= 1.25)
    {
        heaterPWM = maxPWM * 0.6;
        heaterProzent = 60;
    }
    else if (T_diff < 1.25 && T_diff >= 1.0)
    {
        heaterPWM = maxPWM * 0.5;
        heaterProzent = 50;
    }
    else if (T_diff < 1.0 && T_diff >= 0.8)
    {
        heaterPWM = maxPWM * 0.4;
        heaterProzent = 40;
    }
    else if (T_diff < 0.8 && T_diff >= 0.6)
    {
        heaterPWM = maxPWM * 0.3;
        heaterProzent = 30;
    }
    else if (T_diff < 0.6 && T_diff >= 0.4)
    {
        heaterPWM = maxPWM * 0.2;
        heaterProzent = 20;
    }
    else if (T_diff < 0.4 && T_diff >= 0.1)
    {
        heaterPWM = maxPWM * 0.1;
        heaterProzent = 10;
    }

    ;
    // analogWrite(actors[ID_HEATER].pin_actor, HeaterPWM);
    return heaterPWM;
}

    // if (T_diff >= 40)
    // {
    //     HeaterPWM = 64; //full voltage;
    //     heaterProzent = 100;
    // }
    // else if (T_diff < 40 && T_diff >= 30)
    // {
    //     HeaterPWM = 54; //full voltage;
    //     heaterProzent = 85;
    // }
    // else if (T_diff < 30 && T_diff >= 20)
    // {
    //     HeaterPWM = 48; //60%
    //     heaterProzent = 75;
    // }
    // else if (T_diff < 20 && T_diff >= 10)
    // {
    //     HeaterPWM = 38; //50%
    //     heaterProzent = 50;
    // }
    // else if (T_diff < 10 && T_diff >= 2)
    // {
    //     HeaterPWM = 30; //30%
    //     heaterProzent = 30;
    // }
    // else if (T_diff < 2)
    // {
    //     HeaterPWM = 12; //0%
    //     heaterProzent = 0;
    // }
# 1 "c:\\Arduino\\git\\BrewDevice\\99_PINMAP.ino"
/* PINMAP

  0   D3        OnewWire

  1

  2   D4

  3

  4   D2        OLED display (optional)

  5   D1        OLED display (optional)

  6

  7

  8

  9   UNUSED

  10

  11

  12  D6

  13  D7

  14  D5

  15  D8

  16  D0

*/
# 1 "c:\\Arduino\\git\\BrewDevice\\9_SYSTEM.ino"
void millis2wait(const int &value)
{
  unsigned long pause = millis();
  while (millis() < pause + value)
  {
    yield(); //wait approx. [period] ms
  }
}

// Prüfe WebIf Eingaben
float formatDOT(String str)
{
  str.replace(',', '.');
  if (isValidFloat(str))
    return str.toFloat();
  else
    return 0;
}

bool isValidInt(const String &str)
{
  for (int i = 0; i < str.length(); i++)
  {
    if (isdigit(str.charAt(i)))
      continue;
    if (str.charAt(i) == '.')
      return false;
    return false;
  }
  return true;
}

bool isValidFloat(const String &str)
{
  for (int i = 0; i < str.length(); i++)
  {
    if (i == 0)
    {
      if (str.charAt(i) == '-')
        continue;
    }
    if (str.charAt(i) == '.')
      continue;
    if (isdigit(str.charAt(i)))
      continue;
    return false;
  }
  return true;
}

bool isValidDigit(const String &str)
{
  for (int i = 0; i < str.length(); i++)
  {
    if (str.charAt(i) == '.')
      continue;
    if (isdigit(str.charAt(i)))
      continue;
    return false;
  }
  return true;
}

void checkChars(char *input)
{
  char *output = input;
  int j = 0;
  for (int i = 0; i < strlen(input); i++)
  {
    if (input[i] != ' ' && input[i] != '\n' && input[i] != '\r') // Suche nach Leerzeichen und CR LF
      output[j] = input[i];
    else
      j--;

    j++;
  }
  output[j] = '\0';
  *input = *output;
  return;
}

void setTicker()
{
  // Ticker Objekte
  TickerSen.config(tickerSenCallback, SEN_UPDATE, 0);
  TickerAct.config(tickerActCallback, ACT_UPDATE, 0);
  TickerInd.config(tickerIndCallback, IND_UPDATE, 0);
  TickerWLAN.config(tickerWLANCallback, 20000 /* für Ticker Objekt WLAN in ms*/, 0);
  TickerNTP.config(tickerNTPCallback, 60 * 60 * 1000 /* Aktualisierung NTP in ms*/, 0);
  TickerInfluxDB.config(tickerInfluxDBCallback, upInflux, 0);
  TickerMaische.config(tickerMaischeCallback, structMaischePlan[aktMaischeStep].Dauer, 1);
  TickerWLAN.stop();
  TickerMaische.stop();
}

void setMDNS()
{
  if (startMDNS && nameMDNS[0] != '\0' && WiFi.status() == WL_CONNECTED)
    {
      if (mdns.begin(nameMDNS))
        Serial.printf("*** SYSINFO: mDNS gestartet als %s verbunden an %s Time: %s RSSI=%d\n", nameMDNS, WiFi.localIP().toString().c_str(), timeClient.getFormattedTime().c_str(), WiFi.RSSI());
      else
        Serial.printf("*** SYSINFO: Fehler Start mDNS! IP Adresse: %s Time: %s RSSI: %d\n", WiFi.localIP().toString().c_str(), timeClient.getFormattedTime().c_str(), WiFi.RSSI());
    }
}

void setLog()
{
  if (LittleFS.exists("/log1.txt")) // WebUpdate Zertifikate
    {
      fsUploadFile = LittleFS.open("/log1.txt", "r");
      String line;
      while (fsUploadFile.available())
      {
        line = char(fsUploadFile.read());
      }
      fsUploadFile.close();
      Serial.printf("*** SYSINFO: Update Index Anzahl Versuche %s\n", line.c_str());
      LittleFS.remove("/log1.txt");
    }
    if (LittleFS.exists("/log2.txt")) // WebUpdate Index
    {
      fsUploadFile = LittleFS.open("/log2.txt", "r");
      String line;
      while (fsUploadFile.available())
      {
        line = char(fsUploadFile.read());
      }
      fsUploadFile.close();
      Serial.printf("*** SYSINFO: Update Zertifikate Anzahl Versuche %s\n", line.c_str());
      LittleFS.remove("/log2.txt");
    }
    if (LittleFS.exists("/log3.txt")) // WebUpdate Firmware
    {
      fsUploadFile = LittleFS.open("/log3.txt", "r");
      String line;
      while (fsUploadFile.available())
      {
        line = char(fsUploadFile.read());
      }
      fsUploadFile.close();
      Serial.printf("*** SYSINFO: Update Firmware Anzahl Versuche %s\n", line.c_str());
      LittleFS.remove("/log3.txt");
      alertState = true;
    }
}
void checkSummerTime()
{
  time_t rawtime = timeClient.getEpochTime();
  struct tm *ti;
  ti = localtime(&rawtime);
  int year = ti->tm_year + 1900;
  int month = ti->tm_mon + 1;
  int day = ti->tm_mday;
  int hour = ti->tm_hour;
  int tzHours = 1; // UTC: 0 MEZ: 1
  int x1, x2, x3, lastyear;
  int lasttzHours;
  if (month < 3 || month > 10)
  {
    timeClient.setTimeOffset(3600);
    return;
  }
  if (month > 3 && month < 10)
  {
    timeClient.setTimeOffset(7200);
    return;
  }
  if (year != lastyear || tzHours != lasttzHours)
  {
    x1 = 1 + tzHours + 24 * (31 - (5 * year / 4 + 4) % 7);
    x2 = 1 + tzHours + 24 * (31 - (5 * year / 4 + 1) % 7);
    lastyear = year;
    lasttzHours = tzHours;
  }
  x3 = hour + 24 * day;
  if (month == 3 && x3 >= x1 || month == 10 && x3 < x2)
  {
    timeClient.setTimeOffset(7200);
    return;
  }
  else
  {
    timeClient.setTimeOffset(3600);
    return;
  }
}

String decToHex(unsigned char decValue, unsigned char desiredStringLength)
{
  String hexString = String(decValue, 16);
  while (hexString.length() < desiredStringLength)
    hexString = "0" + hexString;

  return "0x" + hexString;
}

unsigned char convertCharToHex(char ch)
{
  unsigned char returnType;
  switch (ch)
  {
  case '0':
    returnType = 0;
    break;
  case '1':
    returnType = 1;
    break;
  case '2':
    returnType = 2;
    break;
  case '3':
    returnType = 3;
    break;
  case '4':
    returnType = 4;
    break;
  case '5':
    returnType = 5;
    break;
  case '6':
    returnType = 6;
    break;
  case '7':
    returnType = 7;
    break;
  case '8':
    returnType = 8;
    break;
  case '9':
    returnType = 9;
    break;
  case 'A':
    returnType = 10;
    break;
  case 'B':
    returnType = 11;
    break;
  case 'C':
    returnType = 12;
    break;
  case 'D':
    returnType = 13;
    break;
  case 'E':
    returnType = 14;
    break;
  case 'F':
    returnType = 15;
    break;
  default:
    returnType = 0;
    break;
  }
  return returnType;
}

void sendAlarm(const uint8_t &setAlarm)
{
  if (!startBuzzer)
    return;
  switch (setAlarm)
  {
  case 1:
    tone(PIN_BUZZER, 440, 50);
    delay(150);
    tone(PIN_BUZZER, 660, 50);
    delay(150);
    tone(PIN_BUZZER, 880, 50);
    break;
  case 2:
    tone(PIN_BUZZER, 880, 50);
    delay(150);
    tone(PIN_BUZZER, 660, 50);
    delay(150);
    tone(PIN_BUZZER, 440, 50);
    break;
  case 3:
    digitalWrite(PIN_BUZZER, 0x1);
    delay(200);
    digitalWrite(PIN_BUZZER, 0x0);
    break;
  case 4:
    for (int i = 0; i < 20; i++)
    {
      tone(PIN_BUZZER, 880, 50);
      delay(150);
      tone(PIN_BUZZER, 440, 50);
      delay(150);
    }
    millis2wait(1000);
    break;
  default:
    break;
  }
}

/*

void checkForTask(String value)

{

  // String retName = "";

  char delimiter[] = "#:";



  // char *taskName;

  // char *taskValue;

  // char *taskTime;

  int index = 0;

  int count = 0;

  while (value.indexOf(':', index) > 0)

  {

    count++;

    index = value.indexOf(':', index);

  }

  

  if (value.charAt(0) == '#' && count == 2)

  {

     structAufgaben[anzahlAufgaben].Aufgabe = value.substring(1, value.indexOf(':'));        //strtok(value, delimiter);

     String tmp = value.substring(value.indexOf(':')+1);

     structAufgaben[anzahlAufgaben].Wert = tmp.substring(0, value.indexOf(':')); // strtok(NULL, delimiter);

     structAufgaben[anzahlAufgaben].Zeit  = tmp.substring(tmp.indexOf(':')+1); // strtok(NULL, delimiter);



     DEBUG_MSG("Task # %d: Aufgabe: %s Wert: %s Zeit: %s\n", anzahlAufgaben, 

                structAufgaben[anzahlAufgaben].Aufgabe, structAufgaben[anzahlAufgaben].Wert, structAufgaben[anzahlAufgaben].Zeit);

     anzahlAufgaben++;

  }

  // return retName;

}

*/
# 1 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
void replyToCLient(int msg_type = 0, const char *msg = "")
{
  server.sendHeader("Access-Control-Allow-Origin", "*");
  switch (msg_type)
  {
  case OK:
    server.send(200, (reinterpret_cast<const __FlashStringHelper *>("text/plain")), "");
    break;
  case CUSTOM:
    server.send(200, (reinterpret_cast<const __FlashStringHelper *>("text/plain")), msg);
    break;
  case NOT_FOUND:
    server.send(404, (reinterpret_cast<const __FlashStringHelper *>("text/plain")), msg);
    break;
  case BAD_REQUEST:
    server.send(400, (reinterpret_cast<const __FlashStringHelper *>("text/plain")), msg);
    break;
  case ERROR:
    server.send(500, (reinterpret_cast<const __FlashStringHelper *>("text/plain")), msg);
    break;
  }
}

void replyOK()
{
  replyToCLient(OK, "");
}

void handleGetEdit()
{
  server.sendHeader(
# 31 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
                   (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "FSBrowser.ino" "." "31" "." "322" "\", \"aSM\", @progbits, 1 #"))) = (
# 31 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                   "Content-Encoding"
# 31 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
                   ); &__pstr__[0];}))
# 31 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                                           , "gzip");
  server.send_P(200, "text/html", edit_htm_gz, sizeof(edit_htm_gz));
}

void handleStatus()
{
  FSInfo fs_info;
  LittleFS.info(fs_info);
  String json;
  json.reserve(128);
  json = "{\"type\":\"Filesystem\", \"isOk\":";
  json += 
# 42 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
         (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "FSBrowser.ino" "." "42" "." "323" "\", \"aSM\", @progbits, 1 #"))) = (
# 42 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
         "\"true\", \"totalBytes\":\""
# 42 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
         ); &__pstr__[0];}))
# 42 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                                            ;
  json += fs_info.totalBytes;
  json += 
# 44 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
         (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "FSBrowser.ino" "." "44" "." "324" "\", \"aSM\", @progbits, 1 #"))) = (
# 44 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
         "\", \"usedBytes\":\""
# 44 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
         ); &__pstr__[0];}))
# 44 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                                     ;
  json += fs_info.usedBytes;
  json += "\"";
  json += 
# 47 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
         (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "FSBrowser.ino" "." "47" "." "325" "\", \"aSM\", @progbits, 1 #"))) = (
# 47 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
         ",\"unsupportedFiles\":\"\"}"
# 47 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
         ); &__pstr__[0];}))
# 47 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                                            ;
  server.send(200, "application/json", json);
}

void handleFileList()
{
  if (!server.hasArg("dir"))
  {
    server.send(500, "text/plain", "BAD ARGS");
    return;
  }
  String path = server.arg("dir");
  Dir dir = LittleFS.openDir(path);
  path = String();

  String output = "[";
  while (dir.next())
  {
    File entry = dir.openFile("r");
    if (output != "[")
    {
      output += ',';
    }
    bool isDir = false;
    output += "{\"type\":\"";
    output += (isDir) ? "dir" : "file";
    output += "\",\"size\":\"";
    output += entry.size();
    output += "\",\"name\":\"";
    output += String(entry.name()).substring(0);
    output += "\"}";
    entry.close();
  }
  output += "]";
  server.send(200, "text/json", output);
}

void checkForUnsupportedPath(String &filename, String &error)
{
  if (!filename.startsWith("/"))
  {
    error += 
# 88 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
            (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "FSBrowser.ino" "." "88" "." "326" "\", \"aSM\", @progbits, 1 #"))) = (
# 88 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
            " !! NO_LEADING_SLASH !! "
# 88 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
            ); &__pstr__[0];}))
# 88 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                                            ;
  }
  if (filename.indexOf("//") != -1)
  {
    error += 
# 92 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
            (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "FSBrowser.ino" "." "92" "." "327" "\", \"aSM\", @progbits, 1 #"))) = (
# 92 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
            " !! DOUBLE_SLASH !! "
# 92 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
            ); &__pstr__[0];}))
# 92 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                                        ;
  }
  if (filename.endsWith("/"))
  {
    error += 
# 96 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
            (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "FSBrowser.ino" "." "96" "." "328" "\", \"aSM\", @progbits, 1 #"))) = (
# 96 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
            " ! TRAILING_SLASH ! "
# 96 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
            ); &__pstr__[0];}))
# 96 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                                        ;
  }
}

// format bytes
String formatBytes(size_t bytes)
{
  if (bytes < 1024)
  {
    return String(bytes) + "B";
  }
  else if (bytes < (1024 * 1024))
  {
    return String(bytes / 1024.0) + "KB";
  }
  else if (bytes < (1024 * 1024 * 1024))
  {
    return String(bytes / 1024.0 / 1024.0) + "MB";
  }
  else
  {
    return String(bytes / 1024.0 / 1024.0 / 1024.0) + "GB";
  }
}

String getContentType(String filename)
{
  if (server.hasArg("download"))
  {
    return "application/octet-stream";
  }
  else if (filename.endsWith(".htm"))
  {
    return "text/html";
  }
  else if (filename.endsWith(".html"))
  {
    return "text/html";
  }
  else if (filename.endsWith(".css"))
  {
    return "text/css";
  }
  else if (filename.endsWith(".sass"))
  {
    return "text/css";
  }
  else if (filename.endsWith(".js"))
  {
    return "application/javascript";
  }
  else if (filename.endsWith(".png"))
  {
    return "image/svg+xml";
  }
  else if (filename.endsWith(".svg"))
  {
    return "image/png";
  }
  else if (filename.endsWith(".gif"))
  {
    return "image/gif";
  }
  else if (filename.endsWith(".jpg"))
  {
    return "image/jpeg";
  }
  else if (filename.endsWith(".ico"))
  {
    return "image/x-icon";
  }
  else if (filename.endsWith(".xml"))
  {
    return "text/xml";
  }
  else if (filename.endsWith(".pdf"))
  {
    return "application/x-pdf";
  }
  else if (filename.endsWith(".zip"))
  {
    return "application/x-zip";
  }
  else if (filename.endsWith(".gz"))
  {
    return "application/x-gzip";
  }
  return "text/plain";
}

// Datei editieren -> speichern CTRL+S
bool handleFileRead(String path)
{
  if (path.endsWith("/"))
  {
    path += "index.html";
  }
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if (LittleFS.exists(pathWithGz) || LittleFS.exists(path))
  {
    if (LittleFS.exists(pathWithGz))
    {
      path += ".gz";
    }
    File file = LittleFS.open(path, "r");
    server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

void handleFileUpload()
{
  if (server.uri() != "/edit")
  {
    return;
  }
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START)
  {
    String filename = upload.filename;
    String result;
    // Make sure paths always start with "/"
    if (!filename.startsWith("/"))
    {
      filename = "/" + filename;
    }
    checkForUnsupportedPath(filename, result);
    if (result.length() > 0)
    {
      replyToCLient(ERROR, 
# 228 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
                          (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "FSBrowser.ino" "." "228" "." "329" "\", \"aSM\", @progbits, 1 #"))) = (
# 228 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                          "INVALID FILENAME"
# 228 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
                          ); &__pstr__[0];}))
# 228 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                                                  );
      return;
    }
    ;
    fsUploadFile = LittleFS.open(filename, "w");
    if (!fsUploadFile)
    {
      replyToCLient(ERROR, 
# 235 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
                          (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "FSBrowser.ino" "." "235" "." "330" "\", \"aSM\", @progbits, 1 #"))) = (
# 235 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                          "CREATE FAILED"
# 235 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
                          ); &__pstr__[0];}))
# 235 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                                               );
      return;
    }
    filename = String();
  }
  else if (upload.status == UPLOAD_FILE_WRITE)
  {
    ;
    if (fsUploadFile)
    {
      fsUploadFile.write(upload.buf, upload.currentSize);
    }
  }
  else if (upload.status == UPLOAD_FILE_END)
  {
    if (fsUploadFile)
    {
      fsUploadFile.close();
    }
    ;
    loadConfig();
  }
}

/*

    Handle a file deletion request

    Operation      | req.responseText

    ---------------+--------------------------------------------------------------

    Delete file    | parent of deleted file, or remaining ancestor

    Delete folder  | parent of deleted folder, or remaining ancestor

*/
# 267 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
void handleFileDelete()
{
  if (server.args() == 0)
  {
    return server.send(500, "text/plain", "BAD ARGS");
  }
  String path = server.arg(0);
  if (!LittleFS.exists(path))
  {
    replyToCLient(NOT_FOUND, 
# 276 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
                            (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "FSBrowser.ino" "." "276" "." "331" "\", \"aSM\", @progbits, 1 #"))) = (
# 276 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                            "FileNotFound"
# 276 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
                            ); &__pstr__[0];}))
# 276 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                                                );
    return;
  }
  //deleteRecursive(path);
  File root = LittleFS.open(path, "r");
  // If it's a plain file, delete it
  if (!root.isDirectory())
  {
    root.close();
    LittleFS.remove(path);
    replyOK();
  }
  else
  {
    LittleFS.rmdir(path);
    replyOK();
  }
}

/*

    Handle the creation/rename of a new file

    Operation      | req.responseText

    ---------------+--------------------------------------------------------------

    Create file    | parent of created file

    Create folder  | parent of created folder

    Rename file    | parent of source file

    Move file      | parent of source file, or remaining ancestor

    Rename folder  | parent of source folder

    Move folder    | parent of source folder, or remaining ancestor

*/
# 307 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
void handleFileCreate()
{
  String path = server.arg("path");
  if (path.isEmpty())
  {
    replyToCLient(BAD_REQUEST, 
# 312 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
                              (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "FSBrowser.ino" "." "312" "." "332" "\", \"aSM\", @progbits, 1 #"))) = (
# 312 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                              "PATH ARG MISSING"
# 312 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
                              ); &__pstr__[0];}))
# 312 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                                                      );
    return;
  }
  if (path == "/")
  {
    replyToCLient(BAD_REQUEST, 
# 317 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
                              (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "FSBrowser.ino" "." "317" "." "333" "\", \"aSM\", @progbits, 1 #"))) = (
# 317 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                              "BAD PATH"
# 317 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
                              ); &__pstr__[0];}))
# 317 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                                              );
    return;
  }

  String src = server.arg("src");
  if (src.isEmpty())
  {
    // No source specified: creation
    if (path.endsWith("/"))
    {
      // Create a folder
      path.remove(path.length() - 1);
      if (!LittleFS.mkdir(path))
      {
        replyToCLient(ERROR, 
# 331 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
                            (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "FSBrowser.ino" "." "331" "." "334" "\", \"aSM\", @progbits, 1 #"))) = (
# 331 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                            "MKDIR FAILED"
# 331 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
                            ); &__pstr__[0];}))
# 331 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                                                );
        return;
      }
    }
    else
    {
      // Create a file
      File file = LittleFS.open(path, "w");
      if (file)
      {
        file.write(0);
        file.close();
      }
      else
      {
        replyToCLient(ERROR, 
# 346 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
                            (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "FSBrowser.ino" "." "346" "." "335" "\", \"aSM\", @progbits, 1 #"))) = (
# 346 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                            "CREATE FAILED"
# 346 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
                            ); &__pstr__[0];}))
# 346 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                                                 );
        return;
      }
    }
    replyToCLient(CUSTOM, path.c_str());
  }
  else
  {
    // Source specified: rename
    if (src == "/")
    {
      replyToCLient(BAD_REQUEST, 
# 357 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
                                (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "FSBrowser.ino" "." "357" "." "336" "\", \"aSM\", @progbits, 1 #"))) = (
# 357 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                                "BAD SRC"
# 357 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
                                ); &__pstr__[0];}))
# 357 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                                               );
      return;
    }

    if (!LittleFS.exists(src))
    {
      replyToCLient(BAD_REQUEST, 
# 363 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
                                (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "FSBrowser.ino" "." "363" "." "337" "\", \"aSM\", @progbits, 1 #"))) = (
# 363 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                                "BSRC FILE NOT FOUND"
# 363 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
                                ); &__pstr__[0];}))
# 363 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                                                           );
      return;
    }

    if (path.endsWith("/"))
    {
      path.remove(path.length() - 1);
    }
    if (src.endsWith("/"))
    {
      src.remove(src.length() - 1);
    }
    if (!LittleFS.rename(src, path))
    {
      replyToCLient(ERROR, 
# 377 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
                          (__extension__({static const char __pstr__[] __attribute__((__aligned__(4))) __attribute__((section( "\".irom0.pstr." "FSBrowser.ino" "." "377" "." "338" "\", \"aSM\", @progbits, 1 #"))) = (
# 377 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                          "RENAME FAILED"
# 377 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino" 3
                          ); &__pstr__[0];}))
# 377 "c:\\Arduino\\git\\BrewDevice\\FSBrowser.ino"
                                               );
      return;
    }
    replyOK();
  }
}
