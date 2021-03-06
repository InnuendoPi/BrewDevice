void setup()
{
  Serial.begin(115200);
// Debug Ausgaben prüfen
#ifdef DEBUG_ESP_PORT
  Serial.setDebugOutput(true);
#endif

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
  pins_used[ONE_WIRE_BUS] = true;

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
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
  }

  // numberOfSensorsFound = searchSensors();

  setLog();                             // webUpdate log
    if (LittleFS.exists(fileMaischePlan)) // Lade Konfiguration
    readMaischePlan();
  if (LittleFS.exists(fileHopfenPlan))  // Lade Konfiguration
    readHopfenPlan();
  if (LittleFS.exists(fileSonstigesPlan)) // Lade Konfiguration
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
