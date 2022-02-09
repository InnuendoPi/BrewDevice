bool loadConfig()
{
  DEBUG_MSG("%s\n", "------ loadConfig started ------");
  File configFile = LittleFS.open("/config.txt", "r");
  if (!configFile)
  {
    DEBUG_MSG("%s\n", "Failed to open config file\n");
    DEBUG_MSG("%s\n", "------ loadConfig aborted ------\n");
    return false;
  }

  size_t size = configFile.size();
  if (size > 2048)
  {
    DEBUG_MSG("%s\n", "Config file size is too large");
    DEBUG_MSG("%s\n", "------ loadConfig aborted ------");
    if (startBuzzer)
      sendAlarm(ALARM_ERROR);
    return false;
  }

  DynamicJsonDocument doc(1536);
  DeserializationError error = deserializeJson(doc, configFile);
  if (error)
  {
    DEBUG_MSG("Conf: Error Json %s\n", error.c_str());
    if (startBuzzer)
      sendAlarm(ALARM_ERROR);
    return false;
  }

  JsonArray msArray = doc["MS"];
  JsonObject msObj = msArray[0];

  char ms_sensoraddress[20] = "";
  float ms_sensoroffset = 0.0;

  strlcpy(ms_sensoraddress, msObj["ADDRESS"] | "", sizeof(ms_sensoraddress));
  ms_sensoroffset = msObj["OFFSET"] | 0;

  sensors[ID_MS].change(ms_sensoraddress, "MaischeSud", ms_sensoroffset);
  DEBUG_MSG("Sensor MaischeSud: Address: %s Offset: %f\n", ms_sensoraddress, ms_sensoroffset);

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
    int ids2Delay = msObj["DELAY"] | DEF_DELAY_IND;
    inductionCooker.change(StringToPin(WHITE), StringToPin(YELLOW), StringToPin(BLUE), ids2Delay, true, ms_grafana);
    DEBUG_MSG("Induction: %d Relais (WHITE): %s Command channel (YELLOW): %s Backchannel (BLUE): %s Delay after power off %d \n", inductionStatus, WHITE.c_str(), YELLOW.c_str(), BLUE.c_str(), (ids2Delay / 1000));
    DEBUG_MSG("%s\n", "--------------------");
  }
  else
  {
    inductionStatus = 0;
    DEBUG_MSG("Induction: %d\n", inductionStatus);
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

  sensors[ID_HEATER].change(ng_sensorsAddress, "Nachguss", ng_sensorsOffset);
  DEBUG_MSG("Sensor Nachguss: Address: %s Offset: %f\n", ng_sensorsAddress, ng_sensorsOffset);

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
  actors[ID_HEATER].change(ng_heater, "Heater", ng_inverted, ng_pwm, ng_grafana);
  
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

  strlcpy(miNameAgitator, miObj["MINAMEAGI"] | "", maxSign);
  strlcpy(miNamePump, miObj["MINAMEPUMP"] | "", maxSign);
  strlcpy(miNameRHE, miObj["MINAMERHE"] | "", maxSign);

  strlcpy(mi_agitator, miObj["AGI"] | "", sizeof(mi_agitator));
  mi_agitator_inverted = miObj["INVAGI"] | false;
  mi_agitator_pwm = miObj["PWMAGI"] | false;

  actors[ID_AGITATOR].change(mi_agitator, "Agitator", mi_agitator_inverted, mi_agitator_pwm, false);
  // DEBUG_MSG("Actor Agitator PIN: %s INV: %s GRAF: %s\n", ms_agitator, ms_inverted, ms_grafana);

  strlcpy(mi_pump, miObj["PUMP"] | "", sizeof(mi_pump));
  mi_pump_inverted = miObj["INVPUMP"] | false;
  mi_pump_pwm = miObj["PWMPUMP"] | false;

  actors[ID_PUMP].change(mi_pump, "Pump", mi_pump_inverted, mi_pump_pwm, false);

  strlcpy(mi_rhe, miObj["RHE"] | "", sizeof(mi_rhe));
  mi_rhe_inverted = miObj["INVRHE"] | false;
  mi_rhe_inverted = true;
  mi_rhe_pwm = miObj["PWMRHE"] | false;
  actors[ID_RHE].change(mi_rhe, "RHE", mi_rhe_inverted, mi_rhe_pwm, false);

  strlcpy(sudName, miObj["SUD"] | "", sizeof(sudName));

  // System Settings
  JsonArray sysArray = doc["Sys"];
  JsonObject sysObj = sysArray[0];

  startMDNS = sysObj["MDNS"] | false;
  strlcpy(nameMDNS, sysObj["NAME"] | "", sizeof(nameMDNS));
  DEBUG_MSG("MDNS: %d Name %s \n", startMDNS, nameMDNS);

  startBuzzer = sysObj["BUZ"] | false;
  
  DEBUG_MSG("Buzzer: %d\n", startBuzzer);

  startDB = sysObj["STARTDB"] | false;
  strlcpy(dbServer, sysObj["DBSERVER"] | "", sizeof(dbServer));
  strlcpy(dbDatabase, sysObj["DB"] | "", sizeof(dbDatabase));
  strlcpy(dbUser, sysObj["DBUSER"] | "", sizeof(dbUser));
  strlcpy(dbPass, sysObj["DBPASS"] | "", sizeof(dbPass));
  upInflux = sysObj["DBUP"] | 15000;

  DEBUG_MSG("InfluxDB: %d Server URL %s User: %s Pass: %s\n", startDB, dbServer, dbUser, dbPass);

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

  DEBUG_MSG("Sensors update intervall: %d sec\n", (SEN_UPDATE / 1000));
  DEBUG_MSG("Actors update intervall: %d sec\n", (ACT_UPDATE / 1000));
  DEBUG_MSG("Induction update intervall: %d sec\n", (IND_UPDATE / 1000));
  DEBUG_MSG("InfluxDB update intervall: %d sec\n", (upInflux / 1000));

  testmode = sysObj["TEST"] | false;
  DEBUG_MSG("Testmode: %d\n", testmode);
  DEBUG_MSG("%s\n", "------ loadConfig finished ------");
  configFile.close();
  DEBUG_MSG("Config file size %d\n", size);
  size_t len = measureJson(doc);
  DEBUG_MSG("JSON config length: %d\n", len);
  int memoryUsed = doc.memoryUsage();
  DEBUG_MSG("JSON memory usage: %d\n", memoryUsed);

  if (startBuzzer)
    sendAlarm(ALARM_ON);

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
  DEBUG_MSG("%s\n", "------ saveConfig started ------");
  DynamicJsonDocument doc(1024);

  JsonArray msArray = doc.createNestedArray("MS");
  JsonObject msObj = msArray.createNestedObject();
  msObj["ADDRESS"] = sensors[ID_MS].getSens_adress_string();
  msObj["OFFSET"] = sensors[ID_MS].sens_offset;
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
    DEBUG_MSG("Induction: Enabled %d WHITE: %s YELLOW: %s BLUE: %s Delay %d Graf %d\n", inductionCooker.isEnabled, PinToString(inductionCooker.PIN_WHITE).c_str(), PinToString(inductionCooker.PIN_YELLOW).c_str(), PinToString(inductionCooker.PIN_INTERRUPT).c_str(), (inductionCooker.delayAfteroff / 1000), inductionCooker.setGrafana);
  }
  else
  {
    inductionStatus = 0;
    DEBUG_MSG("Induction: %d\n", inductionCooker.isEnabled);
  }

  DEBUG_MSG("Sensor MaischeSud Address: %s Offset: %f\n", sensors[ID_MS].getSens_adress_string().c_str(), sensors[ID_MS].sens_offset);
  DEBUG_MSG("%s\n", "--------------------");

  JsonArray ngArray = doc.createNestedArray("NG");
  JsonObject ngObj = ngArray.createNestedObject();

  ngObj["NGNAME"] = ngName;
  ngObj["ADDRESS"] = sensors[ID_HEATER].getSens_adress_string();
  ngObj["OFFSET"] = sensors[ID_HEATER].sens_offset;

  DEBUG_MSG("Sensor Nachguss Address: %s Offset: %f\n", sensors[ID_HEATER].getSens_adress_string().c_str(), sensors[ID_HEATER].sens_offset);

  ngObj["PIN"] = PinToString(actors[ID_HEATER].pin_actor);
  ngObj["INV"] = (int)actors[ID_HEATER].getInverted();
  ngObj["PWM"] = (int)actors[ID_HEATER].getPWM();
  ngObj["KP"] = configNG.Kp;
  ngObj["KI"] = configNG.Ki;
  ngObj["KD"] = configNG.Kd;
  ngObj["GRAF"] = (int)actors[ID_HEATER].getGrafana();
  DEBUG_MSG("Actor Nachguss PIN: %s INV: %d PWM: %d GRAF: %d\n", PinToString(actors[ID_HEATER].pin_actor).c_str(), actors[ID_HEATER].getInverted(), actors[ID_HEATER].getPWM(), actors[ID_HEATER].getGrafana());
  DEBUG_MSG("%s\n", "--------------------");

  JsonArray miArray = doc.createNestedArray("MI");
  JsonObject miObj = miArray.createNestedObject();

  miObj["MINAMEAGI"] = miNameAgitator;
  miObj["MINAMEPUMP"] = miNamePump;
  miObj["MINAMRHE"] = miNameRHE;
  miObj["AGI"] = PinToString(actors[ID_AGITATOR].pin_actor);
  miObj["PUMP"] = PinToString(actors[ID_PUMP].pin_actor);
  miObj["RHE"] = PinToString(actors[ID_RHE].pin_actor);
  miObj["INVAGI"] = (int)actors[ID_AGITATOR].getInverted();
  miObj["INVPUMP"] = (int)actors[ID_PUMP].getInverted();
  miObj["INVRHE"] = (int)actors[ID_RHE].getInverted();
  miObj["PWMAGI"] = (int)actors[ID_AGITATOR].getPWM();
  miObj["PWMPUMP"] = (int)actors[ID_PUMP].getPWM();
  miObj["PWMRHE"] = (int)actors[ID_RHE].getPWM();
  miObj["SUD"] = sudName;

  DEBUG_MSG("Actor Misc Agitator PIN: %s INV: %d GRAF: %d\n", PinToString(actors[ID_AGITATOR].pin_actor).c_str(), actors[ID_AGITATOR].getInverted(), actors[ID_AGITATOR].getGrafana());
  DEBUG_MSG("Actor Misc Pump PIN: %s INV: %d PWM: %d GRAF: %d\n", PinToString(actors[ID_PUMP].pin_actor).c_str(), actors[ID_PUMP].getInverted(), actors[ID_PUMP].getPWM(), actors[ID_PUMP].getGrafana());
  DEBUG_MSG("Actor Misc RHE PIN: %s INV: %d PWM: %d GRAF: %d\n", PinToString(actors[ID_RHE].pin_actor).c_str(), actors[ID_RHE].getInverted(), actors[ID_RHE].getPWM(), actors[ID_RHE].getGrafana());
  DEBUG_MSG("%s\n", "--------------------");

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

  DEBUG_MSG("MDNS: %d Name %s \n", startMDNS, nameMDNS);
  DEBUG_MSG("Buzzer: %d\n", startBuzzer);
  DEBUG_MSG("InfluxDB: %d Server URL %s User: %s Pass: %s\n", startDB, dbServer, dbUser, dbPass);
  DEBUG_MSG("Sensor update interval %d sec\n", (SEN_UPDATE / 1000));
  DEBUG_MSG("Actors update interval %d sec\n", (ACT_UPDATE / 1000));
  DEBUG_MSG("Induction update interval %d sec\n", (IND_UPDATE / 1000));
  DEBUG_MSG("InfluxDB update interval %d sec\n", (upInflux / 1000));
  DEBUG_MSG("Testmode: %d\n", testmode);

  size_t len = measureJson(doc);
  int memoryUsed = doc.memoryUsage();

  if (len > 2048 || memoryUsed > 2500)
  {
    DEBUG_MSG("JSON config length: %d\n", len);
    DEBUG_MSG("JSON memory usage: %d\n", memoryUsed);
    DEBUG_MSG("%s\n", "Failed to write config file - config too large");
    DEBUG_MSG("%s\n", "------ saveConfig aborted ------");
    if (startBuzzer)
      sendAlarm(ALARM_ERROR);
    return false;
  }

  File configFile = LittleFS.open("/config.txt", "w");
  if (!configFile)
  {
    DEBUG_MSG("%s\n", "Failed to open config file for writing");
    DEBUG_MSG("%s\n", "------ saveConfig aborted ------");
    if (startBuzzer)
      sendAlarm(ALARM_ERROR);
    return false;
  }
  serializeJson(doc, configFile);
  configFile.close();
  DEBUG_MSG("%s\n", "------ saveConfig finished ------");
  String Network = WiFi.SSID();
  DEBUG_MSG("ESP8266 device IP Address: %s\n", WiFi.localIP().toString().c_str());
  DEBUG_MSG("Configured WLAN SSID: %s\n", Network.c_str());
  DEBUG_MSG("%s\n", "---------------------------------");
  if (startBuzzer)
    sendAlarm(ALARM_ON);
  return true;
}
