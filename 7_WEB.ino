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
    message += Version;
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
        delay(PAUSE2SEC);
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
        delay(PAUSE2SEC);
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
        DEBUG_MSG("WEB: Start MS Auto %d\n", structMaischePlan[aktMaischeStep].Dauer);
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
  DEBUG_MSG("%s", "Web: Misc Rew\n");
  server.send(201, "text/plain", "created");
}
void BtnMiscPause()
{
  for (int i = 0; i < server.args(); i++)
  {
    if (server.argName(i) == "mistatePause")
    {
      DEBUG_MSG("Web: Misc Pause %s\n", server.arg(i).c_str());
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
    DEBUG_MSG("%s", "Web: Misc For\n");
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
  webDoc["mstempIst"] = sensors[ID_MS].sens_value;
  webDoc["mstempSoll"] = mstempSoll; //structMaischePlan[aktMaischeStep].Temperatur;
  webDoc["mspowerIst"] = inductionCooker.power;
  webDoc["mspowerSoll"] = mspowerSoll;
  webDoc["ngName"] = ngName;
  webDoc["ngtempIst"] = sensors[ID_NG].sens_value;
  webDoc["ngtempSoll"] = ngtempSoll;
  webDoc["ngpowerIst"] = actors[ID_HEATER].power_actor;
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
  DEBUG_MSG("%s\n", "Rezept Import gestartet");
  int typeRezept = 0;
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START)
  {
    String filename = "upRezept.json"; //upload.filename;
    if (!filename.startsWith("/"))
      filename = "/" + filename;
    DEBUG_MSG("WEB Import recipe handleFileUpload Name: %s\n", filename.c_str());
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
    {                       // If the file was successfully created
      fsUploadFile.close(); // Close the file again
      DEBUG_MSG("WEB Import recipe handleFileUpload Size: %d\n", upload.totalSize);
      server.sendHeader("Location", "/rezept.html"); // Redirect the client to the success page
      server.send(303);                              // Antwort für Post und Put

      fsUploadFile = LittleFS.open("/upRezept.json", "r");
      DynamicJsonDocument testDoc(sizeImportMax);
      DeserializationError error = deserializeJson(testDoc, fsUploadFile);
      fsUploadFile.close();

      if (error)
      {
        DEBUG_MSG("WEB1 Error Json %s\n", error.c_str());
        if (startBuzzer)
          sendAlarm(ALARM_ERROR);
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
  doc["sensoraddress"] = SensorAddressToString(sensors[ID_MS].sens_address);
  doc["sensoroffset"] = sensors[ID_MS].sens_offset;
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
      message += F("<option>");
      message += PinToString(pinswitched);
      message += F("</option><option disabled>──────────</option>");
    }

    for (int i = 0; i < numberOfPins; i++)
    {
      if (pins_used[pins[i]] == false)
      {
        message += F("<option>");
        message += pin_names[i];
        message += F("</option>");
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
  String ms_sensoraddress = sensors[ID_MS].getSens_adress_string();
  float ms_sensoroffset = sensors[ID_MS].sens_offset;
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
  sensors[ID_MS].change(ms_sensoraddress, "MaischeSud", ms_sensoroffset);
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

  sensors[ID_NG].change(ng_sensoraddress, "Nachguss", ng_sensoroffset);
  actors[ID_HEATER].change(ng_heater, "Heater", ng_inverted, ng_pwm, ng_grafana);
  saveConfig();
  server.send(201, "text/plain", "created");
}

void handleRequestNG()
{
  StaticJsonDocument<256> doc;
  doc["ngName"] = ngName;
  doc["sensoraddress"] = SensorAddressToString(sensors[ID_NG].sens_address);
  doc["sensoroffset"] = sensors[ID_NG].sens_offset;
  doc["heater"] = PinToString(actors[ID_HEATER].pin_actor);
  doc["inverted"] = actors[ID_HEATER].getInverted();
  doc["ngautotune"] = ngPIDAutoTune;
  doc["ngkp"] = configNG.Kp;
  doc["ngki"] = configNG.Ki;
  doc["ngkd"] = configNG.Kd;
  doc["grafana"] = actors[ID_HEATER].getGrafana();
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
  doc["agitator"] = PinToString(actors[ID_AGITATOR].pin_actor);
  doc["pump"] = PinToString(actors[ID_PUMP].pin_actor);
  doc["rhe"] = PinToString(actors[ID_RHE].pin_actor);
  doc["agitator_inverted"] = actors[ID_AGITATOR].getInverted();
  doc["pump_inverted"] = actors[ID_PUMP].getInverted();
  doc["rhe_inverted"] = actors[ID_RHE].getInverted();
  doc["agitator_pwm"] = actors[ID_AGITATOR].getPWM();
  doc["pump_pwm"] = actors[ID_PUMP].getPWM();
  doc["rhe_pwm"] = actors[ID_RHE].getPWM();
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
  DynamicJsonDocument doc(sizeRezeptMax);
  DeserializationError error = deserializeJson(doc, server.arg(0));
  if (error)
  {
    DEBUG_MSG("Maische: Error Json %s\n", error.c_str());
    if (startBuzzer)
      sendAlarm(ALARM_ERROR);
    return;
  }
  initMaischePlan();

  JsonArray rastenArray = doc.as<JsonArray>();
  anzahlRasten = rastenArray.size();
  if (anzahlRasten > anzahlRastenMax)
    anzahlRasten = anzahlRastenMax;
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
  if (LittleFS.exists(fileMaischePlan))
    LittleFS.remove(fileMaischePlan);

  maischeResponse = "";
  serializeJson(doc, maischeResponse);

  File maischeFile = LittleFS.open(fileMaischePlan, "w");
  if (!maischeFile)
  {
    DEBUG_MSG("%s\n", "Failed to maische file for writing");
    DEBUG_MSG("%s\n", "------ save Maische aborted ------");
  }
  else
  {
    serializeJson(doc, maischeFile);
    maischeFile.close();
  }
}

void SetHopfenTable()
{
  DynamicJsonDocument doc(sizeHopfenMax);
  DeserializationError error = deserializeJson(doc, server.arg(0));
  if (error)
  {
    DEBUG_MSG("Hopfen: Error Json %s\n", error.c_str());
    if (startBuzzer)
      sendAlarm(ALARM_ERROR);
    return;
  }
  initMaischePlan();

  JsonArray hopfenArray = doc.as<JsonArray>();
  anzahlHopfen = hopfenArray.size();
  if (anzahlHopfen > anzahlHopfenMax)
    anzahlHopfen = anzahlHopfenMax;
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
  if (LittleFS.exists(fileHopfenPlan))
    LittleFS.remove(fileHopfenPlan);

  hopfenResponse = "";
  serializeJson(doc, hopfenResponse);

  File hopfenFile = LittleFS.open(fileHopfenPlan, "w");
  if (!hopfenFile)
  {
    DEBUG_MSG("%s\n", "Failed to hopfen file for writing");
    DEBUG_MSG("%s\n", "------ save Hopfen aborted ------");
  }
  else
  {
    serializeJson(doc, hopfenFile);
    hopfenFile.close();
  }
}

void SetSonstigesTable()
{
  DynamicJsonDocument doc(sizeSonstigesMax);
  DeserializationError error = deserializeJson(doc, server.arg(0));
  if (error)
  {
    DEBUG_MSG("Sonstiges: Error Json %s\n", error.c_str());
    if (startBuzzer)
      sendAlarm(ALARM_ERROR);
    return;
  }
  initMaischePlan();

  JsonArray sonstigesArray = doc.as<JsonArray>();
  anzahlSonstiges = sonstigesArray.size();
  if (anzahlSonstiges > anzahlSonstigesMax)
    anzahlSonstiges = anzahlSonstigesMax;
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
  if (LittleFS.exists(fileSonstigesPlan))
    LittleFS.remove(fileSonstigesPlan);

  sonstigesResponse = "";
  serializeJson(doc, sonstigesResponse);

  File sonstigesFile = LittleFS.open(fileSonstigesPlan, "w");
  if (!sonstigesFile)
  {
    DEBUG_MSG("%s\n", "Failed to sonstiges file for writing");
    DEBUG_MSG("%s\n", "------ save Sonstiges aborted ------");
  }
  else
  {
    serializeJson(doc, sonstigesFile);
    sonstigesFile.close();
  }
}

void handleRequestStepsCount()
{
  DEBUG_MSG("RequestStepsCount: %d\n", anzahlRasten);
  String message;
  // if (anzahlRasten < 1)
  // {
  //   message += F("<option>");
  //   message += F("0");
  //   message += F("</option><option disabled>──────────</option>");
  // }
  for (int i = 0; i < anzahlRasten; i++)
  {
    message += F("<option>");
    message += i+1;
    message += F("</option>");
    yield();
  }
  server.send(200, "text/html", message);
}

void handleRequestActorNames()
{
  DEBUG_MSG("%s\n", "RequestActorNames");
  String message;
  message += F("<option>");
  message += msName;
  message += F("</option>");
  message += F("<option>");
  message += ngName;
  message += F("</option>");
  message += F("<option>");
  message += miNameAgitator;
  message += F("</option>");
  message += F("<option>");
  message += miNamePump;
  message += F("</option>");
  message += F("<option>");
  message += miNameRHE;
  message += F("</option>");
  message += F("<option>");
  message += F("no_actor");
  message += F("</option>");
  yield();

  server.send(200, "text/html", message);
}