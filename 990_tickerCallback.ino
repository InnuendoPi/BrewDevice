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
    msIn = sensors[ID_MS].sens_value;
    msSet = structMaischePlan[aktMaischeStep].Temperatur;
    mspowerSoll = msOut;
    msPID.Compute();
    DEBUG_MSG("Sensor Callback msIn: %f msOut %f msSet %f\n", msIn, msOut, msSet);
  }
}
void tickerActCallback() // Timer Objekt Aktoren
{
  handleActors();
}
void tickerIndCallback() // Timer Objekt Induktion
{
  // Test PID

  DEBUG_MSG("Induction Callback msOut: %f\n", msOut);
  // inductionCooker.newPower = msOut;

  handleInduction();
}
void tickerInfluxDBCallback() // Timer Objekt Influx Datenbank
{
  sendData();
}

void tickerWLANCallback() // Ticker helper function calling Event WLAN Error
{
  DEBUG_MSG("%s", "tickerWLAN: callback\n");
  if (TickerWLAN.counter() == 1)
  {
    switch (WiFi.status())
    {
    case 0: // WL_IDLE_STATUS
      DEBUG_MSG("WiFi status: Fehler rc: %d WL_IDLE_STATUS");
      break;
    case 1: // WL_NO_SSID_AVAIL
      DEBUG_MSG("WiFi status: Fehler rc: %d WL_NO_SSID_AVAIL");
      break;
    case 2: // WL_SCAN_COMPLETED
      DEBUG_MSG("WiFi status: Fehler rc: %d WL_SCAN_COMPLETED");
      break;
    case 3: // WL_CONNECTED
      DEBUG_MSG("WiFi status: Fehler rc: %d WL_CONNECTED");
      break;
    case 4: // WL_CONNECT_FAILED
      DEBUG_MSG("WiFi status: Fehler rc: %d WL_CONNECT_FAILED");
      break;
    case 5: // WL_CONNECTION_LOST
      DEBUG_MSG("WiFi status: Fehler rc: %d WL_CONNECTION_LOST");
      break;
    case 6: // WL_DISCONNECTED
      DEBUG_MSG("WiFi status: Fehler rc: %d WL_DISCONNECTED");
      break;
    case 255: // WL_NO_SHIELD
      DEBUG_MSG("WiFi status: Fehler rc: %d WL_NO_SHIELD");
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
  DEBUG_MSG("%s", "EM WLAN: WLAN Fehler ... versuche neu zu verbinden\n");
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
