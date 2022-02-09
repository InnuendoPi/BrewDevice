void loop()
{
  server.handleClient();    // Webserver handle
  
  if (startMDNS)            // MDNS handle
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

  TickerNTP.update();       // NTP Ticker
}
