void readMaischePlan()
{
    File rezMaische = LittleFS.open(fileMaischePlan, "r");
    DynamicJsonDocument doc(sizeRezeptMax);
    DeserializationError error = deserializeJson(doc, rezMaische);
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
    DEBUG_MSG("Ticker handleMaische: %d Dauer: %d Temp: %d\n", aktMaischeStep, structMaischePlan[aktMaischeStep].Dauer, structMaischePlan[aktMaischeStep].Temperatur);
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
                DEBUG_MSG("Ticker handleTaskStart: %d Aufgabe: %s\n", aktMaischeStep, structMaischePlan[i].Name.c_str());
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
    File rezHopfen = LittleFS.open(fileHopfenPlan, "r");
    DynamicJsonDocument doc(sizeHopfenMax);
    DeserializationError error = deserializeJson(doc, rezHopfen);
    if (error)
    {
        DEBUG_MSG("Hopfen: Error Json %s\n", error.c_str());
        if (startBuzzer)
            sendAlarm(ALARM_ERROR);
        return;
    }
    initHopfenPlan();
    JsonArray hopfenArray = doc.as<JsonArray>();
    anzahlHopfen = hopfenArray.size();
    if (anzahlHopfen > anzahlHopfenMax)
        anzahlHopfen = anzahlHopfenMax;
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
    File rezSonstiges = LittleFS.open(fileSonstigesPlan, "r");
    DynamicJsonDocument doc(sizeSonstigesMax);
    DeserializationError error = deserializeJson(doc, rezSonstiges);
    if (error)
    {
        DEBUG_MSG("Sonstiges: Error Json %s\n", error.c_str());
        if (startBuzzer)
            sendAlarm(ALARM_ERROR);
        return;
    }
    initSonstigesPlan();
    JsonArray sonstigesArray = doc.as<JsonArray>();
    anzahlSonstiges = sonstigesArray.size();
    if (anzahlSonstiges > anzahlSonstigesMax)
        anzahlSonstiges = anzahlSonstigesMax;
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
    DynamicJsonDocument dataDoc(sizeRezeptMax);
    DynamicJsonDocument doc(sizeImportMax);
    DeserializationError error = deserializeJson(doc, kbh2File);
    if (error)
    {
        DEBUG_MSG("KBH2: Error Json %s\n", error.c_str());
        if (startBuzzer)
            sendAlarm(ALARM_ERROR);
        return;
    }
    if (LittleFS.exists(fileMaischePlan))
        LittleFS.remove(fileMaischePlan);

    initMaischePlan();

    JsonArray rastenArray = doc["Rasten"];
    anzahlRasten = rastenArray.size();
    if (anzahlRasten > anzahlRastenMax)
        anzahlRasten = anzahlRastenMax;
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

            DEBUG_MSG("Rast #: %d Name: %s Temp: %d Dauer: %d \n", (i + 1), structMaischePlan[i].Name.c_str(), structMaischePlan[i].Temperatur, structMaischePlan[i].Dauer);
            i++;
        }
    }

    JsonObject Sud = doc["Sud"];
    String leseDauer = Sud["Kochdauer"];
    
    strlcpy(sudName, Sud["Sudname"], sizeof(sudName));
    
    String leseName = "Kochen";
    String leseTemp = "100°C";
    DEBUG_MSG("Rast #: %d Name: %s Temp: %s Dauer: %s \n", (i + 1), leseName.c_str(), leseTemp.c_str(), leseDauer.c_str());
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

    File maischeFile = LittleFS.open(fileMaischePlan, "w");
    if (!maischeFile)
    {
        DEBUG_MSG("%s\n", "Failed to maische file for writing");
        DEBUG_MSG("%s\n", "------ save Maische aborted ------");
    }
    else
    {
        serializeJson(dataDoc, maischeFile);
        maischeFile.close();
    }

    size_t len = measureJson(dataDoc);
    int memoryUsed = dataDoc.memoryUsage();

    DEBUG_MSG("JSON dataDoc length: %d\n", len);
    DEBUG_MSG("JSON dataDoc memory usage: %d\n", memoryUsed);

    if (LittleFS.exists(fileHopfenPlan))
        LittleFS.remove(fileHopfenPlan);
    initHopfenPlan();
    DynamicJsonDocument hopfenDoc(512);

    JsonArray readArray = doc["Hopfengaben"];
    anzahlHopfen = readArray.size();
    if (anzahlHopfen > anzahlHopfenMax)
        anzahlHopfen = anzahlHopfenMax;
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

            DEBUG_MSG("Hopfen #: %d Name: %s Zeit: %d Menge: %f \n", (i + 1), structHopfenPlan[i].Name.c_str(), structHopfenPlan[i].Zeit, structHopfenPlan[i].Menge);
            i++;
        }
    }
    hopfenResponse = "";
    serializeJson(hopfenDoc, hopfenResponse);

    File hopfenFile = LittleFS.open(fileHopfenPlan, "w");
    if (!hopfenFile)
    {
        DEBUG_MSG("%s\n", "Failed to Hopfen file for writing");
        DEBUG_MSG("%s\n", "------ save Hopfen aborted ------");
    }
    else
    {
        serializeJson(hopfenDoc, hopfenFile);
        hopfenFile.close();
    }

    size_t hopfenLen = measureJson(hopfenDoc);
    int hopfenMemoryUsed = hopfenDoc.memoryUsage();

    DEBUG_MSG("JSON hopfenDoc length: %d\n", hopfenLen);
    DEBUG_MSG("JSON hopfenDoc memory usage: %d\n", hopfenMemoryUsed);
    kbh2File.close();
}

void BtnImportMMUM()
{
    File mmumFile = LittleFS.open("/upRezept.json", "r");
    DynamicJsonDocument dataDoc(sizeRezeptMax);
    DynamicJsonDocument doc(sizeImportMax);
    DeserializationError error = deserializeJson(doc, mmumFile);
    if (error)
    {
        DEBUG_MSG("MMuM: Error Json %s\n", error.c_str());
        if (startBuzzer)
            sendAlarm(ALARM_ERROR);
        return;
    }
    if (LittleFS.exists(fileMaischePlan))
        LittleFS.remove(fileMaischePlan);
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

    File maischeFile = LittleFS.open(fileMaischePlan, "w");
    if (!maischeFile)
    {
        DEBUG_MSG("%s\n", "Failed to maische file for writing");
        DEBUG_MSG("%s\n", "------ save Maische aborted ------");
    }
    else
    {
        serializeJson(dataDoc, maischeFile);
        maischeFile.close();
    }

    size_t len = measureJson(dataDoc);
    int memoryUsed = dataDoc.memoryUsage();

    DEBUG_MSG("JSON dataDoc length: %d\n", len);
    DEBUG_MSG("JSON dataDoc memory usage: %d\n", memoryUsed);

    if (LittleFS.exists(fileHopfenPlan))
        LittleFS.remove(fileHopfenPlan);
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

    File hopfenFile = LittleFS.open(fileHopfenPlan, "w");
    if (!hopfenFile)
    {
        DEBUG_MSG("%s\n", "Failed to Hopfen file for writing");
        DEBUG_MSG("%s\n", "------ save Hopfen aborted ------");
    }
    else
    {
        serializeJson(hopfenDoc, hopfenFile);
        hopfenFile.close();
    }

    size_t hopfenLen = measureJson(hopfenDoc);
    int hopfenMemoryUsed = hopfenDoc.memoryUsage();

    DEBUG_MSG("JSON hopfenDoc length: %d\n", hopfenLen);
    DEBUG_MSG("JSON hopfenDoc memory usage: %d\n", hopfenMemoryUsed);
    mmumFile.close();
}

void initMaischePlan()
{
    struct MaischePlan structMaischePlan[anzahlRastenMax] = {
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
    struct HopfenPlan structHopfenPlan[anzahlHopfenMax] = {
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
    struct SonstigesPlan structSonstigesPlan[anzahlSonstigesMax] = {
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
