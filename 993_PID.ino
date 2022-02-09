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
    DEBUG_MSG("PID MaischeSud Testmode %d\n", testmode);
    if (!testmode)
        msIn = sensors[ID_MS].sens_value;
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
        msIn = sensors[ID_MS].sens_value;
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
    msPID.SetMode(AUTOMATIC);
}

void msSaveAutoTune(void)
{
    DEBUG_MSG("AutoTune abgeschlossen PRE Kp: %f Ki: %f Kd: %f\n", configMS.Kp, configMS.Ki, configMS.Kd);
    configMS.Kp = msTune.GetKp();
    configMS.Ki = msTune.GetKi();
    configMS.Kd = msTune.GetKd();
    msPID.SetTunings(configMS.Kp, configMS.Ki, configMS.Kd);
    DEBUG_MSG("AutoTune abgeschlossen POST Kp: %f Ki: %f Kd: %f\n", configMS.Kp, configMS.Ki, configMS.Kd);
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
    DEBUG_MSG("PID Nachguss Testmode %d\n", testmode);
    if (!testmode)
        ngIn = sensors[ID_NG].sens_value;
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
        ngIn = sensors[ID_NG].sens_value;
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
            analogWrite(actors[ID_HEATER].pin_actor, ngOut);

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
    ngPID.SetMode(AUTOMATIC);
}

void ngSaveAutoTune(void)
{
    DEBUG_MSG("AutoTune abgeschlossen PRE Kp: %f Ki: %f Kd: %f\n", configNG.Kp, configNG.Ki, configNG.Kd);
    configNG.Kp = ngTune.GetKp();
    configNG.Ki = ngTune.GetKi();
    configNG.Kd = ngTune.GetKd();
    ngPID.SetTunings(configNG.Kp, configNG.Ki, configNG.Kd);
    DEBUG_MSG("AutoTune abgeschlossen POST Kp: %f Ki: %f Kd: %f\n", configNG.Kp, configNG.Ki, configNG.Kd);
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
