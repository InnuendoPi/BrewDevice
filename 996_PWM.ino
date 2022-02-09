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
    
    DEBUG_MSG("UpdatePWM: %f %d\n", heaterPWM, heaterProzent);
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