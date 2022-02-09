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
      digitalWrite(pin_actor, HIGH);
      pins_used[pin_actor] = false;
      millis2wait(10);
    }

    pin_actor = StringToPin(pin);
    if (isPin(pin_actor))
    {
      pinMode(pin_actor, OUTPUT);
      digitalWrite(pin_actor, HIGH);
      pins_used[pin_actor] = true;
    }

    isOn = false;

    name_actor = aname;
    if (ainverted == true)
    {
      isInverted = true;
      ON = HIGH;
      OFF = LOW;
    }
    else
    {
      isInverted = false;
      ON = LOW;
      OFF = HIGH;
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
Actor actors[numberOfActorsMax] = {
    Actor("", "", false, false, false),  // Agitator
    Actor("", "", false, false, false),  // Heater
    Actor("", "", false, false, false),  // Buzzer
    Actor("", "", false, false, false),  // Pump
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
    message += F("<option>");
    message += PinToString(actors[id].pin_actor);
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
  message += F("<option>");
  message += "deaktivieren";
  message += F("</option>");
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
