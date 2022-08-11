//Declare libraries
#include "max6675.h"
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <PID_v1.h>

#define maxTarget 500
#define minTarget 0
#define maxRate 10
#define minRate 1
#define maxDuration 999
#define minDuration 0
//#define maxMinpoint 50
//#define minMinpoint 0
#define maxMenu 6
#define minMenu 0
#define maxPIDMenuNum 3
#define minPIDMenuNum 0
#define EEPROM_SIZE 512
#define addTarget1 0
#define addTarget2 2
#define addRate 10
#define addDuration 20
//#define addMinpoint 30
#define addDurationMode 39
#define addKp 40
#define addKi 50
#define addKd 60
#define addSencal 70
#define addNegsen 80

//Decalre LCD
//LiquidCrystal lcd(RS, E, BUS4,BUS5,BUS6,BUS7);
LiquidCrystal_I2C lcd(0x27, 16, 2);

byte degree[8] = {
  0B01110,
  0B01010,
  0B01110,
  0B00000,
  0B00000,
  0B00000,
  0B00000,
  0B00000
};
byte runSign[8] = {
  0B00100,
  0B01110,
  0B10101,
  0B00100,
  0B00100,
  0B00100,
  0B11111,
  0B00000
};
byte holdSign[8] = {
  0B11111,
  0B11111,
  0B10101,
  0B10001,
  0B10101,
  0B11111,
  0B11111,
  0B00000
};
//Declare PWM
byte hCtrl = 27;
const int freq = 5000;
const int pwmChannel = 0;
const int resolution = 8;
//Khai bao cho module doc nhiet do
unsigned long previousMillis = 0;
unsigned long serialMillis = 0;
byte thermoDO = 19;
byte thermoCS = 5;
byte thermoCLK = 18;
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

//Declare rate
double setpoint;
double setpointIner;
int minpoint;
double target;
double rate;
int duration;

//Declare real teperature variable
double curtem;

//Declare sensor calibration variable
double sencal;

//Declare PID variables
double kp = 1, ki = 1, kd = 0.01;
double output = 0;

//Declare encoder/button pins
byte runButton = 32;
byte setButton = 14;
byte dt = 25;
byte clk = 33;
bool dtDetec;
volatile bool clkDetec = false;
bool clkStatus = 0;
bool clkLastStatus = 0;

bool refreshLCD = false;

//Declare run/hold status
volatile bool setDetec = false;
bool lastsetDetec = false;

bool setButtonStatus = 0;
bool setButtonLastStatus = 0;

//Declare menu
bool menuStatus = false;
int menuNum = 0;
bool targetStatus;
bool rateStatus;
bool durationStatus;
bool durationModeStatus;
bool durationMenuNum;
bool PIDStatus;
int PIDMenuNum = 0;
bool kpStatus;
bool kiStatus;
bool kdStatus;
bool minpointStatus;
bool sencalStatus;
bool negsen;

//Declare run/hold status
bool runStatus;
bool runButtonStatus = 0;
bool runButtonLastStatus = 0;

//Declare duration OFF/ON status
bool durationMode;

//Interrupt function for encoder
void IRAM_ATTR isrEncoder()
{
  //  clkDetec = true;
  clkStatus = digitalRead(clk);
  if (clkStatus != clkLastStatus) {
    if (clkStatus == HIGH) {
      clkDetec = true;
    }
  }
  clkLastStatus = clkStatus;
}

//Interrupt function for encoder's button a.k.a setbutton
void IRAM_ATTR isrsetButton()
{
  //  setDetec = !setDetec;
  setButtonStatus = digitalRead(setButton);
  if (setButtonStatus != setButtonLastStatus) {
    if (setButtonStatus == HIGH) {
      setDetec = !setDetec;
    }
  }
  setButtonLastStatus = setButtonStatus;
  refreshLCD = true;
}

//Interrupt function for runMode/holdMode
void IRAM_ATTR isrrunButton()
{
  //  runStatus = !runStatus;
  runButtonStatus = digitalRead(runButton);
  if (runButtonStatus != runButtonLastStatus) {
    if (runButtonStatus == HIGH) {
      runStatus = !runStatus;
    }
  }
  runButtonLastStatus = runButtonStatus;
  refreshLCD = true;
}

//Declare timer1 for rate* variable
hw_timer_t*timer1 = NULL;
void IRAM_ATTR rateUp()
{
  setpoint = setpoint + rate / 1000.0;
  setpoint = min(setpoint, double(target));
}

//Declare timer2 for duration* variable
hw_timer_t*timer2 = NULL;
void IRAM_ATTR durationDown()
{
  duration--;
  duration = max(duration, minDuration);
}

//==================================================//
//                      SETUP                       //
//==================================================//
void setup() {
  Serial.begin(115200);
  //
  pinMode(clk, INPUT_PULLUP);
  pinMode(dt, INPUT_PULLUP);
  pinMode(setButton, INPUT_PULLUP);
  pinMode(runButton, INPUT_PULLUP);
  attachInterrupt(clk, isrEncoder, FALLING);
  attachInterrupt(setButton, isrsetButton, FALLING);
  attachInterrupt(runButton, isrrunButton, FALLING);
  //
  lcd.init();
  lcd.backlight();
  lcd.createChar(1, degree);  //degree sidn
  lcd.createChar(2, runSign);  //heat up sign
  lcd.createChar(3, holdSign);
  lcd.clear();
  Serial.println("MAX6675 test");
  // wait for MAX chip to stabilize
  delay(250);
  //Read be saved variables
  EEPROM.begin(EEPROM_SIZE);
  target = EEPROM.read(addTarget1) + EEPROM.read(addTarget2);
  rate = EEPROM.read(addRate);
  durationMode = EEPROM.read(addDurationMode);
  duration = EEPROM.read(addDuration);
  minpoint = thermocouple.readCelsius();
  kp = double(EEPROM.read(addKp)) / 10.0;
  ki = double(EEPROM.read(addKi)) / 10.0;
  kd = double(EEPROM.read(addKd)) / 100.0;
  negsen = EEPROM.read(addNegsen);
  if (negsen)
  {
    sencal = double(EEPROM.read(addSencal)) / 10.0 * -1;
  } else
  {
    sencal = double(EEPROM.read(addSencal)) / 10.0;
  }

  runStatus = false;
  
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(hCtrl, pwmChannel);

  //setup timer1 for rate*
  timer1 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer1, rateUp, true);

  setpoint = minpoint;

  //setup timer2 for duration*
  timer2 = timerBegin(2, 80, true);
  timerAttachInterrupt(timer2, durationDown, true);
}

//==================================================//
//                      LOOP                        //
//==================================================//
void loop() {
  if (refreshLCD) {
    updateLCD();
  }

  PID myPID(&curtem, &output, &setpoint, kp, ki, kd, DIRECT);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(0, 255);
  //Detec set status
  if ((setDetec ^ lastsetDetec) && menuStatus == false)
  {
    menuStatus = true;
    lastsetDetec = setDetec;
  }
  else
  {
    mainScreen();
    mainScreenDisplay();
  }
  attachInterrupt(setButton, isrsetButton, FALLING);
  attachInterrupt(runButton, isrrunButton, FALLING);
  durationMode = EEPROM.read(addDurationMode);
  if (isnan(curtem))
  {
    runStatus = false;
  }
  while (runStatus)
  {
    detachInterrupt(setButton);
    timerAlarmWrite(timer1, 60000, runStatus);
    timerAlarmEnable(timer1);
    lcd.setCursor(7, 0);
    lcd.write(2);

    myPID.Compute();
    Serial.println(output);
    if (curtem < setpoint)
    {
      ledcWrite(pwmChannel, output);
    } else
    {
      ledcWrite(pwmChannel, 0);
    }
    if (durationMode == true) {
      if (setpoint == target )
      {
        timerAlarmWrite(timer2, 60000000, true);
        timerAlarmEnable(timer2);
      }
      else
      {
        timerAlarmWrite(timer2, 60000000, false);
        timerAlarmDisable(timer2);
      }
      if (duration == 0 )
      {
        runStatus = !runStatus;
        setpoint = minpoint;
      }
    }
    mainScreen();
    mainScreenDisplay();
  }
  
  ledcWrite(pwmChannel, 0);
  
  lcd.setCursor(7, 0);
  lcd.write(3);
  timerAlarmWrite(timer1, 60000, false);
  timerAlarmDisable(timer1);

  //Che do cai dat cac thong so trong menu
  while (menuStatus)
  {
    detachInterrupt(runButton);
    encoderMenu();
    setModeDisplay();
    switch (menuNum) {
      case 0:
        if (setDetec ^ lastsetDetec)
        {
          menuStatus = false;
          lastsetDetec = setDetec;
        }
        break;
      case 1:
        if (setDetec ^ lastsetDetec)
        {
          targetStatus = true;
          lastsetDetec = setDetec;
        }
        target = EEPROM.read(addTarget1) + EEPROM.read(addTarget2);
        while (targetStatus)
        {
          if (refreshLCD) {
            updateLCD();
          }
          setTarget();
          if (setDetec ^ lastsetDetec)
          {
            targetStatus = false;
            lastsetDetec = setDetec;
          }
        }
        if (target > 255)
        {
          EEPROM.write(addTarget1, 255);
          EEPROM.write(addTarget2, target - 255);
        } else {
          EEPROM.write(addTarget1, target);
          EEPROM.write(addTarget2, 0);
        }
        EEPROM.commit();
        setModeDisplay();
        break;
      case 2:
        if (setDetec ^ lastsetDetec)
        {
          rateStatus = true;
          lastsetDetec = setDetec;
        }
        rate = EEPROM.read(addRate);
        while (rateStatus)
        {
          if (refreshLCD) {
            updateLCD();
          }
          setRate();
          if (setDetec ^ lastsetDetec)
          {
            rateStatus = false;
            lastsetDetec = setDetec;
          }
        }
        EEPROM.write(addRate, rate);
        EEPROM.commit();
        setModeDisplay();
        break;
      case 3:
        if (setDetec ^ lastsetDetec)
        {
          durationStatus = true;
          lastsetDetec = setDetec;
        }
        durationMode = EEPROM.read(addDurationMode);
        durationMenuNum = durationMode;
        while (durationStatus)
        {
          if (refreshLCD) {
            updateLCD();
          }
          encoderDurationMenu();
          switch (durationMenuNum) {
            case 0:
              lcd.setCursor(1, 0);
              lcd.print("OFF            ");
              lcd.setCursor(1, 1);
              lcd.print("ON             ");
              lcd.setCursor(0, 0);
              lcd.print(">");
              lcd.setCursor(0, 1);
              lcd.print(" ");
              break;
            case 1:
              lcd.setCursor(1, 0);
              lcd.print("OFF            ");
              lcd.setCursor(1, 1);
              lcd.print("ON             ");
              lcd.setCursor(0, 1);
              lcd.print(">");
              lcd.setCursor(0, 0);
              lcd.print(" ");
              break;
          }
          switch (durationMenuNum) {
            case 0:
              if (setDetec ^ lastsetDetec)
              {
                durationStatus = false;
                durationMode = false;
                EEPROM.write(addDurationMode, durationMode);
                EEPROM.commit();
                lastsetDetec = setDetec;
              }
              break;
            case 1:
              if (setDetec ^ lastsetDetec)
              {
                durationModeStatus = true;
                lastsetDetec = setDetec;
              }
              while (durationModeStatus)
              {
                if (refreshLCD) {
                  updateLCD();
                }
                setDuration();
                if (setDetec ^ lastsetDetec)
                {
                  durationModeStatus = false;
                  durationStatus = false;
                  durationMode = true;
                  EEPROM.write(addDurationMode, durationMode);
                  EEPROM.write(addDuration, duration);
                  EEPROM.commit();
                  lastsetDetec = setDetec;
                }
              }
              break;
          }
        }
        setModeDisplay();
        break;
      case 4:
        if (setDetec ^ lastsetDetec)
        {
          PIDStatus = true;
          lastsetDetec = setDetec;
        }

        while (PIDStatus)
        {
          if (refreshLCD) {
            updateLCD();
          }
          encoderPIDMenu();
          switch (PIDMenuNum) {
            case 0:
              lcd.setCursor(1, 0);
              lcd.print("BACK           ");
              lcd.setCursor(1, 1);
              lcd.print("Kp         ");
              lcd.setCursor(12, 1);
              lcd.print(kp, 2);
              lcd.setCursor(0, 0);
              lcd.print(">");
              lcd.setCursor(0, 1);
              lcd.print(" ");
              if (setDetec ^ lastsetDetec)
              {
                PIDStatus = false;
                lastsetDetec = setDetec;
              }
              break;
            case 1:
              lcd.setCursor(1, 0);
              lcd.print("BACK           ");
              lcd.setCursor(1, 1);
              lcd.print("Kp         ");
              lcd.setCursor(12, 1);
              lcd.print(kp, 2);
              lcd.setCursor(0, 1);
              lcd.print(">");
              lcd.setCursor(0, 0);
              lcd.print(" ");
              if (setDetec ^ lastsetDetec)
              {
                kpStatus = true;
                lastsetDetec = setDetec;
              }
              kp = double(EEPROM.read(addKp)) / 10.0;
              while (kpStatus)
              {
                if (refreshLCD) {
                  updateLCD();
                }
                setKp();
                if (setDetec ^ lastsetDetec)
                {
                  kpStatus = false;

                  lastsetDetec = setDetec;
                }
                kp = kp * 10;
                EEPROM.write(addKp, int(kp));
                EEPROM.commit();
                kp = kp / 10.0;
              }
              break;
            case 2:
              lcd.setCursor(1, 0);
              lcd.print("Ki         ");
              lcd.setCursor(12, 0);
              lcd.print(ki, 2);
              lcd.setCursor(1, 1);
              lcd.print("Kd         ");
              lcd.setCursor(12, 1);
              lcd.print(kd, 2);
              lcd.setCursor(0, 0);
              lcd.print(">");
              lcd.setCursor(0, 1);
              lcd.print(" ");
              if (setDetec ^ lastsetDetec)
              {
                kiStatus = true;
                lastsetDetec = setDetec;
              }
              ki = double(EEPROM.read(addKi)) / 10.0;
              while (kiStatus)
              {
                if (refreshLCD) {
                  updateLCD();
                }
                setKi();
                if (setDetec ^ lastsetDetec)
                {
                  kiStatus = false;

                  lastsetDetec = setDetec;
                }
                ki = ki * 10;
                EEPROM.write(addKi, int(ki));
                EEPROM.commit();
                ki = ki / 10.0;
              }
              break;
            case 3:
              lcd.setCursor(1, 0);
              lcd.print("Ki         ");
              lcd.setCursor(12, 0);
              lcd.print(ki, 2);
              lcd.setCursor(1, 1);
              lcd.print("Kd         ");
              lcd.setCursor(12, 1);
              lcd.print(kd, 2);
              lcd.setCursor(0, 1);
              lcd.print(">");
              lcd.setCursor(0, 0);
              lcd.print(" ");
              if (setDetec ^ lastsetDetec)
              {
                kdStatus = true;
                lastsetDetec = setDetec;
              }
              kd = double(EEPROM.read(addKd)) / 100.0;
              while (kdStatus)
              {
                if (refreshLCD) {
                  updateLCD();
                }
                setKd();
                //kd=0.01;
                if (setDetec ^ lastsetDetec)
                {
                  kdStatus = false;

                  lastsetDetec = setDetec;
                }
                kd = kd * 100;
                EEPROM.write(addKd, int(kd));
                EEPROM.commit();
                kd = kd / 100.0;
              }
              break;
          }
        }
        setModeDisplay();
        break;
      
      case 5:
        if (setDetec ^ lastsetDetec)
        {
          sencalStatus = true;
          lastsetDetec = setDetec;
        }
        if (negsen)
        {
          sencal = double(EEPROM.read(addSencal)) / 10.0 * -1;
        } else
        {
          sencal = double(EEPROM.read(addSencal)) / 10.0;
        }
        while (sencalStatus)
        {
          if (refreshLCD) {
            updateLCD();
          }
          setSencal();
          if (sencal < 0)
          {
            negsen = true;
          } else
          {
            negsen = false;
          }
          if (setDetec ^ lastsetDetec)
          {
            sencalStatus = false;
            lastsetDetec = setDetec;
          }
        }
        if (negsen)
        {
          EEPROM.write(addNegsen, true);
          sencal = sencal * 10 * -1;
          EEPROM.write(addSencal, int(sencal));
          EEPROM.commit();
          sencal = sencal / 10.0 * -1;
        } else
        {
          EEPROM.write(addNegsen, false);
          sencal = sencal * 10;
          EEPROM.write(addSencal, int(sencal));
          EEPROM.commit();
          sencal = sencal / 10.0;
        }
        setModeDisplay();
        break;
    }
  }
}

//Cai dat nhiet do dich - Target
void setTarget() {
  if (target < 10)
  {
    lcd.setCursor(9, 0);
    lcd.print(target);
    lcd.setCursor(7, 0);
    lcd.print("  ");
  }
  else if (target < 100 && target >= 10)
  {
    lcd.setCursor(8, 0);
    lcd.print(target);
    lcd.setCursor(7, 0);
    lcd.print(" ");
  }
  else
  {
    lcd.setCursor(7, 0);
    lcd.print(target);
  }
  if (clkDetec && (!digitalRead(clk)))
  {
    if (digitalRead(dt) != dtDetec)
    {
      target++;
      target = min(target, double(maxTarget));
    }
    else
    {
      target--;
      target = max(target, double(minTarget));
    }
    clkDetec = false;
  }
}
//Cai dat toc do gia nhiet - Rate
void setRate() {
  if (rate < 10)
  {
    lcd.setCursor(8, 0);
    lcd.print(rate);
    lcd.setCursor(7, 0);
    lcd.print(" ");
  }
  else
  {
    lcd.setCursor(7, 0);
    lcd.print(rate);
  }
  if (clkDetec && (!digitalRead(clk)))
  {
    if (digitalRead(dt) != dtDetec)
    {
      rate++;
      rate = min(rate, double(maxRate));
    }
    else
    {
      rate--;
      rate = max(rate, double(minRate));
    }
    clkDetec = false;
  }
}
//Cai dat thoi gian gia nhiet duration
void setDuration() {
  if (duration < 10)
  {
    lcd.setCursor(9, 0);
    lcd.print(duration);
    lcd.setCursor(7, 0);
    lcd.print("  ");
  }
  else if (duration < 100 && duration >= 10)
  {
    lcd.setCursor(8, 0);
    lcd.print(duration);
    lcd.setCursor(7, 0);
    lcd.print(" ");
  }
  else
  {
    lcd.setCursor(7, 0);
    lcd.print(duration);
  }
  if (clkDetec && (!digitalRead(clk)))
  {
    if (digitalRead(dt) != dtDetec)
    {
      duration++;
      duration = min(duration, maxDuration);
    }
    else
    {
      duration--;
      duration = max(duration, minDuration);
    }
    clkDetec = false;
  }
}

//Cai dat thong so PID
void setKp() {
  lcd.setCursor(9, 0);
  lcd.print(kp, 1);
  if (clkDetec && (!digitalRead(clk)))
  {
    if (digitalRead(dt) != dtDetec)
    {
      kp = kp + 0.1;
    }
    else
    {
      kp = kp - 0.1;
    }
    clkDetec = false;
  }
}
void setKi() {
  lcd.setCursor(9, 0);
  lcd.print(ki, 1);
  if (clkDetec && (!digitalRead(clk)))
  {
    if (digitalRead(dt) != dtDetec)
    {
      ki = ki + 0.1;
    }
    else
    {
      ki = ki - 0.1;
    }
    clkDetec = false;
  }
}
void setKd() {
  lcd.setCursor(9, 0);
  lcd.print(kd, 2);
  if (clkDetec && (!digitalRead(clk)))
  {
    if (digitalRead(dt) != dtDetec)
    {
      kd = kd + 0.01;
    }
    else
    {
      kd = kd - 0.01;
    }
    clkDetec = false;
  }
}
void setSencal()
{
  lcd.setCursor(9, 0);
  lcd.print(sencal, 1);
  if (sencal >= 0 && sencal < 10)
  {
    lcd.setCursor(12, 0);
    lcd.print(" ");
  }
  if (clkDetec && (!digitalRead(clk)))
  {
    if (digitalRead(dt) != dtDetec)
    {
      sencal = sencal + 0.5;
    }
    else
    {
      sencal = sencal - 0.5;
    }
    clkDetec = false;
  }
}
//Hien thi man hinh setMode
void setModeDisplay() {
  switch (menuNum) {
    case 0:
      lcd.setCursor(1, 0);
      lcd.print("Main Screen    ");
      lcd.setCursor(1, 1);
      lcd.print("Target         ");
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      break;
    case 1:
      lcd.setCursor(1, 0);
      lcd.print("Main Screen    ");
      lcd.setCursor(1, 1);
      lcd.print("Target         ");
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(0, 0);
      lcd.print(" ");
      break;
    case 2:
      lcd.setCursor(1, 0);
      lcd.print("Rate           ");
      lcd.setCursor(1, 1);
      lcd.print("Duration       ");
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      break;
    case 3:
      lcd.setCursor(1, 0);
      lcd.print("Rate           ");
      lcd.setCursor(1, 1);
      lcd.print("Duration       ");
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(0, 0);
      lcd.print(" ");
      break;
    case 4:
      lcd.setCursor(1, 0);
      lcd.print("P.I.D          ");
      lcd.setCursor(1, 1);
      lcd.print("Sensor Calib    ");
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      break;
    case 5:
      lcd.setCursor(1, 0);
      lcd.print("P.I.D          ");
      lcd.setCursor(1, 1);
      lcd.print("Sensor Calib    ");
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(0, 0);
      lcd.print(" ");
      break;
  }
}

void encoderMenu() {
  if (clkDetec && (!digitalRead(clk)))
  {
    if (digitalRead(dt) != dtDetec)
    {
      menuNum++;
      menuNum = min(menuNum, maxMenu);
    }
    else
    {
      menuNum--;
      menuNum = max(menuNum, minMenu);
    }
    clkDetec = false;
  }
}
void encoderDurationMenu() {
  if (clkDetec && (!digitalRead(clk)))
  {
    if (digitalRead(dt) != dtDetec)
    {
      durationMenuNum = !durationMenuNum;
    }
    else
    {
      durationMenuNum = !durationMenuNum;
    }
    clkDetec = false;
  }
}

void encoderPIDMenu() {
  if (clkDetec && (!digitalRead(clk)))
  {
    if (digitalRead(dt) != dtDetec)
    {
      PIDMenuNum++;
      PIDMenuNum = min(PIDMenuNum, maxPIDMenuNum);
    }
    else
    {
      PIDMenuNum--;
      PIDMenuNum = max(PIDMenuNum, minPIDMenuNum);
    }
    clkDetec = false;
  }
}

//==================================================//
//     Ham hien thi thong tin len man hinh chinh    //
//==================================================//
void mainScreenDisplay() {
  if (millis() - previousMillis >= 500)
  {
    curtem = thermocouple.readCelsius() + sencal;

    previousMillis = millis();
  }
  if (millis() - serialMillis >= 1000)
  {
    Serial.println(curtem);

    serialMillis = millis();
  }
  //Hien thi curtem
  if (curtem < 10)
  {
    lcd.setCursor(8, 0);
    lcd.print("  ");
    lcd.setCursor(10, 0);
    lcd.print(curtem, 1);
  }
  else if (curtem < 100 && curtem >= 10)
  {
    lcd.setCursor(8, 0);
    lcd.print(" ");
    lcd.setCursor(9, 0);
    lcd.print(curtem, 1);
  }
  else
  {
    lcd.setCursor(8, 0);
    lcd.print(curtem, 1);
  }
  //Hien thi setpoint
  if (setpoint < 10)
  {
    lcd.setCursor(3, 0);
    lcd.print(int(setpoint));
  }
  else if (setpoint < 100 && setpoint >= 10)
  {
    lcd.setCursor(2, 0);
    lcd.print(int(setpoint));
  }
  else
  {
    lcd.setCursor(1, 0);
    lcd.print(int(setpoint));
  }
  //Hien thi target
  if (target < 10)
  {
    lcd.setCursor(9, 1);
    lcd.print(int(target));
  }
  else if (target < 100 && target >= 10)
  {
    lcd.setCursor(8, 1);
    lcd.print(int(target));
  }
  else
  {
    lcd.setCursor(7, 1);
    lcd.print(int(target));
  }
  //Hien thi rate
  if (rate < 10)
  {
    lcd.setCursor(1, 1);
    lcd.print(int(rate));
  }
  else
  {
    lcd.setCursor(0, 1);
    lcd.print(int(rate));
  }
  //Hien thi duration
  if (durationMode == true)
  {
    if (duration == 0)
    {
      lcd.setCursor(13, 1);
      lcd.print("STO");
    }
    else if (duration < 10 && duration >= 1)
    {
      lcd.setCursor(15, 1);
      lcd.print(duration);
      lcd.setCursor(13, 1);
      lcd.print("  ");
    }
    else if (duration < 100 && duration >= 10)
    {
      lcd.setCursor(14, 1);
      lcd.print(duration);
      lcd.setCursor(13, 1);
      lcd.print(" ");
    }
    else
    {
      lcd.setCursor(13, 1);
      lcd.print(duration);
    }
  }
  else
  {
    lcd.setCursor(13, 1);
    lcd.print("OFF");
  }
}

//==================================================//
//          Ham cau hinh cho man hinh chinh         //
//==================================================//
void mainScreen() {
  lcd.setCursor(4, 0);
  lcd.write(1);
  lcd.setCursor(5, 0);
  lcd.print("C");
  lcd.setCursor(6, 0);
  lcd.print("|");
  lcd.setCursor(14, 0);
  lcd.write(1);
  lcd.setCursor(15, 0);
  lcd.print("C");
  lcd.setCursor(2, 1);
  lcd.write(1);
  lcd.setCursor(3, 1);
  lcd.print("C/m");
  lcd.setCursor(6, 1);
  lcd.print("|");
  lcd.setCursor(10, 1);
  lcd.write(1);
  lcd.setCursor(11, 1);
  lcd.print("C");
  lcd.setCursor(12, 1);
  lcd.print("|");
}
void updateLCD()
{
  lcd.clear();
  refreshLCD = false;
}
