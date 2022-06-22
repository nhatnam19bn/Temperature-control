//Khai bao thu vien
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
#define maxMinpoint 50
#define minMinpoint 0
#define maxMenu 5
#define minMenu 0
#define EEPROM_SIZE 512
#define addTarget 0
#define addRate 10
#define addDuration 20
#define addMinpoint 30
#define addDurationMode 40

//Khai bao cho LCD
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
byte runSignal[8] = {
  0B00100,
  0B01110,
  0B10101,
  0B00100,
  0B00100,
  0B00100,
  0B11111,
  0B00000
};
byte holdSignal[8] = {
  0B11111,
  0B11111,
  0B10101,
  0B10001,
  0B10101,
  0B11111,
  0B11111,
  0B00000
};
//Khai bao thong so PWM
int hCtrl = 27;
const int freq = 5000;
const int pwmChannel = 0;
const int resolution = 8;
//Khai bao cho module doc nhiet do
int thermoDO = 19;
int thermoCS = 5;
int thermoCLK = 18;
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

//Khai bao cho rate
double setpoint;
int minpoint;
int target;
int rate;
int duration;

//Khai bao bien nhiet do thuc te
double curtem;

//Khai bao thong so PID
double kp = 1, ki = 1, kd = 0.01;
double output = 0;
PID myPID(&curtem, &output, &setpoint, kp, ki, kd, DIRECT);

//Khai bao chan encoder/ nut nhan
int runButton = 32;
int setButton = 14;
int dt = 25;
int clk = 33;
bool encoderStatus;
bool clkDetec = false;
bool butStatus;
bool butDetec = false;
bool lastbutDetec = false;
bool refreshLCD = false;

//Khai bao Menu
bool menuStatus = false;
int menuNum = 0;
bool durationMenuNum;

//Khai bao trang thai menu
bool targetStatus;
bool rateStatus;
bool durationStatus;
bool minpointStatus;
bool durationModeStatus;

//Khai bao trang thai run/hold
bool runStatus;

//Khai bao trang thai duration OFF/ON
bool durationMode;

//Ham ngat cho encoder
void IRAM_ATTR isrEncoder()
{
  clkDetec = true;
}

//Ham ngat cho nut nhan encoder
void IRAM_ATTR isrsetButton()
{
  butDetec = !butDetec;
  refreshLCD = true;
}

//Ham ngat cho nut nhan runMode/holdMode
void IRAM_ATTR isrrunButton()
{
  runStatus = !runStatus;
  refreshLCD = true;
}

//Khai bao + ham timer0 doc nhiet do
hw_timer_t*timer0 = NULL;
void IRAM_ATTR tempRead()
{
  curtem = thermocouple.readCelsius()   ; //Hieu chinh sai so nhiet do o day
    Serial.println(curtem, 1);
}

//Khai bao + ham timer1 rate
hw_timer_t*timer1 = NULL;
void IRAM_ATTR rateUp()
{
  setpoint = setpoint + rate;
  setpoint = min(int(setpoint), target);
}

//Khai bao + ham timer2 duration
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
  Serial.begin(9600);
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
  lcd.createChar(1, degree);  //Tao ki hieu do
  lcd.createChar(2, runSignal);  //Tao ki hieu gia nhiet
  lcd.createChar(3, holdSignal);
  lcd.clear();
  Serial.println("MAX6675 test");
  // wait for MAX chip to stabilize
  delay(500);
  //Doc thong so duoc luu truoc do
//  EEPROM.begin(EEPROM_SIZE);
//  target = EEPROM.read(addTarget);
//  rate = EEPROM.read(addRate);
//  durationMode = EEPROM.read(addDurationMode);
//  duration = EEPROM.read(addDuration);
//  minpoint = EEPROM.read(addMinpoint);
  runStatus = false;
  //Khoi tao PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(0, 255);
//  ledcSetup(pwmChannel, freq, resolution);
//  ledcAttachPin(hCtrl, pwmChannel);

  //setup cho timer1 doc nhiet do sau 250ms
  timer0 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer0, tempRead, true);
  timerAlarmWrite(timer0, 300000, true); // For the MAX6675 to update, you must delay AT LEAST 250ms between reads!
  timerAlarmEnable(timer0);

  //setup cho timer1 rate
  timer1 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer1, rateUp, true);
  //  timerAlarmWrite(timer1, 10000000, true); // Tang rate sau 1 phut (vd 10s. 1phut~60,000,000)
  //  timerAlarmEnable(timer1);
  setpoint = minpoint;

  //setup cho timer2 duration
  timer2 = timerBegin(2, 80, true);
  timerAttachInterrupt(timer2, durationDown, true);
  //    timerAlarmWrite(timer2, 10000000, false); // Tang rate sau 1 phut
  //  timerAlarmEnable(timer2);
}

//==================================================//
//                      LOOP                        //
//==================================================//
void loop() {
  if (refreshLCD) {
    updateLCD();
  }
//  Serial.println(EEPROM.read(addDurationMode));
  //Nhan biet nhan nut encoder
  if ((butDetec ^ lastbutDetec) && menuStatus == false)
  {
    menuStatus = true;
    lastbutDetec = butDetec;
  }
  else
  {
    mainScreen();
    mainScreenDisplay();
  }
  //  attachInterrupt(setButton, isrsetButton, FALLING);
  //  attachInterrupt(runButton, isrrunButton, FALLING);
//  durationMode = EEPROM.read(addDurationMode);
  while (runStatus)
  {
    detachInterrupt(setButton);
    timerAlarmWrite(timer1, 5000000, runStatus);
    timerAlarmEnable(timer1);
    lcd.setCursor(7, 0);
    lcd.write(2);
    myPID.Compute();
//    ledcWrite(pwmChannel, output);
    if (durationMode == true) {
      if (setpoint == target )
      {
        timerAlarmWrite(timer2, 5000000, true);
        timerAlarmEnable(timer2);
      }
      else
      {
        timerAlarmWrite(timer2, 5000000, false);
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
  lcd.setCursor(7, 0);
  lcd.write(3);
  timerAlarmWrite(timer1, 5000000, false);
  timerAlarmDisable(timer1);

  //Che do cai dat cac thong so trong menu
  while (menuStatus)
  {
    detachInterrupt(runButton);
    encoderMenu();
    setModeDisplay();
    switch (menuNum) {
      case 0:
        if (butDetec ^ lastbutDetec)
        {
          menuStatus = false;
          lastbutDetec = butDetec;
        }
        break;
      case 1:
        if (butDetec ^ lastbutDetec)
        {
          targetStatus = true;
          lastbutDetec = butDetec;
        }
//        target = EEPROM.read(addTarget);
        while (targetStatus)
        {
          if (refreshLCD) {
            updateLCD();
          }
          setTarget();
          if (butDetec ^ lastbutDetec)
          {
            targetStatus = false;
            lastbutDetec = butDetec;
          }
        }
//        EEPROM.write(addTarget, target);
//        EEPROM.commit();
        setModeDisplay();
        break;
      case 2:
        if (butDetec ^ lastbutDetec)
        {
          rateStatus = true;
          lastbutDetec = butDetec;
        }
//        rate = EEPROM.read(addRate);
        while (rateStatus)
        {
          if (refreshLCD) {
            updateLCD();
          }
          setRate();
          if (butDetec ^ lastbutDetec)
          {
            rateStatus = false;
            lastbutDetec = butDetec;
          }
        }
//        EEPROM.write(addRate, rate);
//        EEPROM.commit();
        setModeDisplay();
        break;
      case 3:
        if (butDetec ^ lastbutDetec)
        {
          durationStatus = true;
          lastbutDetec = butDetec;
        }
//        durationMode = EEPROM.read(addDurationMode);
        durationMenuNum = durationMode;
        while (durationStatus)
        {
          if (refreshLCD) {
            updateLCD();
          }
          encoderDurationMenu();
          lcd.setCursor(1, 0);
          lcd.print("OFF            ");
          lcd.setCursor(1, 1);
          lcd.print("ON             ");
          if (durationMenuNum == 0) {
            lcd.setCursor(0, 0);
            lcd.print(">");
            lcd.setCursor(0, 1);
            lcd.print(" ");
          } else {
            lcd.setCursor(0, 1);
            lcd.print(">");
            lcd.setCursor(0, 0);
            lcd.print(" ");
          }
          switch (durationMenuNum) {
            case 0:
              if (butDetec ^ lastbutDetec)
              {
                //                durationStatus = false;
                durationMode = false;
//                EEPROM.write(addDurationMode, durationMode);
//                EEPROM.commit();
                durationStatus = false;
                lastbutDetec = butDetec;
              }
              break;
            case 1:
              if (butDetec ^ lastbutDetec)
              {
                durationModeStatus = true;
                lastbutDetec = butDetec;
              }
              while (durationModeStatus)
              {
                if (refreshLCD) {
                  updateLCD();
                }
                setDuration();
                if (butDetec ^ lastbutDetec)
                {
                  durationModeStatus = false;
                  durationStatus = false;
                  durationMode = true;
//                  EEPROM.write(addDurationMode, durationMode);
//                  EEPROM.write(addDuration, duration);
//                  EEPROM.commit();
                  lastbutDetec = butDetec;
                }
              }
              break;
          }
        }
        setModeDisplay();
        break;
      case 5:
        if (butDetec ^ lastbutDetec)
        {
          minpointStatus = true;
          lastbutDetec = butDetec;
        }
//        minpoint = EEPROM.read(addMinpoint);
        while (minpointStatus)
        {
          if (refreshLCD) {
            updateLCD();
          }
          setMinpoint();
          if (butDetec ^ lastbutDetec)
          {
            minpointStatus = false;
            lastbutDetec = butDetec;
          }
        }
//        EEPROM.write(addMinpoint, minpoint);
//        EEPROM.commit();
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
    if (digitalRead(dt) != encoderStatus)
    {
      target++;
      target = min(target, maxTarget);
    }
    else
    {
      target--;
      target = max(target, minTarget);
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
    if (digitalRead(dt) != encoderStatus)
    {
      rate++;
      rate = min(rate, maxRate);
    }
    else
    {
      rate--;
      rate = max(rate, minRate);
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
    if (digitalRead(dt) != encoderStatus)
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

//Cai dat toc do gia nhiet - Rate
void setMinpoint() {
  if (minpoint < 10)
  {
    lcd.setCursor(8, 0);
    lcd.print(minpoint);
    lcd.setCursor(7, 0);
    lcd.print(" ");
  }
  else
  {
    lcd.setCursor(7, 0);
    lcd.print(minpoint);
  }
  if (clkDetec && (!digitalRead(clk)))
  {
    if (digitalRead(dt) != encoderStatus)
    {
      minpoint++;
      minpoint = min(minpoint, maxMinpoint);
    }
    else
    {
      minpoint--;
      minpoint = max(minpoint, minMinpoint);
    }
    clkDetec = false;
  }
}
//Hien thi man hinh setMode
void setModeDisplay() {
  if (menuNum < 2) {
    lcd.setCursor(1, 0);
    lcd.print("Main Screen    ");
    lcd.setCursor(1, 1);
    lcd.print("Target         ");
    if (menuNum == 0) {
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print(" ");
    } else {
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(0, 0);
      lcd.print(" ");
    }
  } else if (2 <= menuNum && menuNum < 4) {
    lcd.setCursor(1, 0);
    lcd.print("Rate           ");
    lcd.setCursor(1, 1);
    lcd.print("Duration       ");
    if (menuNum == 2) {
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print(" ");
    } else {
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(0, 0);
      lcd.print(" ");
    }
  } else {
    lcd.setCursor(1, 0);
    lcd.print("P.I.D          ");
    lcd.setCursor(1, 1);
    lcd.print("MinPoint        ");
    if (menuNum == 4) {
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print(" ");
    } else {
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.setCursor(0, 0);
      lcd.print(" ");
    }
  }
}
void encoderMenu() {
  if (clkDetec && (!digitalRead(clk)))
  {
    if (digitalRead(dt) != encoderStatus)
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
    if (digitalRead(dt) != encoderStatus)
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
//==================================================//
//     Ham hien thi thong tin len man hinh chinh    //
//==================================================//
void mainScreenDisplay() {
  //Hien thi curtem
  if (curtem < 10)
  {
    lcd.setCursor(10, 0);
    lcd.print(curtem, 1);
  }
  else if (curtem < 100 && curtem >= 10)
  {
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
    lcd.print(target);
  }
  else if (target < 100 && target >= 10)
  {
    lcd.setCursor(8, 1);
    lcd.print(target);
  }
  else
  {
    lcd.setCursor(7, 1);
    lcd.print(target);
  }
  //Hien thi rate
  if (rate < 10)
  {
    lcd.setCursor(1, 1);
    lcd.print(rate);
  }
  else
  {
    lcd.setCursor(0, 1);
    lcd.print(rate);
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
